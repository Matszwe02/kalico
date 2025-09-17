# Support for a peltier heater/cooler
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from .output_pin import GCodeRequestQueue
from heaters import Heater


KELVIN_TO_CELSIUS = -273.15
MAX_HEAT_TIME = 5.0
AMBIENT_TEMP = 25.0
PID_PARAM_BASE = 255.0
MAX_MAINTHREAD_TIME = 5.0
PID_PROFILE_VERSION = 1


class PrinterPeltier:
    def __init__(self, config):
        self.printer = config.get_printer()
        pheaters = self.printer.load_object(config, "heaters")
        self.heater = pheaters.setup_heater(config)
        self.get_status = self.heater.get_status
        self.stats = self.heater.stats
        # Register commands
        # gcode = self.printer.lookup_object("gcode")

        # self.heater.control

        ppins = self.printer.lookup_object("pins")
        self.relay_pin = None
        relay_pin_name = config.get('relay_pin', None)
        if relay_pin_name:
            self.relay_pin = ppins.setup_pin('digital_out', relay_pin_name)
            self.relay_pin.setup_max_duration(0.0)  # No duration for digital out
            self.relay_pin.setup_start_value(0, 0)  # Default to low (cooling mode)
            self.relay_gcrq = GCodeRequestQueue(config, self.relay_pin.get_mcu(), self._set_relay_pin)
        self.last_relay_state = -1  # Initialize to an invalid state
        self.polarity_hysteresis = config.getfloat('polarity_hysteresis', 1.0, minval=0.1)

        # Override the heater's control with our custom Peltier PID control
        # We need to pass the relay pin and hysteresis to the control algorithm
        old_control = self.heater.get_control()
        self.heater.set_control(ControlPeltierPID(
            old_control.get_profile(), self.heater,
            relay_pin=self.relay_pin,
            relay_gcrq=self.relay_gcrq,
            last_relay_state=self.last_relay_state,
            polarity_hysteresis=self.polarity_hysteresis
        ))


######################################################################
# Peltier Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 1.0
PID_SETTLE_SLOPE = 0.1


class ControlPeltierPID:
    def __init__(self, profile, heater, load_clean=False,
                 relay_pin=None, relay_gcrq=None, last_relay_state=-1,
                 polarity_hysteresis=1.0):
        self.profile = profile
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.relay_pin = relay_pin
        self.relay_gcrq = relay_gcrq
        self.last_relay_state = last_relay_state
        self.polarity_hysteresis = polarity_hysteresis
        self.Kp = profile["pid_kp"] / PID_PARAM_BASE
        self.Ki = profile["pid_ki"] / PID_PARAM_BASE
        self.Kd = profile["pid_kd"] / PID_PARAM_BASE
        self.min_deriv_time = (
            self.heater.get_smooth_time()
            if profile["smooth_time"] is None
            else profile["smooth_time"]
        )
        self.heater.set_inv_smooth_time(1.0 / self.min_deriv_time)
        self.temp_integ_max = 0.0
        if self.Ki:
            self.temp_integ_max = self.heater_max_power / self.Ki
        self.prev_temp = (
            AMBIENT_TEMP
            if load_clean
            else self.heater.get_temp(self.heater.reactor.monotonic())[0]
        )
        self.prev_temp_time = 0.0
        self.prev_temp_deriv = 0.0
        self.prev_temp_integ = 0.0

    def temperature_update(self, read_time, temp, target_temp):
        time_diff = read_time - self.prev_temp_time
        # Calculate change of temperature
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (
                self.prev_temp_deriv * (self.min_deriv_time - time_diff)
                + temp_diff
            ) / self.min_deriv_time
        # Calculate accumulated temperature "error"
        temp_err = target_temp - temp
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0.0, min(self.temp_integ_max, temp_integ))
        # Calculate output
        co = self.Kp * temp_err + self.Ki * temp_integ - self.Kd * temp_deriv
        # logging.debug("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%d",
        #    temp, read_time, temp_diff, temp_deriv, temp_err, temp_integ, co)
        bounded_co = max(0.0, min(self.heater_max_power, co))
        self.heater.set_pwm(read_time, bounded_co)
        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        return (
            abs(temp_diff) > PID_SETTLE_DELTA
            or abs(self.prev_temp_deriv) > PID_SETTLE_SLOPE
        )

    def update_smooth_time(self):
        self.smooth_time = self.heater.get_smooth_time()  # smoothing window

    def get_profile(self):
        return self.profile

    def get_type(self):
        return "pid"




def load_config(config):
    return PrinterPeltier(config)
