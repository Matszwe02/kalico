# Support for a peltier heater/cooler
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import os
import logging
import threading
from .output_pin import GCodeRequestQueue


KELVIN_TO_CELSIUS = -273.15
MAX_HEAT_TIME = 5.0
AMBIENT_TEMP = 25.0
PID_PARAM_BASE = 255.0
MAX_MAINTHREAD_TIME = 5.0
PID_PROFILE_VERSION = 1
PID_PROFILE_OPTIONS = {
    "pid_target": (float, "%.2f"),
    "pid_tolerance": (float, "%.4f"),
    "control": (str, "%s"),
    "smooth_time": (float, "%.3f"),
    "pid_kp": (float, "%.3f"),
    "pid_ki": (float, "%.3f"),
    "pid_kd": (float, "%.3f"),
}


class Peltier:
    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.short_name = short_name = self.name.split()[-1]
        self.reactor = self.printer.get_reactor()
        self.config = config
        self.configfile = self.printer.lookup_object("configfile")
        # Setup sensor
        self.sensor = sensor
        self.min_temp = config.getfloat("min_temp", minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat("max_temp", above=self.min_temp)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        self.pwm_delay = self.sensor.get_report_time_delta()
        self.max_power = config.getfloat(
            "max_power", 1.0, above=0.0, maxval=1.0
        )
        self.config_smooth_time = config.getfloat("smooth_time", 1.0, above=0.0)
        self.smooth_time = self.config_smooth_time
        self.inv_smooth_time = 1.0 / self.smooth_time
        self.verify_mainthread_time = -999.0
        self.lock = threading.Lock()
        self.last_temp = self.smoothed_temp = self.target_temp = 0.0
        self.last_temp_time = 0.0
        # pwm caching
        self.next_pwm_time = 0.0
        self.last_pwm_value = 0.0
        config.getfloat("pid_kp", None)
        config.getfloat("pid_ki", None)
        config.getfloat("pid_kd", None)
        config.getfloat("max_delta", None)

        self.last_relay_state = -1 # Initialize to an invalid state

        # Setup control algorithm sub-class
        self.control = ControlPID(self, config)

        # Setup output heater pin
        heater_pin = config.get("heater_pin")
        ppins = self.printer.lookup_object("pins")
        self.mcu_pwm = ppins.setup_pin("pwm", heater_pin)
        pwm_cycle_time = config.getfloat(
            "pwm_cycle_time", 0.100, above=0.0, maxval=self.pwm_delay
        )
        self.mcu_pwm.setup_cycle_time(pwm_cycle_time)
        self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)

        # Setup relay pin for polarity switching
        self.relay_pin = None
        relay_pin_name = config.get('relay_pin', None)
        if relay_pin_name:
            self.relay_pin = ppins.setup_pin('digital_out', relay_pin_name)
            self.relay_pin.setup_max_duration(0.) # No duration for digital out
            self.relay_pin.setup_start_value(0, 0) # Default to low (cooling mode)
            self.relay_gcrq = GCodeRequestQueue(config, self.relay_pin.get_mcu(),
                                                self._set_relay_pin)

        self.polarity_hysteresis = config.getfloat('polarity_hysteresis', 1.0, minval=0.)

        # Load additional modules
        # self.printer.load_object(config, "pid_calibrate")
        # self.printer.load_object(config, "verify_heater %s" % (short_name,))
        self.printer.load_object(config, "pid_calibrate")
        gcode = self.printer.lookup_object("gcode") # Re-add gcode lookup for command registration
        gcode.register_mux_command(
            "SET_PELTIER_TEMPERATURE",
            "PELTIER",
            short_name,
            self.cmd_SET_PELTIER_TEMPERATURE,
            desc=self.cmd_SET_PELTIER_TEMPERATURE_help,
        )
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )

    def set_pwm(self, read_time, value):
        if self.target_temp <= 0.0 or read_time > self.verify_mainthread_time:
            value = 0.0
        if (read_time < self.next_pwm_time or not self.last_pwm_value) and abs(
            value - self.last_pwm_value
        ) < 0.05:
            # No significant change in value - can suppress update
            return

        # Control relay pin based on target vs current temperature
        if self.relay_pin:
            new_relay_state = self.last_relay_state # Default to maintain current state
            if self.target_temp > self.smoothed_temp + self.polarity_hysteresis:
                # Heating mode
                new_relay_state = 1 # High for heating
            elif self.target_temp < self.smoothed_temp - self.polarity_hysteresis:
                # Cooling mode
                new_relay_state = 0 # Low for cooling

            self.relay_gcrq.queue_gcode_request(new_relay_state)

        pwm_time = read_time + self.pwm_delay
        self.next_pwm_time = pwm_time + 0.75 * MAX_HEAT_TIME
        self.last_pwm_value = value
        self.mcu_pwm.set_pwm(pwm_time, value)

    def temperature_callback(self, read_time, temp):
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            self.control.temperature_update(read_time, temp, self.target_temp)
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.0)
            self.smoothed_temp += temp_diff * adj_time

    def _handle_shutdown(self):
        self.verify_mainthread_time = -999.0
        if self.relay_pin:
            # Provide the current time as the first argument
            self.relay_pin.set_digital(self.printer.get_reactor().monotonic(), 0) # Ensure relay is off on shutdown
        if self.relay_pin:
            self.relay_gcrq.queue_gcode_request(0) # Ensure relay is off on shutdown

    def _set_relay_pin(self, print_time, value):
        if value == self.last_relay_state:
            return "discard", 0.0
        self.last_relay_state = value
        self.relay_pin.set_digital(print_time, value)

    # External commands
    def get_name(self):
        return self.name

    def get_pwm_delay(self):
        return self.pwm_delay

    def get_max_power(self):
        return self.max_power

    def get_smooth_time(self):
        return self.smooth_time

    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp)
            )
        with self.lock:
            self.target_temp = degrees

    def get_temp(self, eventtime):
        print_time = (
            self.mcu_pwm.get_mcu().estimated_print_time(eventtime) - 5.0
        )
        with self.lock:
            if self.last_temp_time < print_time:
                return 0.0, self.target_temp
            return self.smoothed_temp, self.target_temp

    def check_busy(self, eventtime):
        with self.lock:
            return self.control.check_busy(
                eventtime, self.smoothed_temp, self.target_temp
            )

    def set_control(self, control, keep_target=True):
        with self.lock:
            old_control = self.control
            self.control = control
            if not keep_target:
                self.target_temp = 0.0
        return old_control

    def get_control(self):
        return self.control

    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.target_temp = target_temp

    def stats(self, eventtime):
        est_print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime)
        if not self.printer.is_shutdown():
            self.verify_mainthread_time = est_print_time + MAX_MAINTHREAD_TIME
        with self.lock:
            target_temp = self.target_temp
            last_temp = self.last_temp
            last_pwm_value = self.last_pwm_value
            # Determine mode for stats based on temperature difference
            if target_temp > last_temp + self.polarity_hysteresis:
                mode_str = "heating"
            elif target_temp < last_temp - self.polarity_hysteresis:
                mode_str = "cooling"
            else:
                mode_str = "idle" # Or previous state, for now 'idle'
        is_active = target_temp or last_temp > 50.0
        return is_active, "%s: target=%.0f temp=%.1f pwm=%.3f mode=%s" % (
            self.short_name,
            target_temp,
            last_temp,
            last_pwm_value,
            mode_str,
        )
    def get_status(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            smoothed_temp = self.smoothed_temp
            last_pwm_value = self.last_pwm_value
            # Determine mode for status based on temperature difference
            # if target_temp > smoothed_temp + self.polarity_hysteresis:
            #     mode_str = "heating"
            # elif target_temp < smoothed_temp - self.polarity_hysteresis:
            #     mode_str = "cooling"
            # else:
            #     mode_str = "idle" # Or previous state, for now 'idle'
        return {'temperature': round(smoothed_temp, 2), 'target': target_temp,
                'power': last_pwm_value}

    cmd_SET_PELTIER_TEMPERATURE_help = "Sets a peltier temperature"
    def cmd_SET_PELTIER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float('TARGET', 0.0)
        printer_peltiers = self.printer.lookup_object('peltier')
        printer_peltiers.set_temperature(self, temp)


######################################################################
# Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 1.0
PID_SETTLE_SLOPE = 0.1


class ControlPID:
    def __init__(self, peltier, config):
        self.peltier = peltier
        self.peltier_max_power = peltier.get_max_power()
        self.Kp = config.getfloat('pid_Kp') / PID_PARAM_BASE
        self.Ki = config.getfloat('pid_Ki') / PID_PARAM_BASE
        self.Kd = config.getfloat('pid_Kd') / PID_PARAM_BASE
        self.min_deriv_time = peltier.get_smooth_time()
        self.temp_integ_max = 0.
        if self.Ki:
            self.temp_integ_max = self.peltier_max_power / self.Ki
        self.prev_temp = AMBIENT_TEMP
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
        # Determine if heating or cooling for PID error inversion
        is_heating_mode = target_temp > temp + self.peltier.polarity_hysteresis
        is_cooling_mode = target_temp < temp - self.peltier.polarity_hysteresis

        if is_cooling_mode: # Cooling mode
            temp_err = temp - target_temp # Invert error for cooling
        # If within hysteresis, maintain previous mode for PID calculation, or just use target_temp - temp

        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0.0, min(self.temp_integ_max, temp_integ))
        # Calculate output
        co = self.Kp * temp_err + self.Ki * temp_integ - self.Kd * temp_deriv
        bounded_co = max(0.0, min(self.peltier_max_power, co))
        self.peltier.set_pwm(read_time, bounded_co)
        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        # Determine if heating or cooling for check_busy
        is_heating_mode = target_temp > smoothed_temp + self.peltier.polarity_hysteresis
        is_cooling_mode = target_temp < smoothed_temp - self.peltier.polarity_hysteresis

        if is_cooling_mode: # Cooling mode
            temp_diff = smoothed_temp - target_temp # Invert for cooling
        # If within hysteresis, maintain previous mode for check_busy, or just use target_temp - smoothed_temp
        return (abs(temp_diff) > PID_SETTLE_DELTA
                or abs(self.prev_temp_deriv) > PID_SETTLE_SLOPE)


######################################################################
# Sensor and peltier lookup
######################################################################

class PrinterPeltiers:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.sensor_factories = {}
        self.peltiers = {}
        self.gcode_id_to_sensor = {}
        self.available_peltiers = []
        self.available_sensors = []
        self.available_monitors = []
        self.has_started = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "gcode:request_restart", self.turn_off_all_peltiers
        )
        # Load default temperature sensors (this will populate the global sensor_factories in heaters.py)
        pconfig = self.printer.lookup_object("configfile")
        dir_name = os.path.dirname(__file__)
        filename = os.path.join(dir_name, "temperature_sensors.cfg")
        try:
            dconfig = pconfig.read_config(filename)
        except Exception:
            logging.exception("Unable to load temperature_sensors.cfg")
            raise config.error("Cannot load config '%s'" % (filename,))
        for c in dconfig.get_prefix_sections(""):
            self.printer.load_object(dconfig, c.get_name())

    def load_config(self, config):
        # This load_config is for the [peltier] section (module name) and returns the PrinterPeltiers object
        return PrinterPeltiers(config)


    def setup_peltier(self, config, gcode_id=None):
        peltier_name = config.get_name().split()[-1]
        if peltier_name in self.peltiers:
            raise config.error("Peltier %s already registered" % (peltier_name,))

        # Get sensor factories from the heaters object
        pheaters = self.printer.lookup_object('heaters')

        # Create sensor
        sensor_type = config.get('sensor_type') # Get sensor_type from the main peltier config
        if sensor_type not in pheaters.sensor_factories:
            raise self.printer.config_error(
                "Unknown temperature sensor '%s'" % (sensor_type,))
        sensor = pheaters.sensor_factories[sensor_type](config) # Pass the main peltier config directly

        # Create peltier
        self.peltiers[peltier_name] = peltier = Peltier(config, sensor)
        self.register_sensor(config, sensor, gcode_id) # Register the single sensor
        self.available_peltiers.append(config.get_name())
        return peltier
    def get_all_peltiers(self):
        return self.available_peltiers
    def lookup_peltier(self, peltier_name):
        if peltier_name not in self.peltiers:
            raise self.printer.config_error(
                "Unknown peltier '%s'" % (peltier_name,))
        return self.peltiers[peltier_name]
    def register_sensor(self, config, psensor, gcode_id=None):
        self.available_sensors.append(config.get_name())
        if gcode_id is None:
            gcode_id = config.get('gcode_id', None)
            if gcode_id is None:
                return
        if gcode_id in self.gcode_id_to_sensor:
            raise self.printer.config_error(
                "G-Code sensor id %s already registered" % (gcode_id,))
        self.gcode_id_to_sensor[gcode_id] = psensor
    def register_monitor(self, config):
        self.available_monitors.append(config.get_name())
    def get_status(self, eventtime):
        return {'available_peltiers': self.available_peltiers,
                'available_sensors': self.available_sensors,
                'available_monitors': self.available_monitors}
    def turn_off_all_peltiers(self, print_time=0.):
        for peltier in self.peltiers.values():
            peltier.set_temp(0.0)
            peltier.set_mode('cooling')
    cmd_TURN_OFF_PELTIERS_help = "Turn off all peltiers"
    def cmd_TURN_OFF_PELTIERS(self, gcmd):
        self.turn_off_all_peltiers()
    # G-Code M105 temperature reporting
    def _handle_ready(self):
        self.has_started = True
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("TURN_OFF_PELTIERS", self.cmd_TURN_OFF_PELTIERS,
                               desc=self.cmd_TURN_OFF_PELTIERS_help)
    def _get_temp(self, eventtime):
        # Tn:XXX /YYY B:XXX /YYY
        out = []
        if self.has_started:
            for gcode_id, sensor in sorted(self.gcode_id_to_sensor.items()):
                cur, target = sensor.get_temp(eventtime)
                out.append("%s:%.1f /%.1f" % (gcode_id, cur, target))
        if not out:
            return "T:0"
        return " ".join(out)
    def cmd_M105(self, gcmd):
        # Get Extruder Temperature
        reactor = self.printer.get_reactor()
        msg = self._get_temp(reactor.monotonic())
        did_ack = gcmd.ack(msg)
        if not did_ack:
            gcmd.respond_raw(msg)
    def _wait_for_temperature(self, peltier):
        # Helper to wait on peltier.check_busy() and report M105 temperatures
        if self.printer.get_start_args().get("debugoutput") is not None:
            return
        toolhead = self.printer.lookup_object("toolhead")
        gcode = self.printer.lookup_object("gcode")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        while not self.printer.is_shutdown() and peltier.check_busy(eventtime):
            print_time = toolhead.get_last_move_time()
            gcode.respond_raw(self._get_temp(eventtime))
            eventtime = reactor.pause(eventtime + 1.)
    def set_temperature(self, peltier, temp, wait=False):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt: None))
        peltier.set_temp(temp)
        if wait and temp:
            self._wait_for_temperature(peltier)
    cmd_TEMPERATURE_WAIT_help = "Wait for a temperature on a sensor"
    def cmd_TEMPERATURE_WAIT(self, gcmd):
        sensor_name = gcmd.get("SENSOR")
        if sensor_name not in self.available_sensors:
            raise gcmd.error("Unknown sensor '%s'" % (sensor_name,))
        min_temp = gcmd.get_float("MINIMUM", float("-inf"))
        max_temp = gcmd.get_float("MAXIMUM", float("inf"), above=min_temp)
        error_on_cancel = gcmd.get("ALLOW_CANCEL", None) is None
        if min_temp == float("-inf") and max_temp == float("inf"):
            raise gcmd.error(
                "Error on 'TEMPERATURE_WAIT': missing MINIMUM or MAXIMUM."
            )
        if self.printer.get_start_args().get("debugoutput") is not None:
            return
        if sensor_name in self.peltiers:
            sensor = self.peltiers[sensor_name].sensor # Use primary sensor for wait
        else:
            sensor = self.printer.lookup_object(sensor_name)
        toolhead = self.printer.lookup_object("toolhead")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        gcmd.respond_info("Waiting for sensor '%s' to reach temperature between %.1f and %.1f." % (sensor_name, min_temp, max_temp))
        while not self.printer.is_shutdown():
            temp, target = sensor.get_temp(eventtime)
            if temp >= min_temp and temp <= max_temp:
                return
            print_time = toolhead.get_last_move_time()
            gcmd.respond_raw(self._get_temp(eventtime))
            eventtime = reactor.pause(eventtime + 1.)

def load_config(config):
    # This load_config is for the [peltier] section (module name) and returns the PrinterPeltiers object
    return PrinterPeltiers(config)

def load_config_prefix(config):
    # This load_config_prefix is for [peltier <name>] sections
    ppeltiers = config.get_printer().load_object(config, 'peltier')
    return ppeltiers.setup_peltier(config)
