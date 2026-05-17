"""
turbovac_250i_server.py
=======================
LabRAD server for the controlling only Leybold TurboVac 250i turbopump over RS-485.

Wraps turbovac_250i.py and exposes all pump functions as LabRAD settings.

Setup
-----
1. Start the LabRAD manager.
2. Run this server:
       python turbovac_250i_server.py

Registry keys
-------------
    port        Serial port string  e.g. "COM6"
    address     RS-485 device address (int, default 0 — do not change)
    baud_rate   Baud rate (int, default 19200 — do not change)

Setting numbers
---------------
    10  connect
    11  disconnect
    20  start
    21  stop
    22  reset_error
    30  get_telemetry       all PZD fields in one call
    31  rotor_speed
    32  motor_current
    33  temperature
    34  bearing_temperature
    35  dc_voltage
    36  warning_code
    37  rs485_address
    50  start_monitor       begin firing telemetry_signal at interval
    51  stop_monitor        stop the monitor loop

Signal
------
    543210  telemetry_signal    fired each monitor tick
            type: (v, v, v, v, v, w)
            fields: speed_hz, current_a, temp_conv_c, temp_bear_c,
                    dc_voltage_v, status_word

Contact Burak Akel about any questions.
"""

import logging

from labrad.server import LabradServer, setting, Signal
from labrad.units import Value
from twisted.internet.defer import inlineCallbacks, returnValue
from twisted.internet.task import LoopingCall
from twisted.internet.threads import deferToThread

from turbovac_250i import TurboVac250i, PumpTelemetry

log = logging.getLogger(__name__)

# Registry path where config is stored
_REG_PATH = ['', 'Servers', 'TurboVac250i']

# Defaults written to registry on first run
_DEFAULTS = {
    'port'      : 'COM6',
    'address'   : 0,
    'baud_rate' : 19200,
}


class TurboVac250iServer(LabradServer):
    """LabRAD server for the Leybold TurboVac 250i turbopump."""

    name = 'TurboVac250i'

    # Signal fired on every monitor tick.
    # Tuple: (speed_hz, current_a, temp_conv_c, temp_bear_c, dc_voltage_v, status_word)
    telemetry_signal = Signal(543210, 'signal: telemetry', '(vvvvvw)')

    def __init__(self):
        super().__init__()
        self._pump          = None
        self._monitor_loop  = None
        self._port          = _DEFAULTS['port']
        self._address       = _DEFAULTS['address']
        self._baud_rate     = _DEFAULTS['baud_rate']

    # ------------------------------------------------------------------
    # Server lifecycle
    # ------------------------------------------------------------------

    @inlineCallbacks
    def initServer(self):
        """Load config from registry on startup."""
        log.info("Loading config from registry...")
        yield self._load_config()
        log.info(
            "TurboVac250i server ready. port=%s address=%d",
            self._port, self._address,
        )

    @inlineCallbacks
    def stopServer(self):
        """Clean up on shutdown — stop monitor and disconnect pump."""
        if self._monitor_loop and self._monitor_loop.running:
            self._monitor_loop.stop()
        if self._pump and self._pump.is_connected:
            yield deferToThread(self._pump.disconnect)

    @inlineCallbacks
    def _load_config(self):
        """
        Read config from the LabRAD registry.
        Missing keys are created with default values so the user knows
        what to edit on first run.
        """
        reg = self.client.registry()
        yield reg.cd(_REG_PATH, True)   # True = create path if missing

        for key, default in _DEFAULTS.items():
            try:
                value = yield reg.get(key)
                setattr(self, f'_{key}', value)
                log.info("  registry: %s = %s", key, value)
            except Exception:
                yield reg.set(key, default)
                log.info("  registry: %s not found, created with default %s", key, default)

    # ------------------------------------------------------------------
    # Connection  (settings 10-11)
    # ------------------------------------------------------------------

    @setting(10, returns='s')
    @inlineCallbacks
    def connect(self, c):
        """
        Connect to the pump using the port and address from the registry.
        Returns a status string.
        """
        self._pump = TurboVac250i(
            port        = self._port,
            address     = self._address,
            baud_rate   = self._baud_rate,
        )
        yield deferToThread(self._pump.connect)
        returnValue(f"Connected to TurboVac 250i on {self._port} (address={self._address})")

    @setting(11, returns='s')
    @inlineCallbacks
    def disconnect(self, c):
        """Disconnect from the pump. Returns a status string."""
        self._check_connected()
        if self._monitor_loop and self._monitor_loop.running:
            self._monitor_loop.stop()
        yield deferToThread(self._pump.disconnect)
        returnValue(f"Disconnected from {self._port}")

    # ------------------------------------------------------------------
    # Pump control  (settings 20-22)
    # ------------------------------------------------------------------

    @setting(20, returns='s')
    @inlineCallbacks
    def start(self, c):
        """
        Start the pump via fieldbus.
        Asserts process data channel (bit 10) then sends start word (bits 0+6+10).
        Pump must be fault-free.
        """
        self._check_connected()
        yield deferToThread(self._pump.start)
        returnValue("Start command sent.")

    @setting(21, returns='s')
    @inlineCallbacks
    def stop(self, c):
        """
        Coast stop — pump decelerates to standstill under its own control.
        """
        self._check_connected()
        yield deferToThread(self._pump.stop)
        returnValue("Stop command sent.")

    @setting(22, returns='s')
    @inlineCallbacks
    def reset_error(self, c):
        """
        Reset the error latch. Pump must be stopped before calling this.
        """
        self._check_connected()
        yield deferToThread(self._pump.reset_error)
        returnValue("Error reset sent.")

    # ------------------------------------------------------------------
    # Telemetry  (settings 30-37)
    # ------------------------------------------------------------------

    @setting(30, returns='(vvvvvw)')
    @inlineCallbacks
    def get_telemetry(self, c):
        """
        Poll all live process data in one call.

        Returns a tuple:
            (speed_hz, current_a, temp_conv_c, temp_bear_c, dc_voltage_v, status_word)

        This is the most efficient way to read the pump state — one RS-485
        round trip returns everything.
        """
        self._check_connected()
        t = yield deferToThread(self._pump.poll)
        returnValue((
            t.rotor_speed_hz,
            t.current_a,
            t.temperature_c,
            t.bearing_temp_c,
            t.dc_voltage_v,
            t.status_word,
        ))

    @setting(31, returns='v[Hz]')
    @inlineCallbacks
    def rotor_speed(self, c):
        """Actual rotor speed in Hz (PZD2)."""
        self._check_connected()
        val = yield deferToThread(self._pump.get_rotor_speed)
        returnValue(Value(val, 'Hz'))

    @setting(32, returns='v[A]')
    @inlineCallbacks
    def motor_current(self, c):
        """Motor current in Amperes (PZD4 x 0.1)."""
        self._check_connected()
        val = yield deferToThread(self._pump.get_motor_current)
        returnValue(Value(val, 'A'))

    @setting(33, returns='v[degC]')
    @inlineCallbacks
    def temperature(self, c):
        """Frequency-converter temperature in degrees Celsius (PZD3)."""
        self._check_connected()
        val = yield deferToThread(self._pump.get_temperature)
        returnValue(Value(val, 'degC'))

    @setting(34, returns='v[degC]')
    @inlineCallbacks
    def bearing_temperature(self, c):
        """Bearing temperature in degrees Celsius (PZD5)."""
        self._check_connected()
        val = yield deferToThread(self._pump.get_bearing_temperature)
        returnValue(Value(val, 'degC'))

    @setting(35, returns='v[V]')
    @inlineCallbacks
    def dc_voltage(self, c):
        """DC intermediate circuit voltage in Volts (PZD6)."""
        self._check_connected()
        val = yield deferToThread(self._pump.get_dc_voltage)
        returnValue(Value(val, 'V'))

    @setting(36, returns='w')
    @inlineCallbacks
    def warning_code(self, c):
        """Active warning code (P227). Returns 0 if no warning."""
        self._check_connected()
        val = yield deferToThread(self._pump.get_warning_code)
        returnValue(val)

    @setting(37, returns='w')
    @inlineCallbacks
    def rs485_address(self, c):
        """Configured RS-485 device address (P37)."""
        self._check_connected()
        val = yield deferToThread(self._pump.get_rs485_address)
        returnValue(val)

    # ------------------------------------------------------------------
    # Monitor  (settings 50-51)
    # ------------------------------------------------------------------

    @setting(50, interval='v[s]', returns='s')
    def start_monitor(self, c, interval=Value(1.0, 's')):
        """
        Start firing telemetry_signal at the given interval.

        Each tick polls the pump and broadcasts a telemetry tuple to all
        subscribed clients:
            (speed_hz, current_a, temp_conv_c, temp_bear_c, dc_voltage_v, status_word)

        To receive the signal in a client:
            cxn.turbovac250i.signal__telemetry(543210)
            cxn.turbovac250i.addListener(handler, source=..., ID=543210)

        Default interval is 1 second.
        """
        self._check_connected()
        if self._monitor_loop and self._monitor_loop.running:
            self._monitor_loop.stop()
        secs = interval['s']
        self._monitor_loop = LoopingCall(self._monitor_tick)
        self._monitor_loop.start(secs, now=True)
        return f"Monitor started at {secs:.1f} s interval."

    @setting(51, returns='s')
    def stop_monitor(self, c):
        """Stop the telemetry monitor loop."""
        if self._monitor_loop and self._monitor_loop.running:
            self._monitor_loop.stop()
            return "Monitor stopped."
        return "Monitor was not running."

    @inlineCallbacks
    def _monitor_tick(self):
        """
        Called by LoopingCall on each monitor interval.
        Polls the pump in a thread (non-blocking) then fires the signal.
        """
        try:
            t = yield deferToThread(self._pump.poll)
            self.telemetry_signal((
                t.rotor_speed_hz,
                t.current_a,
                t.temperature_c,
                t.bearing_temp_c,
                t.dc_voltage_v,
                t.status_word,
            ))
        except Exception as e:
            log.warning("Monitor tick failed: %s", e)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _check_connected(self):
        """Raise if the pump is not connected."""
        if self._pump is None or not self._pump.is_connected:
            raise Exception(
                "Pump not connected. Call 'connect' first."
            )


__server__ = TurboVac250iServer()

if __name__ == '__main__':
    from labrad import util
    util.runServer(__server__)
