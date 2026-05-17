"""
bus_server.py
=============
Single LabRAD server that manages both the TurboVac 250i pump and the
Waveshare relay module through one shared RS-485 bus on a single COM port.

Bus server manages the port so that each device can use the port when necessary.
The COM port is initialized in the shared bus file.

Usage
-----
    # Terminal 1 — scalabrad manager
    # Terminal 2 — python bus_server.py
    # Terminal 3 — python control.py

Contact Burak Akel about any questions.
"""

from labrad.server import LabradServer, setting

from shared_bus      import SharedBus
from turbovac_250i   import TurboVac250i
from waveshare_relay import RelayModule


# ---------------------------------------------------------------------------
# Hardware configuration — only change PORT
# ---------------------------------------------------------------------------

PORT      = "COM6"
BAUD_RATE = 19200

PUMP_ADDRESS     = 0
PUMP_RETRIES     = 3
PUMP_RETRY_DELAY = 0.1

RELAY_SLAVE_ID  = 1
RELAY_NUM       = 8


class BusServer(LabradServer):
    """
    Single LabRAD server for all RS-485 devices on one shared COM port.
    """

    name = "busserver"

    def initServer(self):
        # Open the shared bus once — both devices use this single connection
        self.bus = SharedBus(port=PORT, baud_rate=BAUD_RATE)

        # Create device wrappers with the shared bus
        self.pump = TurboVac250i(
            address     = PUMP_ADDRESS,
            retries     = PUMP_RETRIES,
            retry_delay = PUMP_RETRY_DELAY,
            bus         = self.bus,
        )
        self.relay = RelayModule(
            slave_id   = RELAY_SLAVE_ID,
            num_relays = RELAY_NUM,
            baud_rate  = BAUD_RATE,
            bus        = self.bus,
        )

        self.pump.connect()
        self.relay.connect()

        print(f"Shared RS-485 bus open on {PORT}")
        print(f"  Pump  : address={PUMP_ADDRESS}")
        print(f"  Relay : slave_id={RELAY_SLAVE_ID}, {RELAY_NUM} channels")

    def stopServer(self):
        self.pump.disconnect()
        self.relay.disconnect()
        self.bus.close()
        print("Shared bus closed.")

    # ------------------------------------------------------------------
    # Pump settings
    # ------------------------------------------------------------------

    @setting(1, "pump_start", returns="s")
    def pump_start(self, c):
        """Start the pump rotor via fieldbus."""
        self.pump.start()
        return "Pump start command sent."

    @setting(2, "pump_stop", returns="s")
    def pump_stop(self, c):
        """Coast stop — pump decelerates to standstill."""
        self.pump.stop()
        return "Pump stop command sent."

    @setting(3, "pump_reset", returns="s")
    def pump_reset(self, c):
        """Reset the error latch. Pump must be stopped first."""
        self.pump.reset_error()
        return "Pump error reset sent."

    @setting(4, "pump_telemetry", returns="s")
    def pump_telemetry(self, c):
        """
        Full telemetry snapshot.

        Returns a comma-separated string:
            "speed_hz,current_a,temp_conv_c,temp_bear_c,dc_voltage_v,status_word"

        Parse in the client with:
            speed, current, t_conv, t_bear, vdc, zsw = parse_telemetry(dev.pump_telemetry())
        """
        t = self.pump.poll()
        return (
            f"{t.rotor_speed_hz:.3f},"
            f"{t.current_a:.3f},"
            f"{t.temperature_c:.3f},"
            f"{t.bearing_temp_c:.3f},"
            f"{t.dc_voltage_v:.3f},"
            f"{t.status_word}"
        )

    # ------------------------------------------------------------------
    # Relay settings
    # ------------------------------------------------------------------

    @setting(10, "relay_on", n="w", returns="s")
    def relay_on(self, c, n):
        """Turn relay n ON (Counting from 1)."""
        self.relay.on(n)
        return f"Relay {n} -> ON"

    @setting(11, "relay_off", n="w", returns="s")
    def relay_off(self, c, n):
        """Turn relay n OFF (Counting from 1)."""
        self.relay.off(n)
        return f"Relay {n} -> OFF"

    @setting(12, "relay_toggle", n="w", returns="s")
    def relay_toggle(self, c, n):
        """Toggle relay n (Counting from 1)."""
        self.relay.toggle(n)
        return f"Relay {n} toggled."

    @setting(13, "relay_status", n="w", returns="s")
    def relay_status(self, c, n):
        """Get state of relay n. Returns 'ON' or 'OFF'."""
        states = self.relay._read_coils(0, self.relay.num_relays)
        return "ON" if (states and states[n - 1]) else "OFF"

    @setting(14, "relay_all_on", returns="s")
    def relay_all_on(self, c):
        """Turn all relays ON."""
        self.relay.all_on()
        return "All relays -> ON"

    @setting(15, "relay_all_off", returns="s")
    def relay_all_off(self, c):
        """Turn all relays OFF."""
        self.relay.all_off()
        return "All relays -> OFF"

    @setting(16, "relay_all_status", returns="s")
    def relay_all_status(self, c):
        """
        Get state of all relays.
        Returns comma-separated string of ON/OFF values, e.g. "OFF,OFF,ON,OFF,..."
        Index 0 = relay 1.
        """
        states = self.relay._read_coils(0, self.relay.num_relays)
        if not states:
            return ""
        return ",".join("ON" if s else "OFF" for s in states)

    @setting(17, "relay_demo", returns="s")
    def relay_demo(self, c):
        """Cycle all relays ON then OFF to verify wiring."""
        self.relay.demo()
        return "Demo complete."


if __name__ == "__main__":
    from labrad import util
    util.runServer(BusServer())