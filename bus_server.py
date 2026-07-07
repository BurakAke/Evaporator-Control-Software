"""
bus_server.py
=============
Single LabRAD server that manages both the TurboVac 250i pump and the
Waveshare relay module through one shared RS-485 bus on a single COM port.

Bus server manages the port so that each device can use the port when necessary.
The COM port is initialized in the shared bus file.

Current code has the pump control through relay so ignore the pump commands.

Usage
-----
    # Terminal 1 — scalabrad manager
    # Terminal 2 — python bus_server.py

Contact Burak Akel about any questions.
"""

from labrad.server import LabradServer, setting

from shared_bus      import SharedBus
from waveshare_relay import RelayModule


# ---------------------------------------------------------------------------
# Hardware configuration — only change PORT
# ---------------------------------------------------------------------------

PORT      = "COM6"
BAUD_RATE = 19200

RELAY_SLAVE_ID  = 1
RELAY_NUM       = 8
GATE_VALVE      = 7
TURBO_RELAY      = 7


class BusServer(LabradServer):
    """
    Single LabRAD server for all RS-485 devices on one shared COM port.
    """

    name = "busserver"

    def initServer(self):
        # Open the shared bus once — both devices use this single connection
        self.bus = SharedBus(port=PORT, baud_rate=BAUD_RATE)

        self.relay = RelayModule(
            slave_id   = RELAY_SLAVE_ID,
            num_relays = RELAY_NUM,
            baud_rate  = BAUD_RATE,
            bus        = self.bus,
        )

        self.pump.connect()
        self.relay.connect()

        print(f"Shared RS-485 bus open on {PORT}")
        print(f"  Relay : slave_id={RELAY_SLAVE_ID}, {RELAY_NUM} channels")

    def stopServer(self):
        self.relay.disconnect()
        self.bus.close()
        print("Shared bus closed.")


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
    
    # ------------------------------------------------------------------
    # Gate Valve Functions
    # ------------------------------------------------------------------

    @setting(20, "gate_valve_on", returns="s")
    def gate_valve_on(self, c):
        """Turn gate valve ON."""
        self.relay.on(GATE_VALVE)
        return f"Gate Valve -> ON"

    @setting(21, "gate_valve_off", returns="s")    
    def gate_valve_off(self, c):
        """Turn gate valve OFF."""
        self.relay.off(GATE_VALVE)
        return f"Gate Valve -> OFF"

    @setting(22, "gate_valve_status", returns="s")
    def gate_valve_status(self, c):
        """Get state of gate valve. Returns 'ON' or 'OFF'."""
        states = self.relay._read_coils(0, self.relay.num_relays)
        return "ON" if (states and states[GATE_VALVE - 1]) else "OFF"
    
    # ------------------------------------------------------------------
    # Turbo Pump Functions
    # ------------------------------------------------------------------

    @setting(30, "turbo_on", returns="s")
    def turbo_on(self, c):
        """Turn turbo pump ON."""
        self.relay.on(TURBO_RELAY)
        return f"Gate Valve -> ON"

    @setting(31, "turbo_off", returns="s")    
    def turbo_off(self, c):
        """Turn turbo pump OFF."""
        self.relay.off(TURBO_RELAY)
        return f"Gate Valve -> OFF"

    @setting(32, "turbo_status", returns="s")
    def turbo_status(self, c):
        """Get state of the turbo pump relay. Returns 'ON' or 'OFF'."""
        states = self.relay._read_coils(0, self.relay.num_relays)
        return "ON" if (states and states[TURBO_RELAY - 1]) else "OFF"

if __name__ == "__main__":
    from labrad import util
    util.runServer(BusServer())