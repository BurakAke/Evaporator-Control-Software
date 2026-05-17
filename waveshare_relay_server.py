"""
waveshare_relay_server.py
=========================
LabRAD server for controlling only Waveshare Modbus RTU Relay (B) over RS-485.

Usage
-----
    # Terminal 1 — start scalabrad manager
    # Terminal 2 — start this server    python waveshare_relay_server.py
    # Terminal 3 — run control script   python control.py

Contact Burak Akel about any questions.
"""

from labrad.server import LabradServer, setting
from waveshare_relay import RelayModule


# ---------------------------------------------------------------------------
# Hardware configuration
# ---------------------------------------------------------------------------

RELAY_PORT      = "COM6"
RELAY_SLAVE_ID  = 1
RELAY_BAUD_RATE = 19200
RELAY_NUM       = 8


class WaveshareRelayServer(LabradServer):
    name = "relay"

    def initServer(self):
        self.relay = RelayModule(
            port       = RELAY_PORT,
            slave_id   = RELAY_SLAVE_ID,
            baud_rate  = RELAY_BAUD_RATE,
            num_relays = RELAY_NUM,
        )
        self.relay.connect()
        print(f"Relay module connected on {RELAY_PORT} (slave_id={RELAY_SLAVE_ID})")

    def stopServer(self):
        if self.relay and self.relay._serial and self.relay._serial.is_open:
            self.relay.disconnect()
            print("Relay module disconnected.")

    # ------------------------------------------------------------------
    # Single relay control
    # ------------------------------------------------------------------

    @setting(1, "relay_on", n="w", returns="s")
    def relay_on(self, c, n):
        """
        Turn relay n ON (Counting from 1).

        Parameters
        ----------
        n : Relay number, 1 to num_relays.
        """
        self.relay.on(n)
        return f"Relay {n} -> ON"

    @setting(2, "relay_off", n="w", returns="s")
    def relay_off(self, c, n):
        """
        Turn relay n OFF (Counting from 1).

        Parameters
        ----------
        n : Relay number, 1 to num_relays.
        """
        self.relay.off(n)
        return f"Relay {n} -> OFF"

    @setting(3, "relay_toggle", n="w", returns="s")
    def relay_toggle(self, c, n):
        """
        Toggle relay n based on its current state (Counting from 1).

        Parameters
        ----------
        n : Relay number, 1 to num_relays.
        """
        self.relay.toggle(n)
        return f"Relay {n} toggled."

    @setting(4, "relay_status", n="w", returns="b")
    def relay_status(self, c, n):
        """
        Get the current state of relay n (Counting from 1).
        Returns True if ON, False if OFF.
        """
        states = self.relay._read_coils(0, self.relay.num_relays)
        return bool(states[n - 1]) if states else False

    # ------------------------------------------------------------------
    # All relay control
    # ------------------------------------------------------------------

    @setting(5, "all_on", returns="s")
    def all_on(self, c):
        """Turn all relays ON."""
        self.relay.all_on()
        return "All relays -> ON"

    @setting(6, "all_off", returns="s")
    def all_off(self, c):
        """Turn all relays OFF."""
        self.relay.all_off()
        return "All relays -> OFF"

    @setting(7, "all_status", returns="*b")
    def all_status(self, c):
        """
        Get the state of all relays.
        Returns a list of booleans, index 0 = relay 1.
        True = ON, False = OFF.
        """
        states = self.relay._read_coils(0, self.relay.num_relays)
        return [bool(s) for s in states] if states else []

    @setting(8, "demo", returns="s")
    def demo(self, c):
        """Cycle all relays ON then OFF to verify wiring."""
        self.relay.demo()
        return "Demo complete."


if __name__ == "__main__":
    from labrad import util
    util.runServer(WaveshareRelayServer())