"""
control.py
==========
Single terminal control script for the TurboVac 250i and Waveshare relay
via their LabRAD servers. This software is intended to control all the 
devices on the RS485 bus line connected to the AJA evaporator.

Usage
-----
    # Terminal 1 — start scalabrad manager
    # Terminal 3 — python waveshare_relay_server.py
    # Terminal 4 — python control.py

Contact Burak Akel about any questions.
"""

import time
from labrad import connect


HELP_TEXT = """
Available commands
------------------
  relay on  <n>           turn relay n ON  (Counting from 1)
  relay off <n>           turn relay n OFF (Counting from 1)
  relay toggle <n>        toggle relay n
  relay status <n>        get state of relay n (True=ON, False=OFF)
  relay all_on            turn all relays ON
  relay all_off           turn all relays OFF
  relay all_status        get state of all relays
  relay demo              cycle all relays to verify wiring

  help                    show this message
  quit                    exit
""".strip()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("Connecting to LabRAD manager...")
    cxn = connect("localhost", port=7682)

    relay = cxn.relay

    print("Connected to the servers. Type 'help' for commands.\n")

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break

            if not line:
                continue

            tokens  = line.split()
            command = tokens[0].lower()
            args    = tokens[1:]

            try:
                # -----------------------------------------------------------
                # Relay commands
                # -----------------------------------------------------------
                if command == "relay":
                    if not args:
                        print("Usage: relay <command> [n]  |  type 'help'")
                        continue

                    sub = args[0].lower()

                    def _n():   #checks if parameters are called correctly then sends in the right n
                        if len(args) < 2:
                            raise ValueError("Relay number required, e.g.: relay on 3")
                        return int(args[1])

                    if sub == "on":
                        print(relay.relay_on(_n()))

                    elif sub == "off":
                        print(relay.relay_off(_n()))

                    elif sub == "toggle":
                        print(relay.relay_toggle(_n()))

                    elif sub == "status":
                        n = _n()
                        state = relay.relay_status(n)
                        print(f"Relay {n} : {'ON' if state else 'OFF'}")

                    elif sub == "all_on":
                        print(relay.all_on())

                    elif sub == "all_off":
                        print(relay.all_off())

                    elif sub == "all_status":
                        states = relay.all_status()
                        for i, s in enumerate(states):
                            print(f"  Relay {i+1} : {'ON' if s else 'OFF'}")

                    elif sub == "demo":
                        print(relay.demo())

                    else:
                        print(f"Unknown relay command: '{sub}'  |  type 'help'")

                # -----------------------------------------------------------
                # General
                # -----------------------------------------------------------
                elif command == "help":
                    print(HELP_TEXT)

                elif command == "quit":
                    break

                else:
                    print(f"Unknown command: '{command}'  |  type 'help'")

            except KeyboardInterrupt:
                print()
                continue
            except Exception as e:
                print(f"[ERROR] {e}")

    except KeyboardInterrupt:
        print("\nCtrl-C pressed.")

    finally:
        cxn.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()