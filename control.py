"""
control.py
==========
Terminal control script for all RS-485 devices via the shared bus server.

Usage
-----
    # Terminal 1 — scalabrad manager
    # Terminal 2 — python bus_server.py
    # Terminal 3 — python control.py

Contact Burak Akel about any questions.
"""

import time
from labrad import connect


# ---------------------------------------------------------------------------
# Telemetry helpers
# ---------------------------------------------------------------------------

def parse_telemetry(s):
    """
    Parse the comma-separated telemetry string from pump_telemetry().
    Returns (speed_hz, current_a, temp_conv_c, temp_bear_c, dc_voltage_v, status_word)
    """
    parts = s.split(",")
    return (
        float(parts[0]),   # speed_hz
        float(parts[1]),   # current_a
        float(parts[2]),   # temp_conv_c
        float(parts[3]),   # temp_bear_c
        float(parts[4]),   # dc_voltage_v
        int(parts[5]),     # status_word
    )


def _parse_flags(zsw):
    flags = []
    if zsw & (1 << 2):  flags.append("RUNNING")
    if zsw & (1 << 0):  flags.append("READY")
    if zsw & (1 << 11): flags.append("TURNING")
    if zsw & (1 << 4):  flags.append("ACCEL")
    if zsw & (1 << 5):  flags.append("DECEL")
    if zsw & (1 << 3):  flags.append("FAULT!")
    if zsw & (1 << 7):  flags.append("TEMP_WARN")
    if zsw & (1 << 13): flags.append("OVERLOAD")
    return flags


def print_status(data):
    speed, current, temp_conv, temp_bear, voltage, zsw = data
    flags = _parse_flags(zsw)
    print(f"\n=== TurboVac 250i status ===")
    print(f"  State        : [{', '.join(flags) or 'IDLE'}]")
    print(f"  Rotor speed  : {speed:.1f} Hz")
    print(f"  Motor current: {current:.2f} A")
    print(f"  Conv temp    : {temp_conv:.1f} C")
    print(f"  Bearing temp : {temp_bear:.1f} C")
    print(f"  DC voltage   : {voltage:.1f} V")
    print(f"  Status word  : 0x{int(zsw):04X}\n")


HELP_TEXT = """
Available commands
------------------
  pump start              start the pump rotor
  pump stop               coast stop
  pump reset              reset error latch (pump must be stopped before)
  pump status             full telemetry snapshot
  pump monitor <n>        stream n samples at 1 s interval
                          use -1 for continuous, Ctrl-C to stop

  relay on  <n>           turn relay n ON  (Counting from 1)
  relay off <n>           turn relay n OFF (Counting from 1)
  relay toggle <n>        toggle relay n
  relay status <n>        get state of relay n
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
    dev = cxn.busserver
    print("Connected. Type 'help' for commands.\n")

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
                # Pump commands
                # -----------------------------------------------------------
                if command == "pump":
                    if not args:
                        print("Usage: pump <command>  |  type 'help'")
                        continue

                    sub = args[0].lower()

                    if sub == "start":
                        print(dev.pump_start())

                    elif sub == "stop":
                        print(dev.pump_stop())

                    elif sub == "reset":
                        print(dev.pump_reset())

                    elif sub == "status":
                        print_status(parse_telemetry(dev.pump_telemetry()))

                    elif sub == "monitor":
                        count = int(args[1]) if len(args) > 1 else 10
                        header = (
                            f"{'#':>5}  {'Speed(Hz)':>10}  {'I(A)':>8}  "
                            f"{'T_conv(C)':>10}  {'T_bear(C)':>10}  "
                            f"{'Vdc(V)':>8}  {'Flags':<30}"
                        )
                        print(header)
                        print("-" * len(header))
                        i = 0
                        try:
                            while count < 0 or i < count:
                                speed, current, t_conv, t_bear, vdc, zsw = \
                                    parse_telemetry(dev.pump_telemetry())
                                flags = _parse_flags(zsw)
                                print(
                                    f"{i:>5}  "
                                    f"{speed:>10.1f}  "
                                    f"{current:>8.2f}  "
                                    f"{t_conv:>10.1f}  "
                                    f"{t_bear:>10.1f}  "
                                    f"{vdc:>8.1f}  "
                                    f"{', '.join(flags) or 'IDLE':<30}"
                                )
                                i += 1
                                if count < 0 or i < count:
                                    time.sleep(1.0)
                        except KeyboardInterrupt:
                            print("\nMonitor stopped.")

                    else:
                        print(f"Unknown pump command: '{sub}'  |  type 'help'")

                # -----------------------------------------------------------
                # Relay commands
                # -----------------------------------------------------------
                elif command == "relay":
                    if not args:
                        print("Usage: relay <command> [n]  |  type 'help'")
                        continue

                    sub = args[0].lower()

                    def _n():
                        if len(args) < 2:
                            raise ValueError("Relay number required, e.g.: relay on 3")
                        return int(args[1])

                    if sub == "on":
                        print(dev.relay_on(_n()))

                    elif sub == "off":
                        print(dev.relay_off(_n()))

                    elif sub == "toggle":
                        print(dev.relay_toggle(_n()))

                    elif sub == "status":
                        n = _n()
                        print(f"Relay {n} : {dev.relay_status(n)}")

                    elif sub == "all_on":
                        print(dev.relay_all_on())

                    elif sub == "all_off":
                        print(dev.relay_all_off())

                    elif sub == "all_status":
                        raw = dev.relay_all_status()
                        for i, s in enumerate(raw.split(",")):
                            print(f"  Relay {i+1} : {s}")

                    elif sub == "demo":
                        print(dev.relay_demo())

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