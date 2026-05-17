"""
pump_main.py
=======
Software to control Turbovac 250i parameters.

Edit the configuration block below to match your hardware, then run:

    python pump_main.py

This program should only be operated to change parameters such as speed 
or comm parameters on device side.
To start and stop the pump use the labrad servers.

Contact Burak Akel about any questions.
"""

import logging
import time

from turbovac_250i import TurboVac250i, print_status, make_csv_callback


# ---------------------------------------------------------------------------
# Device configuration — edit these to match your hardware setup
# ---------------------------------------------------------------------------

# TurboVac 250i
PUMP_PORT        = "COM6"   # serial port e.g. "COM3" on Windows
PUMP_ADDRESS     = 0        # RS-485 device address (0-31, factory default 0)
PUMP_BAUD_RATE   = 19200    # fixed by firmware — only change if Leybold says so
PUMP_RETRIES     = 3        # telegram retry attempts on failure
PUMP_RETRY_DELAY = 0.1      # seconds between retries

# Monitoring
MONITOR_INTERVAL = 1.0      # seconds between telemetry polls
MONITOR_COUNT    = 30       # number of samples (-1 = run until Ctrl-C)
MONITOR_CSV      = None     # set to a file path e.g. "pump_log.csv" to enable

# Feature flags — set to False to skip that section
ENABLE_PUMP      = True
ENABLE_MONITOR   = False

# Logging level — set to logging.DEBUG for full telegram traces
LOG_LEVEL        = logging.INFO

# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

logging.basicConfig(
    level=LOG_LEVEL,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":

    if ENABLE_PUMP:
        on_data = make_csv_callback(MONITOR_CSV) if MONITOR_CSV else None

        pump = TurboVac250i(
            port        = PUMP_PORT,
            address     = PUMP_ADDRESS,
            baud_rate   = PUMP_BAUD_RATE,
            retries     = PUMP_RETRIES,
            retry_delay = PUMP_RETRY_DELAY,
        )
        pump.connect()

        print_status(pump)

        pump.set_speed(1000)

        # pump.start()

        if ENABLE_MONITOR:
            pump.monitor(
                interval = MONITOR_INTERVAL,
                count    = MONITOR_COUNT,
                on_data  = on_data,
            )

        # pump.stop()

        pump.disconnect()

    else:
        print("Pump control is not enabled.")