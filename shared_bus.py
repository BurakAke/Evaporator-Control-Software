"""
shared_bus.py
=============
Shared RS-485 bus manager for multiple devices on a single COM port.

Owns the one serial.Serial instance and a threading.Lock. Every device
transaction (write + read) acquires the lock once called so only one 
device talks at a time. Both TurboVac250i and RelayModule accept a 
SharedBus instance instead of opening their own serial port.

Contact Burak Akel about any questions.
"""

import threading
import time
import serial


class SharedBus:
    """
    Parameters (change in the bus_server)
    ----------
    port      : Serial port, e.g. "COM6".
    baud_rate : Baud rate. Both devices must match (default 19200).
    timeout   : Read timeout in seconds (default 0.5).
    """

    def __init__(self, port: str, baud_rate: int = 19200, timeout: float = 0.5):
        self._lock = threading.Lock()
        self._ser  = serial.Serial(
            port     = port,
            baudrate = baud_rate,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_EVEN,   # required by pump relay reconfigured
            stopbits = serial.STOPBITS_ONE,
            timeout  = timeout,
        )

    @property
    def is_open(self) -> bool:
        return self._ser.is_open

    def transact(
        self,
        frame       : bytes,
        read_length : int,
        pre_delay   : float = 0.0,
        post_delay  : float = 0.05,
    ) -> bytes:
        """
        Acquire the bus lock, send a frame, and read the response.

        Parameters
        ----------
        frame       : Raw bytes to send.
        read_length : Number of bytes to read back.
        pre_delay   : Sleep before writing  — used by Modbus for the 3.5
                      character inter-frame gap.
        post_delay  : Sleep after writing   — used by USS to give the pump
                      time to prepare its response (50 ms).

        Returns
        -------
        Raw response bytes (may be shorter than read_length on timeout).
        """
        with self._lock:    #makes sure any data transaction is complete and then locks the port
            if pre_delay > 0:
                time.sleep(pre_delay)
            self._ser.reset_input_buffer()
            self._ser.write(frame)
            if post_delay > 0:
                time.sleep(post_delay)
            return self._ser.read(read_length)

    def close(self):
        """Close the serial port."""
        if self._ser.is_open:
            self._ser.close()