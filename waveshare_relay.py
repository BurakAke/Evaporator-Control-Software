"""
waveshare_relay.py
==================
Python wrapper for the Waveshare Modbus RTU Relay (B) over RS485.

Comm setup parameters should be set in the server file.

Contact Burak Akel about any questions.
"""

import time
import math
import struct
import serial

# data check needed for the modbus protocol that the relay uses.
def _crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack("<H", crc)
    #converts crc into packs of two bytes using little endian encoding


class RelayModule:

    def __init__(self, port: str, slave_id: int = 1, num_relays: int = 8, baud_rate: int = 19200):
        self.port       = port
        self.slave_id   = slave_id
        self.num_relays = num_relays
        self.baud_rate  = baud_rate
        self._frame_gap = (3.5 * 11) / baud_rate
        self._serial    = None

    # -- Connection ------------------------------------------------------------

    def connect(self) -> None:
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            write_timeout=1,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        print(f"[OK] Connected on {self.port} (slave ID={self.slave_id}, baud={self.baud_rate})")

    def disconnect(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
            print("[OK] Disconnected.")

    # -- Relay control ---------------------------------------------------------

    def on(self, relay_num: int) -> None:
        """Turn ON a single relay (Counting from 1)."""
        self._validate(relay_num)
        self._write_coil(relay_num - 1, True)
        print(f"  Relay {relay_num} -> ON")

    def off(self, relay_num: int) -> None:
        """Turn OFF a single relay (Counting from 1)."""
        self._validate(relay_num)
        self._write_coil(relay_num - 1, False)
        print(f"  Relay {relay_num} -> OFF")

    def toggle(self, relay_num: int) -> None:
        """Toggle a single relay based on its current state."""
        self._validate(relay_num)
        states = self._read_coils(0, self.num_relays)
        if states:
            if states[relay_num - 1]:
                self.off(relay_num)
            else:
                self.on(relay_num)

    def all_on(self) -> None:
        """Turn ON all relays at once."""
        self._write_multiple_coils([True] * self.num_relays)
        print("  All relays -> ON")

    def all_off(self) -> None:
        """Turn OFF all relays at once."""
        self._write_multiple_coils([False] * self.num_relays)
        print("  All relays -> OFF")

    def status(self, relay_num: int):
        """Read and print current relay states. Returns list of bools (index 0 = relay 1)."""
        states = self._read_coils(0, self.num_relays)
        print(f"  Relay {relay_num}: {'ON' if states[relay_num-1] else 'OFF'}")
        return states[relay_num-1]
    
    def all_status(self) -> list:
        """Read and print current relay states. Returns list of bools (index 0 = relay 1)."""
        states = self._read_coils(0, self.num_relays)
        for i, s in enumerate(states):
            print(f"  Relay {i + 1}: {'ON' if s else 'OFF'}")
        return states

    def demo(self) -> None:
        """Cycle all relays ON then OFF to verify wiring."""
        print("\n--- ON one by one ---")
        for i in range(1, self.num_relays + 1):
            self.on(i)
            time.sleep(0.3)

        print("\n--- OFF one by one ---")
        for i in range(1, self.num_relays + 1):
            self.off(i)
            time.sleep(0.3)

        print("\n--- All ON -> All OFF ---")
        self.all_on()
        time.sleep(1)
        self.all_off()

    # -- Modbus frames (the actual data sent through serial to control the relay)--------------------

    def _write_coil(self, address: int, state: bool) -> None:
        payload = struct.pack(">BBHH", self.slave_id, 0x05, address, 0xFF00 if state else 0x0000)
        self._send(payload, response_length=8)

    def _write_multiple_coils(self, states: list) -> None:
        count      = len(states)
        byte_count = math.ceil(count / 8)
        coil_bytes = bytearray(byte_count)
        for i, s in enumerate(states):
            if s:
                coil_bytes[i // 8] |= (1 << (i % 8))
        payload = struct.pack(">BBHHB", self.slave_id, 0x0F, 0, count, byte_count) + bytes(coil_bytes)
        # builds the "message" for the module. Puts id,write/read,coil #,on/off state into register.
        self._send(payload, response_length=8)

    def _read_coils(self, start: int, count: int) -> list:
        payload  = struct.pack(">BBHH", self.slave_id, 0x01, start, count)
        response = self._send(payload, response_length=5 + math.ceil(count / 8))
        if not response:
            return []
        raw = response[3 : 3 + response[2]]
        return [bool((raw[i // 8] >> (i % 8)) & 0x01) for i in range(count)]

    # -- Serial ----------------------------------------------------------------

    def _send(self, payload: bytes, response_length: int) -> bytes | None:
        if not self._serial or not self._serial.is_open:
            raise RuntimeError("Not connected. Call connect() first.")
        
        # frame the relay input and send it
        frame = payload + _crc16(payload)
        time.sleep(self._frame_gap)
        self._serial.reset_input_buffer()
        self._serial.write(frame)
        self._serial.flush()

        # read relay's response throw error if response is not complete.
        response = self._serial.read(response_length)
        if len(response) < response_length:
            print(f"[ERROR] Expected {response_length} bytes, got {len(response)}.")
            return None

        # decode relay's response throw error if response is corrupted during transmit.
        body, recv_crc = response[:-2], response[-2:]
        if _crc16(body) != recv_crc:
            print(f"[ERROR] CRC mismatch: {response.hex(' ').upper()}")
            return None

        return response

    def _validate(self, relay_num: int) -> None:
        if not (1 <= relay_num <= self.num_relays):
            raise ValueError(f"relay_num must be 1-{self.num_relays}, got {relay_num}")