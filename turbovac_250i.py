"""
turbovac_250i.py
================
Python wrapper for the Leybold TurboVac 250i (and all TURBOVAC i/iX series)
over RS-485 using the Siemens USS protocol (VDI/VDE 3689).

Main control software and comm setup parameters are given in main.py.
Contact Burak Akel about any questions.

------------------------------------------------------------------------------
SOURCE DOCUMENT
------------------------------------------------------------------------------
All protocol details, telegram layout, control/status bit definitions, AK
designators, and parameter numbers in this file are taken directly from:

    Leybold GmbH
    "Serial Interfaces for TURBOVAC i/iX —
     RS 232, RS 485, Profibus, Profinet, USB"
    Operating Instructions, Document No. 300450826_002_C2
    Revision C2, September 2020

Specific sections referenced:
    Sec. 1.1  — RS-485 hardware / wiring / pin assignments
    Sec. 3.1  — USS telegram layout (Table, p. 22)
    Sec. 4.1  — PKE field and AK task/reply designator table (p. 24)
    Sec. 4.3  — USS Control Word bit definitions (p. 26)
    Sec. 4.4  — USS Status Word bit definitions (p. 26)
    Sec. 5    — Parameter list
    Annex     — Example telegrams (pp. 61-65)

------------------------------------------------------------------------------
HARDWARE WIRING  (Sec. 1.1, connector X104 on TURBOVAC-i basic model)
------------------------------------------------------------------------------
    Pin 7  ->  TxD/RxD+  (A line)
    Pin 8  ->  TxD/RxD-  (B line)
    Pin 5  ->  GND
    Housing -> Cable shield
    Place a 120 ohm termination resistor at both ends of the bus.

RS-485 settings (all fixed by firmware, Sec. 1.1 Technical Data table)
------------------------------------------------------------------------
    Baud rate  : 19200  (fixed)
    Data bits  : 8
    Stop bits  : 1
    Parity     : Even
    Flow ctrl  : None
    Max length : 100 m, 2-wire twisted pair
    Response delay: 10 ms minimum

------------------------------------------------------------------------------
USS TELEGRAM LAYOUT — 24 bytes  (Sec. 3.1, p. 22)
------------------------------------------------------------------------------
    Byte  0     STX  = 0x02
    Byte  1     LGE  = 0x16  (payload = 22 bytes)
    Byte  2     ADR  device address (0-31, default 0)
    Bytes 3-4   PKE  parameter ID + access type (big-endian)
    Byte  5     0x00 reserved
    Byte  6     IND  parameter index (0 = scalar)
    Bytes 7-10  PWE  parameter value (32-bit big-endian)
    Bytes 11-12 PZD1 control word STW / status word ZSW
    Bytes 13-14 PZD2 speed setpoint HSW / actual rotor freq HIW  [Hz]
    Bytes 15-16 PZD3 freq-converter temperature  [deg C]  (= P11)
    Bytes 17-18 PZD4 motor current x 0.1  [A]             (= P5)
    Bytes 19-20 PZD5 bearing temperature  [deg C]          (= P125)
    Bytes 21-22 PZD6 DC link voltage      [V]              (= P4)
    Byte  23    BCC  XOR checksum of bytes 0-22

------------------------------------------------------------------------------
PKE FIELD  (Sec. 4.1, p. 24)
------------------------------------------------------------------------------
    Bits 15-12  AK  task / reply designator
    Bits 11-0   PNU parameter number

AK TASK DESIGNATORS (master -> pump)  — Sec. 4.1 table, p. 24
---------------------------------------------------------------
    0x0  No task (poll PZD process data only, no param access)
    0x1  Request parameter value  (read; pump replies 0x1 for 16-bit,
                                          0x2 for 32-bit)
    0x2  Write 16-bit parameter value
    0x3  Write 32-bit parameter value
    0x6  Request field (indexed) parameter value
    0x7  Write 16-bit field value
    0x8  Write 32-bit field value

AK REPLY DESIGNATORS (pump -> master)  — Sec. 4.1 table, p. 24
----------------------------------------------------------------
    0x1  16-bit value transferred
    0x2  32-bit value transferred
    0x4  16-bit field value transferred
    0x5  32-bit field value transferred
    0x7  Cannot execute task  (PWE contains error code below)
    0x8  No write permission

Pump error codes in PWE when AK=0x7  (Sec. 4.1, p. 25)
---------------------------------------------------------
    0   Impermissible parameter number
    1   Parameter cannot be changed
    2   Lower or upper threshold exceeded
    3   Faulty index
    5   Wrong data type
    101 Internal communication error
    102 No access — storage process in progress

------------------------------------------------------------------------------
CONTROL WORD STW  (Sec. 4.3, p. 26)
------------------------------------------------------------------------------
    Bit 0   Start / Stop
    Bit 5   24 VDC output X201
    Bit 6   Enable main setpoint on PZD2  (speed setpoint)
    Bit 7   Reset error — only valid when Bit 0 = 0 (pump stopped)
    Bit 8   Enable standby function
    Bit 10  Enable process data  ** MUST be 1 for bits 0,5,6,7,8,13,14,15
                                    to have any effect **
    Bit 11  Error operation relay X1
    Bit 12  Normal operation relay X1
    Bit 13  Warning relay X1
    Bit 14  24 VDC output X202  (TURBOVAC iX only)
    Bit 15  24 VDC output X203  (TURBOVAC iX only)

------------------------------------------------------------------------------
STATUS WORD ZSW  (Sec. 4.4, p. 26)
------------------------------------------------------------------------------
    Bit 0   Ready for operation
    Bit 2   Operation enabled  (pump is spinning at target speed)
    Bit 3   Error condition
    Bit 4   Accelerating
    Bit 5   Decelerating
    Bit 6   Switch-on lock
    Bit 7   Temperature warning
    Bit 9   Parameter channel enabled
    Bit 10  Normal operation detained
    Bit 11  Pump is turning  (rotor moving, not necessarily at speed)
    Bit 13  Overload warning
    Bit 14  Collective warning
    Bit 15  Process channel enabled

------------------------------------------------------------------------------
KEY PARAMETER NUMBERS (Sec. 5 Parameter List)
------------------------------------------------------------------------------
    P3   Rotor frequency, actual  [Hz]             (r)
    P4   DC intermediate voltage  [V]              (r)
    P5   Motor current x 0.1      [A]              (r)
    P8   Save parameters to EEPROM (write 1)       (w) ~30 s, no power cut
    P11  Freq-converter temperature [deg C]        (r)
    P37  RS-485 device address (0-31)              (r/w)
    P125 Bearing temperature  [deg C]              (r)
    P227 Active warning code                       (r)
    P302 Total operating hours                     (r)  32-bit
    P303 Most recent error code                    (r)  32-bit
"""

from __future__ import annotations

import csv
import logging
import os
import struct
import time
from dataclasses import dataclass
from typing import Callable, Optional

import serial  # pip install pyserial

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# USS protocol constants  (Sec. 3.1 and Sec. 4.1)
# ---------------------------------------------------------------------------

_STX = 0x02
_LGE = 0x16          # payload length field value

_BYTESIZE = serial.EIGHTBITS
_PARITY   = serial.PARITY_EVEN
_STOPBITS = serial.STOPBITS_ONE

_RESPONSE_DELAY_S   = 0.05   # 50 ms — above the 10 ms firmware minimum
_RESPONSE_TIMEOUT_S = 0.5

# ---------------------------------------------------------------------------
# AK task designators  (Sec. 4.1, p. 24)
# NOTE: 0x1 is the ONLY read designator — the pump replies with 0x1 (16-bit)
#       or 0x2 (32-bit) depending on the parameter type. Using 0x2 as a
#       "read dword" (as Profidrive generically does) is WRONG for this pump
#       because 0x2 means "write 16-bit" on the Leybold USS implementation.
# ---------------------------------------------------------------------------

_AK_NONE        = 0x0   # no parameter task, poll PZD only
_AK_READ        = 0x1   # request parameter value (any width)
_AK_WRITE_WORD  = 0x2   # write 16-bit parameter value
_AK_WRITE_DWORD = 0x3   # write 32-bit parameter value
_AK_ERROR       = 0x7   # reply-only: pump cannot execute task
_AK_NO_WRITE    = 0x8   # reply-only: no write permission

# ---------------------------------------------------------------------------
# Control word presets  (Sec. 4.3, p. 26)
#
# Bit arithmetic:
#   bit 0  = 0x0001   (Start/Stop)
#   bit 6  = 0x0040   (Enable setpoint PZD2)
#   bit 7  = 0x0080   (Reset error)
#   bit 10 = 0x0400   (Enable process data — mandatory gating bit)
# ---------------------------------------------------------------------------

_CW_ENABLE  = 0x0400   # bit 10 only: open process data channel
_CW_START   = 0x0441   # bit 0 + bit 6 + bit 10
_CW_STOP    = 0x0440   # bit 6 + bit 10, bit 0 cleared
_CW_RST_ERR = 0x0480   # bit 7 + bit 10 (bit 0 must be 0 for reset to work)


# ---------------------------------------------------------------------------
# Public data class
# ---------------------------------------------------------------------------

@dataclass
class PumpTelemetry:
    """
    Live process data snapshot returned with every USS telegram exchange.

    Field sources are the PZD process data words in the response telegram
    (Sec. 3.1, p. 22 of document 300450826_002_C2).

    Attributes
    ----------
    rotor_speed_hz    Actual rotor frequency in Hz          (PZD2 = P3)
    current_a         Motor current in Amperes              (PZD4 = P5 x 0.1)
    temperature_c     Freq-converter temperature in deg C   (PZD3 = P11)
    bearing_temp_c    Bearing temperature in deg C          (PZD5 = P125)
    dc_voltage_v      DC link voltage in Volts              (PZD6 = P4)
    status_word       Raw ZSW integer (PZD1) — use flag properties to decode
    """

    rotor_speed_hz: float
    current_a:      float
    temperature_c:  float
    bearing_temp_c: float
    dc_voltage_v:   float
    status_word:    int

    # ---- Status word flag properties  (Sec. 4.4, p. 26) ------------------

    @property
    def is_ready(self) -> bool:
        """Ready for operation (ZSW bit 0)."""
        return bool(self.status_word & (1 << 0))

    @property
    def is_running(self) -> bool:
        """Operation enabled — at target speed (ZSW bit 2)."""
        return bool(self.status_word & (1 << 2))

    @property
    def has_fault(self) -> bool:
        """Error condition present (ZSW bit 3)."""
        return bool(self.status_word & (1 << 3))

    @property
    def is_accelerating(self) -> bool:
        """Pump is accelerating (ZSW bit 4)."""
        return bool(self.status_word & (1 << 4))

    @property
    def is_decelerating(self) -> bool:
        """Pump is decelerating (ZSW bit 5)."""
        return bool(self.status_word & (1 << 5))

    @property
    def has_temperature_warning(self) -> bool:
        """Temperature warning active (ZSW bit 7)."""
        return bool(self.status_word & (1 << 7))

    @property
    def param_channel_enabled(self) -> bool:
        """Parameter channel is active (ZSW bit 9)."""
        return bool(self.status_word & (1 << 9))

    @property
    def is_turning(self) -> bool:
        """Rotor is physically moving, not necessarily at speed (ZSW bit 11)."""
        return bool(self.status_word & (1 << 11))

    @property
    def has_overload_warning(self) -> bool:
        """Overload warning active (ZSW bit 13)."""
        return bool(self.status_word & (1 << 13))

    @property
    def has_collective_warning(self) -> bool:
        """Collective warning active (ZSW bit 14)."""
        return bool(self.status_word & (1 << 14))

    @property
    def process_channel_enabled(self) -> bool:
        """Process data channel is active (ZSW bit 15)."""
        return bool(self.status_word & (1 << 15))

    def __str__(self) -> str:
        flags = []
        if self.is_running:              flags.append("RUNNING")
        if self.is_ready:                flags.append("READY")
        if self.is_turning:              flags.append("TURNING")
        if self.is_accelerating:         flags.append("ACCEL")
        if self.is_decelerating:         flags.append("DECEL")
        if self.has_fault:               flags.append("FAULT")
        if self.has_temperature_warning: flags.append("TEMP_WARN")
        if self.has_overload_warning:    flags.append("OVERLOAD")
        if self.has_collective_warning:  flags.append("WARNING")
        return (
            f"[{', '.join(flags) or 'IDLE'}]  "
            f"speed={self.rotor_speed_hz:.0f} Hz  "
            f"I={self.current_a:.2f} A  "
            f"T_conv={self.temperature_c:.0f} C  "
            f"T_bear={self.bearing_temp_c:.0f} C  "
            f"Vdc={self.dc_voltage_v:.1f} V  "
            f"ZSW=0x{self.status_word:04X}"
        )


# ---------------------------------------------------------------------------
# Internal USS frame helpers
# ---------------------------------------------------------------------------

def _xor_checksum(data: bytes) -> int:
    """BCC checksum: XOR of all bytes. (Sec. 3.1, p. 22)"""
    result = 0
    for b in data:
        result ^= b
    return result


def _build_telegram(
    address: int,
    ak:      int,
    pnu:     int,
    ind:     int,
    pwe:     int,
    stw:     int,
    hsw:     int = 0,
) -> bytes:
    """
    Assemble a 24-byte USS telegram ready to write to the serial port.
    Layout per Sec. 3.1, p. 22 of document 300450826_002_C2.
    """
    pke = ((ak & 0xF) << 12) | (pnu & 0x0FFF)
    payload = struct.pack(
        ">B B B H B B I H H H H H H",
        _STX,
        _LGE,
        address & 0x1F,
        pke,
        0x00,             # reserved byte
        ind & 0xFF,
        pwe & 0xFFFF_FFFF,
        stw  & 0xFFFF,    # PZD1 control word
        hsw  & 0xFFFF,    # PZD2 speed setpoint
        0x0000,           # PZD3 (read-only, zero in requests)
        0x0000,           # PZD4 (read-only, zero in requests)
        0x0000,           # PZD5 (read-only, zero in requests)
        0x0000,           # PZD6 (read-only, zero in requests)
    )
    return payload + bytes([_xor_checksum(payload)])


def _parse_response(data: bytes) -> tuple[int, int, int, int, int, PumpTelemetry]:
    """
    Validate and decode a 24-byte USS response telegram.
    Layout per Sec. 3.1, p. 22 of document 300450826_002_C2.

    PZD field scalings (Sec. 3.1 table):
        PZD3 — temperature in deg C  (integer, no scaling)
        PZD4 — current in 0.1 A      (divide by 10 for Amperes)
        PZD5 — bearing temp in deg C (integer, no scaling)
        PZD6 — voltage in V           (integer, no scaling — NOT 0.1 V)

    Returns
    -------
    (address, ak, pnu, ind, pwe, telemetry)
    """
    if len(data) != 24:
        raise ValueError(f"USS: expected 24 bytes, received {len(data)}")

    bcc_expected = _xor_checksum(data[:23])
    if data[23] != bcc_expected:
        raise ValueError(
            f"USS BCC mismatch: received 0x{data[23]:02X}, "
            f"expected 0x{bcc_expected:02X}"
        )

    _, _, adr, pke, _, ind, pwe, pzd1, pzd2, pzd3, pzd4, pzd5, pzd6 = \
        struct.unpack(">B B B H B B I H H H H H H", data[:23])

    telemetry = PumpTelemetry(
        rotor_speed_hz = float(pzd2),        # Hz,   Sec. 3.1
        current_a      = pzd4 * 0.1,         # x0.1A, Sec. 3.1
        temperature_c  = float(pzd3),        # deg C, Sec. 3.1
        bearing_temp_c = float(pzd5),        # deg C, Sec. 3.1
        dc_voltage_v   = float(pzd6),        # V (not 0.1 V), Sec. 3.1
        status_word    = pzd1,
    )

    return adr, (pke >> 12) & 0xF, pke & 0x0FFF, ind, pwe, telemetry


# ---------------------------------------------------------------------------
# Device wrapper class
# ---------------------------------------------------------------------------

class TurboVac250i:
    """
    High-level wrapper for the Leybold TurboVac 250i (TURBOVAC i/iX series)
    over RS-485 using the USS protocol (VDI/VDE 3689).

    Protocol source: Leybold document 300450826_002_C2 (September 2020).

    Parameters
    ----------
    port        : Serial port, e.g. ``"COM6"`` or ``"/dev/ttyUSB0"``.
    address     : RS-485 device address (0-31). Factory default is ``0``.
    baud_rate   : Serial baud rate. Fixed at 19200 by firmware.
    retries     : Telegram retry attempts before raising an exception.
    retry_delay : Seconds to wait between retries.
    """

    def __init__(
        self,
        port:        str,
        address:     int   = 0,
        baud_rate:   int   = 19200,
        retries:     int   = 3,
        retry_delay: float = 0.1,
    ):
        self.port        = port
        self.address     = address
        self.baud_rate   = baud_rate
        self.retries     = retries
        self.retry_delay = retry_delay
        self._ser: Optional[serial.Serial] = None
        self._cw  = _CW_ENABLE   # shadow register for current control word
        self._hsw = 0             # shadow register for speed setpoint (HSW, PZD2)

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> "TurboVac250i":
        """Open the RS-485 port. Returns ``self`` for chaining."""
        self._ser = serial.Serial(
            port     = self.port,
            baudrate = self.baud_rate,
            bytesize = _BYTESIZE,
            parity   = _PARITY,
            stopbits = _STOPBITS,
            timeout  = _RESPONSE_TIMEOUT_S,
        )
        log.info(
            "TurboVac250i connected: port=%s address=%d baud=%d parity=Even",
            self.port, self.address, self.baud_rate,
        )
        return self

    def disconnect(self):
        """Close the RS-485 port."""
        if self._ser and self._ser.is_open:
            self._ser.close()
            log.info("TurboVac250i disconnected: port=%s", self.port)
        self._ser = None

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ------------------------------------------------------------------
    # Low-level transport (private)
    # ------------------------------------------------------------------

    def _exchange(
        self,
        ak:  int,
        pnu: int,
        ind: int = 0,
        pwe: int = 0,
        stw: Optional[int] = None,
        hsw: Optional[int] = None,
    ) -> tuple[int, PumpTelemetry]:
        """
        Send one USS telegram and return ``(pwe, telemetry)``.
        Retries up to ``self.retries`` times on any transport or checksum error.
        """
        if not self.is_connected:
            raise RuntimeError("Not connected — call connect() first.")

        if stw is None:
            stw = self._cw
        if hsw is None:
            hsw = self._hsw

        frame = _build_telegram(
            address=self.address, ak=ak, pnu=pnu, ind=ind, pwe=pwe, stw=stw, hsw=hsw
        )
        log.debug("TX [%d bytes]: %s", len(frame), frame.hex(" "))

        last_exc: Exception = RuntimeError("No exchange attempts made")

        for attempt in range(1, self.retries + 1):
            try:
                self._ser.reset_input_buffer()
                self._ser.write(frame)
                time.sleep(_RESPONSE_DELAY_S)

                raw = self._ser.read(24)
                log.debug("RX [%d bytes]: %s", len(raw), raw.hex(" ") if raw else "(empty)")

                if len(raw) != 24:
                    raise TimeoutError(
                        f"Expected 24 bytes, received {len(raw)}. "
                        "Check wiring, address, and termination resistors."
                    )

                _, resp_ak, resp_pnu, _, resp_pwe, telemetry = _parse_response(raw)

                if resp_ak == _AK_ERROR:
                    raise RuntimeError(
                        f"Pump refused task for PNU={pnu} (error code: {resp_pwe})"
                    )
                if resp_ak == _AK_NO_WRITE:
                    raise RuntimeError(
                        f"Pump denied write access for PNU={pnu}"
                    )

                return resp_pwe, telemetry

            except (serial.SerialException, TimeoutError, ValueError, RuntimeError) as exc:
                last_exc = exc
                log.warning("Attempt %d/%d failed: %s", attempt, self.retries, exc)
                if attempt < self.retries:
                    time.sleep(self.retry_delay)

        raise last_exc

    def _send_control_word(self, cw: int):
        """Push a new control word to the pump and update the shadow register."""
        self._cw = cw
        self._exchange(ak=_AK_NONE, pnu=0, stw=cw, hsw=self._hsw)
        log.debug("Control word -> 0x%04X  HSW -> %d Hz", cw, self._hsw)

    # ------------------------------------------------------------------
    # Parameter access
    # ------------------------------------------------------------------

    def read_param(self, pnu: int, ind: int = 0) -> int:
        """
        Read a pump parameter by number.

        Uses AK=0x1 for all reads per Sec. 4.1. The pump determines whether
        to reply with a 16-bit (AK=0x1) or 32-bit (AK=0x2) response based
        on the parameter type — no separate "read dword" designator exists.

        Parameters
        ----------
        pnu : Parameter number (see module docstring).
        ind : Index — ``0`` for scalar parameters.

        Returns
        -------
        Raw integer PWE value.
        """
        pwe, _ = self._exchange(ak=_AK_READ, pnu=pnu, ind=ind)
        log.debug("read_param(PNU=%d, ind=%d) -> %d", pnu, ind, pwe)
        return pwe

    def write_param(self, pnu: int, value: int, ind: int = 0, double_word: bool = False):
        """
        Write a pump parameter by number.

        Parameters
        ----------
        pnu         : Parameter number.
        value       : Integer value to write.
        ind         : Index — ``0`` for scalar parameters.
        double_word : ``True`` for 32-bit parameters (AK=0x3 per Sec. 4.1).
        """
        ak = _AK_WRITE_DWORD if double_word else _AK_WRITE_WORD
        self._exchange(ak=ak, pnu=pnu, ind=ind, pwe=value)
        log.debug("write_param(PNU=%d, ind=%d) <- %d OK", pnu, ind, value)

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def poll(self) -> PumpTelemetry:
        """
        Send a no-task telegram (AK=0x0) to read all live PZD process data
        without accessing any parameter. Safe to call at any time.

        Returns a PumpTelemetry snapshot with speed, current, temperatures,
        voltage, and decoded status flags.
        """
        _, telemetry = self._exchange(ak=_AK_NONE, pnu=0)
        return telemetry

    # ------------------------------------------------------------------
    # Control  (Sec. 4.3)
    # ------------------------------------------------------------------

    def start(self):
        """
        Start the pump via fieldbus.

        Per Sec. 4.3: bit 10 must be set before any control bits take effect.
        Sequence:
          1. Send _CW_ENABLE  (bit 10 only) — open the process data channel.
          2. Brief pause.
          3. Send _CW_START   (bit 0 + bit 6 + bit 10) — start + enable setpoint.

        Pump must be fault-free. Call reset_error() first if has_fault is True.
        """
        log.info("Opening process data channel (STW bit 10)...")
        self._send_control_word(_CW_ENABLE)
        time.sleep(0.1)
        log.info("Sending START (STW bits 0 + 6 + 10)...")
        self._send_control_word(_CW_START)
        log.info("Start command sent.")

    def stop(self):
        """
        Stop the pump.

        Clears bit 0 while keeping bits 6 and 10 set (Sec. 4.3).
        The pump decelerates under its own control to standstill.
        """
        log.info("Sending STOP (STW bit 0 cleared)...")
        self._send_control_word(_CW_STOP)
        log.info("Stop command sent.")

    def reset_error(self):
        """
        Reset the error latch (STW bit 7 + bit 10).

        Per Sec. 4.3: reset is only possible when bit 0 = 0 (pump stopped).
        Automatically returns control word to stopped state after reset pulse.
        """
        log.info("Sending error reset (STW bit 7 + bit 10)...")
        self._send_control_word(_CW_RST_ERR)
        time.sleep(0.1)
        self._send_control_word(_CW_STOP)
        log.info("Error reset sent.")

    # ------------------------------------------------------------------
    # Speed control  (Sec. 4.3 HSW field; P17/P18 in Sec. 5)
    # ------------------------------------------------------------------

    def set_speed(self, hz: int):
        """
        Set the target rotor speed in Hz.

        This writes the value to P18 (nominal speed setpoint) via the
        parameter channel, then updates the live HSW setpoint field in
        PZD2 so the running pump responds immediately if already started.

        The value is clamped to the range [0, P17_max] where P17 is the
        firmware maximum frequency. For the TurboVac 250i the factory
        maximum is typically 1500 Hz; the nominal default shipped is
        1000 Hz. Confirm with read_param(17) before raising above 1000.

        Parameters
        ----------
        hz : Target speed in Hz (integer). Use get_max_speed() to check
             the firmware ceiling before setting above 1000.
        """
        if hz < 0:
            raise ValueError(f"Speed must be >= 0 Hz, got {hz}")
        self.write_param(18, hz)
        # Also push the live setpoint into HSW (PZD2) so a running pump
        # responds without needing a stop/start cycle.
        self._hsw = hz & 0xFFFF
        log.info("Speed setpoint written to P18: %d Hz", hz)

    def get_speed_setpoint(self) -> int:
        """
        Read the current speed setpoint from P18 in Hz.

        This is the *target*, not the actual rotor speed.
        Use get_rotor_speed() for the live reading from PZD2.
        """
        return self.read_param(18)

    def get_max_speed(self) -> int:
        """
        Read the firmware maximum frequency limit from P17 in Hz.

        The pump will never exceed this value regardless of what is
        written to P18. Confirm this before commanding above 1000 Hz.
        """
        return self.read_param(17)

    # ------------------------------------------------------------------
    # Named telemetry convenience reads
    # ------------------------------------------------------------------

    def get_status(self) -> PumpTelemetry:
        """Full telemetry snapshot. Alias for poll()."""
        return self.poll()

    def get_rotor_speed(self) -> float:
        """Actual rotor speed in Hz (PZD2)."""
        return self.poll().rotor_speed_hz

    def get_motor_current(self) -> float:
        """Motor current in Amperes (PZD4 x 0.1)."""
        return self.poll().current_a

    def get_temperature(self) -> float:
        """Frequency-converter temperature in deg C (PZD3)."""
        return self.poll().temperature_c

    def get_bearing_temperature(self) -> float:
        """Bearing temperature in deg C (PZD5)."""
        return self.poll().bearing_temp_c

    def get_dc_voltage(self) -> float:
        """DC intermediate circuit voltage in Volts (PZD6)."""
        return self.poll().dc_voltage_v

    def get_error_code(self) -> int:
        """Most recent error code (P303)."""
        return self.read_param(303)

    def get_warning_code(self) -> int:
        """Active warning code (P227)."""
        return self.read_param(227)

    def get_operating_hours(self) -> int:
        """Total operating hours (P302)."""
        return self.read_param(302)

    def get_rs485_address(self) -> int:
        """Configured RS-485 address (P37)."""
        return self.read_param(37)

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def set_rs485_address(self, new_address: int):
        """
        Write a new RS-485 address to P37 (range 0-31).

        Must be followed by save_parameters() and a power-cycle to take effect.
        Pump must be at standstill before saving.
        """
        if not 0 <= new_address <= 31:
            raise ValueError(f"RS-485 address must be 0-31, got {new_address}")
        self.write_param(37, new_address)
        log.info("RS-485 address written as %d. Save and power-cycle to apply.", new_address)

    def save_parameters(self):
        """
        Permanently save all parameters to EEPROM (writes 1 to P8).

        Takes ~30 seconds. Do NOT cut power during this time.
        Pump must be at standstill. Power-cycle afterward for changes to apply.
        """
        log.info("Saving parameters to EEPROM (~30 s) — do NOT cut power...")
        self.write_param(8, 1)
        log.info("Save command sent. Wait 30 s before cycling power.")

    # ------------------------------------------------------------------
    # Live monitoring
    # ------------------------------------------------------------------

    def monitor(
        self,
        interval: float = 1.0,
        count:    int   = -1,
        on_data:  Optional[Callable[[PumpTelemetry], None]] = None,
    ):
        """
        Stream live telemetry to stdout at a fixed polling interval.

        Parameters
        ----------
        interval : Seconds between polls (default 1.0).
        count    : Samples to collect; ``-1`` runs until KeyboardInterrupt.
        on_data  : Optional callback f(PumpTelemetry) called on every sample.
        """
        header = (
            f"{'#':>6}  {'Speed(Hz)':>10}  {'I(A)':>8}  "
            f"{'T_conv(C)':>10}  {'T_bear(C)':>10}  {'Vdc(V)':>8}  {'Flags':<30}"
        )
        print(header)
        print("-" * len(header))

        i = 0
        try:
            while count < 0 or i < count:
                t = self.poll()
                flags = []
                if t.is_running:              flags.append("RUNNING")
                if t.is_ready:                flags.append("READY")
                if t.is_turning:              flags.append("TURNING")
                if t.is_accelerating:         flags.append("ACCEL")
                if t.is_decelerating:         flags.append("DECEL")
                if t.has_fault:               flags.append("FAULT!")
                if t.has_temperature_warning: flags.append("TEMP_WARN")
                if t.has_overload_warning:    flags.append("OVERLOAD")
                print(
                    f"{i:>6}  "
                    f"{t.rotor_speed_hz:>10.1f}  "
                    f"{t.current_a:>8.2f}  "
                    f"{t.temperature_c:>10.1f}  "
                    f"{t.bearing_temp_c:>10.1f}  "
                    f"{t.dc_voltage_v:>8.1f}  "
                    f"{', '.join(flags) or 'IDLE':<30}"
                )
                if on_data is not None:
                    on_data(t)
                i += 1
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nMonitoring stopped.")


# ---------------------------------------------------------------------------
# Helpers imported by main.py
# ---------------------------------------------------------------------------

def _safe_read(fn: Callable, label: str) -> str:
    """
    Call fn() and return its string result.
    If the pump refuses the parameter, return a friendly fallback instead of
    crashing — some parameters may not be accessible on the basic RS-485
    interface. Error codes per Sec. 4.1, p. 25.
    """
    try:
        return str(fn())
    except RuntimeError as e:
        log.debug("Parameter read skipped (%s): %s", label, e)
        return "n/a"


def print_status(pump: TurboVac250i):
    """
    Print a formatted one-shot status summary for the given pump.

    Note: P302 (operating hours) and P303 (error code) are not accessible
    on the basic RS-485 interface of the TURBOVAC-i — the pump returns
    error code 0 (impermissible parameter number) for both. They are
    omitted here to avoid noisy warnings. They may only be readable via
    the Anybus or Profibus interface.
    """
    t = pump.get_status()
    print(f"\n=== TurboVac 250i  port={pump.port}  address={pump.address} ===")
    print(f"  Telemetry      : {t}")
    print(f"  Warning code   : {_safe_read(pump.get_warning_code, 'P227')}")
    print(f"  RS-485 addr    : {_safe_read(pump.get_rs485_address, 'P37')}")
    print()


def make_csv_callback(path: str) -> Callable[[PumpTelemetry], None]:
    """
    Return a callback that appends one row per PumpTelemetry sample to a
    CSV file. Header is written automatically if the file is new.

    Columns: timestamp, speed_hz, current_a, temp_conv_c, temp_bearing_c,
             dc_voltage_v, status_word

    Pass the returned callable as the on_data argument to monitor().
    """
    write_header = not os.path.exists(path)

    def _callback(t: PumpTelemetry):
        nonlocal write_header
        with open(path, "a", newline="") as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow([
                    "timestamp", "speed_hz", "current_a",
                    "temp_conv_c", "temp_bearing_c", "dc_voltage_v", "status_word",
                ])
                write_header = False
            writer.writerow([
                time.strftime("%Y-%m-%dT%H:%M:%S"),
                t.rotor_speed_hz,
                t.current_a,
                t.temperature_c,
                t.bearing_temp_c,
                t.dc_voltage_v,
                f"0x{t.status_word:04X}",
            ])

    return _callback