"""Minimal Feetech serial bus driver built on scservo_sdk (pip: feetech-servo-sdk).

Modeled after lerobot's FeetechMotorsBus but standalone and register-level,
so cookbook scripts can show exactly what goes over the wire.
"""

from __future__ import annotations

import fcntl
import os
import time
from contextlib import contextmanager
from pathlib import Path

import scservo_sdk as scs

from .registers import (
    BAUDRATE_TABLE,
    MODEL_NUMBER_TABLE,
    STS_SMS_CONTROL_TABLE,
    STS_SMS_SIGN_BITS,
    decode_sign_magnitude,
    encode_sign_magnitude,
)

DEFAULT_BAUDRATE = 1_000_000
DEFAULT_TIMEOUT_MS = 1000
PROTOCOL_STS_SMS = 0


def _patched_set_packet_timeout(self, packet_length):
    # Fixes wrong timeout computation in the PyPI scservo_sdk
    # (https://gitee.com/ftservo/SCServoSDK/issues/IBY2S6), same patch as lerobot.
    self.packet_start_time = self.getCurrentTime()
    self.packet_timeout = (self.tx_time_per_byte * packet_length) + (self.tx_time_per_byte * 3.0) + 50


class FeetechBus:
    """Half-duplex serial bus for Feetech STS/SMS servos (protocol 0).

    Register names come from ``sopo.registers.STS_SMS_CONTROL_TABLE``.
    All values are raw register units (positions in ticks, 0-4095 for STS).
    """

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE):
        self.port_name = port
        self.baudrate = baudrate
        self.port = scs.PortHandler(port)
        self.port.setPacketTimeout = _patched_set_packet_timeout.__get__(self.port)
        self.packet = scs.PacketHandler(PROTOCOL_STS_SMS)
        self.motor_errors: dict[int, int] = {}

    # --- connection -------------------------------------------------------

    def connect(self) -> None:
        # One process per bus: a lock file makes sopod and the cookbook scripts mutually exclusive.
        self._lock_path = Path(f"/tmp/sopo-{Path(self.port_name).name}.lock")
        self._lock_fd = os.open(self._lock_path, os.O_CREAT | os.O_RDWR, 0o666)
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            owner = ""
            try:
                owner = os.read(self._lock_fd, 64).decode().strip()
            except Exception:
                pass
            os.close(self._lock_fd)
            raise ConnectionError(f"{self.port_name} is in use by another sopo process ({owner or 'unknown pid'}) - stop it first (sopod running?)")
        os.ftruncate(self._lock_fd, 0)
        os.write(self._lock_fd, str(os.getpid()).encode())
        if not self.port.openPort():
            raise ConnectionError(f"Failed to open port {self.port_name}")
        self.port.setBaudRate(self.baudrate)
        self.port.setPacketTimeoutMillis(DEFAULT_TIMEOUT_MS)

    def disconnect(self, disable_torque_ids: list[int] | None = None) -> None:
        if disable_torque_ids:
            still_on = self.torque_off_verified(disable_torque_ids)
            if still_on:
                import sys
                print(f"!!! torque-off NOT VERIFIED for motors {still_on} (broadcast off was sent) - check with: python cookbook/1_setup/150_torque_off.py", file=sys.stderr)
        self.port.closePort()
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
            os.close(self._lock_fd)
        except Exception:
            pass

    def torque_off_verified(self, motor_ids: list[int], attempts: int = 3) -> list[int]:
        """Disable torque robustly. Returns ids whose torque-off could not be verified.

        1. Broadcast Torque_Enable=0 with sync_write first: it needs no reply, so it reaches the
           motors even when the receive path is broken (e.g. Ctrl+C mid-transaction left junk in
           the input buffer).
        2. Then per motor: clear the port buffer, write, read back, retry.
        """
        try:
            self.sync_write("Torque_Enable", {mid: 0 for mid in motor_ids})
        except Exception:
            pass
        still_on = []
        for motor_id in motor_ids:
            ok = False
            for _ in range(attempts):
                try:
                    self.port.clearPort()
                    self.write("Torque_Enable", motor_id, 0)
                    self.write("Lock", motor_id, 0)
                    if self.read("Torque_Enable", motor_id) == 0:
                        ok = True
                        break
                except Exception:
                    time.sleep(0.02)
            if not ok:
                still_on.append(motor_id)
        return still_on

    # --- discovery --------------------------------------------------------

    def ping(self, motor_id: int) -> str | None:
        """Returns the model name (or 'unknown_<num>') if the motor responds, else None."""
        model_number, comm, error = self.packet.ping(self.port, motor_id)
        if comm != scs.COMM_SUCCESS:
            return None
        return MODEL_NUMBER_TABLE.get(model_number, f"unknown_{model_number}")

    def scan(self, id_range: range = range(0, 254)) -> dict[int, str]:
        """Pings every ID in range at the current baudrate."""
        found = {}
        for motor_id in id_range:
            model = self.ping(motor_id)
            if model is not None:
                found[motor_id] = model
        return found

    # --- register access --------------------------------------------------

    def _addr(self, reg: str) -> tuple[int, int]:
        try:
            return STS_SMS_CONTROL_TABLE[reg]
        except KeyError:
            raise KeyError(f"Unknown register {reg!r}. See sopo/registers.py") from None

    def read(self, reg: str, motor_id: int) -> int:
        addr, size = self._addr(reg)
        if size == 1:
            value, comm, error = self.packet.read1ByteTxRx(self.port, motor_id, addr)
        else:
            value, comm, error = self.packet.read2ByteTxRx(self.port, motor_id, addr)
        self._check(comm, error, f"read {reg} from id={motor_id}", motor_id)
        sign_bit = STS_SMS_SIGN_BITS.get(reg)
        return decode_sign_magnitude(value, sign_bit) if sign_bit else value

    def write(self, reg: str, motor_id: int, value: int) -> None:
        addr, size = self._addr(reg)
        sign_bit = STS_SMS_SIGN_BITS.get(reg)
        if sign_bit:
            value = encode_sign_magnitude(value, sign_bit)
        if size == 1:
            comm, error = self.packet.write1ByteTxRx(self.port, motor_id, addr, value)
        else:
            comm, error = self.packet.write2ByteTxRx(self.port, motor_id, addr, value)
        self._check(comm, error, f"write {reg}={value} to id={motor_id}", motor_id)

    def _check(self, comm: int, error: int, what: str, motor_id: int | None = None) -> None:
        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(f"Comm error on {what}: {self.packet.getTxRxResult(comm)}")
        if error != 0 and motor_id is not None:
            # Motor health flags (voltage / sensor / temperature / current / overload). The read or
            # write itself succeeded; record the flag for the caller instead of aborting the loop.
            self.motor_errors[motor_id] = error

    def pop_motor_errors(self) -> dict[int, str]:
        """Status-byte error flags seen since the last call, as text. Cleared on return."""
        out = {mid: self.packet.getRxPacketError(err).strip() for mid, err in self.motor_errors.items()}
        self.motor_errors.clear()
        return out

    # --- bulk access ------------------------------------------------------

    def sync_read(self, reg: str, motor_ids: list[int]) -> dict[int, int]:
        """One request, all motors answer in turn. STS/SMS protocol 0 only."""
        addr, size = self._addr(reg)
        reader = scs.GroupSyncRead(self.port, self.packet, addr, size)
        for motor_id in motor_ids:
            reader.addParam(motor_id)
        comm = reader.txRxPacket()
        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(f"Sync read {reg} failed: {self.packet.getTxRxResult(comm)}")
        sign_bit = STS_SMS_SIGN_BITS.get(reg)
        values = {}
        for motor_id in motor_ids:
            if not reader.isAvailable(motor_id, addr, size):
                raise ConnectionError(f"Sync read {reg}: no data from id={motor_id}")
            value = reader.getData(motor_id, addr, size)
            values[motor_id] = decode_sign_magnitude(value, sign_bit) if sign_bit else value
        return values

    def sync_write(self, reg: str, values: dict[int, int]) -> None:
        """One broadcast packet carrying a value per motor. No per-motor ACK."""
        addr, size = self._addr(reg)
        sign_bit = STS_SMS_SIGN_BITS.get(reg)
        writer = scs.GroupSyncWrite(self.port, self.packet, addr, size)
        for motor_id, value in values.items():
            if sign_bit:
                value = encode_sign_magnitude(value, sign_bit)
            data = [value & 0xFF] if size == 1 else [value & 0xFF, (value >> 8) & 0xFF]
            writer.addParam(motor_id, data)
        comm = writer.txPacket()
        writer.clearParam()
        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(f"Sync write {reg} failed: {self.packet.getTxRxResult(comm)}")

    # --- torque -----------------------------------------------------------

    def enable_torque(self, motor_ids: list[int]) -> None:
        for motor_id in motor_ids:
            self.write("Torque_Enable", motor_id, 1)
            self.write("Lock", motor_id, 1)

    def disable_torque(self, motor_ids: list[int]) -> None:
        for motor_id in motor_ids:
            self.write("Torque_Enable", motor_id, 0)
            self.write("Lock", motor_id, 0)

    @contextmanager
    def torque_disabled(self, motor_ids: list[int]):
        self.disable_torque(motor_ids)
        try:
            yield
        finally:
            self.enable_torque(motor_ids)

    @contextmanager
    def eprom_unlocked(self, motor_id: int):
        """EPROM registers (addr < 40) only accept writes while Lock=0."""
        self.write("Lock", motor_id, 0)
        try:
            yield
        finally:
            self.write("Lock", motor_id, 1)
            time.sleep(0.01)
