#!/usr/bin/env python3
# encoding: utf-8
# stm32 python sdk

"""
Minimal STM32 SDK for:
    - commanding wheel motors;
    - receiving raw IMU measurements.
"""

import enum
import queue
import struct
import threading
import time
from typing import Optional
import serial

class PacketControllerState(enum.IntEnum):
    """States used to decode incoming STM32 serial packets."""

    START_BYTE_1 = 0
    START_BYTE_2 = 1
    FUNCTION = 2
    LENGTH = 3
    DATA = 4
    CHECKSUM = 5

class PacketFunction(enum.IntEnum):
    """Identifiers of the STM32 protocol functions."""
    # 可通过串口实现的控制功能(achieve control function via the serial port)
    PACKET_FUNC_MOTOR = 3  # 电机控制(motor control)
    PACKET_FUNC_IMU = 7
    PACKET_FUNC_NONE = 12

CRC8_TABLE = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
]

def checksum_crc8(data: bytes) -> int:
    """Compute the CRC-8 checksum used by the STM32 protocol."""
    checksum = 0

    for byte in data:
        checksum = CRC8_TABLE[checksum ^ byte]

    return checksum & 0xFF

class Board:
    """Serial interface for motor commands and IMU reception."""

    def __init__(self, device: str = '/dev/ttyACM0', baudrate: int = 1_000_000, timeout: float = 10.0) -> None:
        self.enable_recv = False
        self.running = True

        # State of the incoming-packet decoder.
        self.state = PacketControllerState.START_BYTE_1
        self.frame: list[int] = []
        self.recv_count = 0

        # Only keep one pending IMU sample.
        self.imu_queue: queue.Queue[bytes] = queue.Queue(maxsize=1)

        # Associate each received function ID with its decoder.
        self.parsers = {
            PacketFunction.PACKET_FUNC_IMU: self.packet_report_imu,
        }

        # Open the serial communication with the STM32 board.
        self.port = serial.Serial(port=None, baudrate=baudrate, timeout=timeout)
        self.port.rts = False
        self.port.dtr = False
        self.port.port = device
        self.port.open()

        time.sleep(0.5)

        # Continuously receive packets from the STM32.
        self.recv_thread = threading.Thread(
            target=self.recv_task,
            daemon=True,
        )
        self.recv_thread.start()

    def enable_reception(self, enable: bool = True) -> None:
        """Enable or disable STM32 packet reception."""
        self.enable_recv = enable

    def buf_write(self, function: PacketFunction, data: bytes | bytearray | list[int]) -> None:
        """Build and send one STM32 protocol packet."""
        packet = [0xAA, 0x55, int(function), len(data)]

        packet.extend(data)

        checksum = checksum_crc8(bytes(packet[2:]))
        packet.append(checksum)

        self.port.write(bytes(packet))

    def set_motor_speed(self, speeds: list[list[float]]) -> None:
        """
        Send wheel-speed commands.

        Each entry is:
            [motor_id, speed_rps]
        """
        data = bytearray([
            0x01,
            len(speeds),
        ])

        for motor_id, speed_rps in speeds:
            if motor_id not in (1, 2, 3, 4):
                raise ValueError(
                    f'Invalid motor ID: {motor_id}. Expected 1 to 4.'
                )
            
            data.extend(
                struct.pack(
                    '<Bf',
                    motor_id - 1,
                    float(speed_rps),
                )
            )

            self.buf_write(
                PacketFunction.PACKET_FUNC_MOTOR,
                data,
            )

    def stop_motors(self) -> None:
        """Stop the four wheel motors."""
        self.set_motor_speed([
            [1, 0.0],
            [2, 0.0],
            [3, 0.0],
            [4, 0.0],            
        ])
    
    def packet_report_imu(self, data: bytes) -> None:
        """Store one received raw IMU packet."""

        try:
            self.imu_queue.put_nowait(data)

        except queue.Full:
            # Replace the previous unread sample with the newest one.
            try:
                self.imu_queue.get_nowait()
            except queue.Empty:
                pass

            try:
                self.imu_queue.put_nowait(data)
            except queue.Full:
                pass
    
    def get_imu(self) -> Optional[tuple[float, float, float, float, float, float]]:
        """
        Return the newest IMU measurement.

        Returns:
            ax, ay, az, gx, gy, gz

        The six values are encoded by the STM32 as little-endian float32.
        """

        if not self.enable_recv:
            return None

        try:
            data = self.imu_queue.get_nowait()

        except queue.Empty:
            return None

        expected_size = struct.calcsize('<6f')

        if len(data) != expected_size:
            return None

        return struct.unpack('<6f', data)

    def recv_task(self) -> None:
        """Read the serial port and decode incoming STM32 packets."""

        while self.running:
            if not self.enable_recv:
                time.sleep(0.01)
                continue

            try:
                received_data = self.port.read(1)

            except serial.SerialException:
                break

            if not received_data:
                continue

            byte = received_data[0]

            if self.state == PacketControllerState.START_BYTE_1:
                if byte == 0xAA:
                    self.state = PacketControllerState.START_BYTE_2

            elif self.state == PacketControllerState.START_BYTE_2:
                if byte == 0x55:
                    self.state = PacketControllerState.FUNCTION
                else:
                    self.reset_parser()

            elif self.state == PacketControllerState.FUNCTION:
                if byte < int(PacketFunction.PACKET_FUNC_NONE):
                    self.frame = [byte, 0]
                    self.state = PacketControllerState.LENGTH
                else:
                    self.reset_parser()

            elif self.state == PacketControllerState.LENGTH:
                self.frame[1] = byte
                self.recv_count = 0

                if byte == 0:
                    self.state = PacketControllerState.CHECKSUM
                else:
                    self.state = PacketControllerState.DATA

            elif self.state == PacketControllerState.DATA:
                self.frame.append(byte)
                self.recv_count += 1

                if self.recv_count >= self.frame[1]:
                    self.state = PacketControllerState.CHECKSUM

            elif self.state == PacketControllerState.CHECKSUM:
                expected_crc = checksum_crc8(bytes(self.frame))

                if expected_crc == byte:
                    try:
                        function = PacketFunction(self.frame[0])
                    except ValueError:
                        self.reset_parser()
                        continue

                    payload = bytes(self.frame[2:])
                    parser = self.parsers.get(function)

                    if parser is not None:
                        parser(payload)

                self.reset_parser()

    def reset_parser(self) -> None:
        """Reset the serial packet decoder."""

        self.state = PacketControllerState.START_BYTE_1
        self.frame = []
        self.recv_count = 0

    def close(self) -> None:
        """Stop reception, stop motors and close the serial port."""

        self.running = False
        self.enable_recv = False

        try:
            self.stop_motors()
        except serial.SerialException:
            pass

        if self.port.is_open:
            self.port.close()

# buttons_map = {
#             'GAMEPAD_BUTTON_MASK_L2':        0x0001,
#             'GAMEPAD_BUTTON_MASK_R2':        0x0002,
#             'GAMEPAD_BUTTON_MASK_SELECT':    0x0004,
#             'GAMEPAD_BUTTON_MASK_START':     0x0008,
#             'GAMEPAD_BUTTON_MASK_L3':        0x0020,
#             'GAMEPAD_BUTTON_MASK_R3':        0x0040,
#             'GAMEPAD_BUTTON_MASK_CROSS':     0x0100,
#             'GAMEPAD_BUTTON_MASK_CIRCLE':    0x0200,
#             'GAMEPAD_BUTTON_MASK_SQUARE':    0x0800,
#             'GAMEPAD_BUTTON_MASK_TRIANGLE':  0x1000,
#             'GAMEPAD_BUTTON_MASK_L1':        0x4000,
#             'GAMEPAD_BUTTON_MASK_R1':        0x8000
#     }

#     def __init__(self, device="/dev/rrc", baudrate=1000000, timeout=10):
#         self.enable_recv = False
#         self.frame = []
#         self.recv_count = 0

#         self.port = serial.Serial(None, baudrate, timeout=timeout)
#         self.port.rts = False
#         self.port.dtr = False
#         self.port.setPort(device)
#         self.port.open()

#         self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARBYTE1
#         self.servo_read_lock = threading.Lock()
#         self.pwm_servo_read_lock = threading.Lock()

#         # 队列用来存储数据(use queue to store data)
#         self.sys_queue = queue.Queue(max_size=1)
#         self.bus_servo_queue = queue.Queue(max_size=1)
#         self.pwm_servo_queue = queue.Queue(max_size=1)
#         self.key_queue = queue.Queue(max_size=1)
#         self.imu_queue = queue.Queue(max_size=1)
#         self.gamepad_queue = queue.Queue(maxsize=1)
#         self.sbus_queue = queue.Queue(max_size=1)

#         self.parsers = {
#             PacketFunction.PACKET_FUNC_SYS: self.packet_report_sys,
#             PacketFunction.PACKET_FUNC_KEY: self.packet_report_key,
#             PacketFunction.PACKET_FUNC_IMU: self.packet_report_imu,
#             PacketFunction.PACKET_FUNC_GAMEPAD: self.packet_report_gamepad,
#             PacketFunction.PACKET_FUNC_BUS_SERVO: self.packet_report_serial_servo,
#             PacketFunction.PACKET_FUNC_SBUS: self.packet_report_sbus,
#             PacketFunction.PACKET_FUNC_PWM_SERVO: self.packet_report_pwm_servo
#         }

#         time.sleep(0.5)
#         threading.Thread(target=self.recv_task, daemon=True).start()

#     def enable_reception(self, enable=True):
#         self.enable_recv = enable

