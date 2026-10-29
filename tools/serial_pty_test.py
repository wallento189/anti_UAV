#!/usr/bin/env python3
"""Pseudo-terminal loopback test for autoaim_serial.

Creates a pty pair, prints the slave device path, then acts as the lower board:
it keeps writing valid feedback frames and decodes the command frames the ROS
node writes. No hardware or pyserial is required.

Typical use (workspace sourced):

    python3 tools/serial_pty_test.py
    # in another terminal, using the printed device path:
    ros2 run autoaim_serial serial_node --ros-args \
      --params-file $(ros2 pkg prefix autoaim_serial)/share/autoaim_serial/config/serial.yaml \
      -p device:=/dev/pts/N
    # and a fake target source:
    ros2 topic pub -r 50 autoaim_tracker/target \
      autoaim_interfaces/msg/TrackedTarget \
      '{status: 1, yaw_target: 0.05, pitch_target: -0.02, yaw_rate: 0.3, pitch_rate: -0.1}'
"""

import argparse
import math
import os
import pty
import select
import struct
import sys
import time
import tty

SOF = b'\xAA\x55'
MSG_COMMAND = 0x01
MSG_FEEDBACK = 0x02


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_feedback(seq: int, yaw: float, pitch: float) -> bytes:
    payload = struct.pack('<ff', yaw, pitch)
    body = bytes([MSG_FEEDBACK, seq & 0xFF, len(payload)]) + payload
    return SOF + body + struct.pack('<H', crc16_ccitt(body))


class CommandDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.ignored = 0
        self.frames = 0
        self.last = None

    def feed(self, data: bytes):
        self.buffer.extend(data)
        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                if self.buffer[-1:] == b'\xAA':
                    del self.buffer[:-1]
                else:
                    self.buffer.clear()
                return
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 5:
                return
            payload_size = self.buffer[4]
            if payload_size == 0 or payload_size > 32:
                del self.buffer[0]
                continue
            frame_size = 5 + payload_size + 2
            if len(self.buffer) < frame_size:
                return
            received = struct.unpack_from('<H', self.buffer, 5 + payload_size)[0]
            computed = crc16_ccitt(bytes(self.buffer[2:5 + payload_size]))
            if received != computed:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            msg_id = self.buffer[2]
            if msg_id == MSG_COMMAND and payload_size == 17:
                status = self.buffer[5]
                yaw_target, pitch_target, yaw_rate, pitch_rate = struct.unpack_from(
                    '<ffff', self.buffer, 6)
                self.frames += 1
                self.last = (status, yaw_target, pitch_target, yaw_rate, pitch_rate)
            else:
                self.ignored += 1
            del self.buffer[:frame_size]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rate', type=float, default=50.0,
                        help='feedback frames per second (default: 50)')
    args = parser.parse_args()

    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    # Raw mode before the node opens the port: otherwise the line discipline
    # echoes our feedback frames back to the master and the decoder sees noise.
    tty.setraw(slave_fd)
    # Keep the slave fd open: the /dev/pts entry disappears (and the master
    # starts returning EIO) once the last slave descriptor is closed.

    print(f'feedback device: {device}', flush=True)
    print('start the node with -p device:=' + device, flush=True)
    print('waiting for command frames (Ctrl-C to stop)', flush=True)

    decoder = CommandDecoder()
    seq = 0
    started = time.monotonic()
    next_feedback = started
    next_report = started + 1.0
    feedback_sent = 0
    last_frames = 0
    last_crc = 0
    last_ignored = 0

    try:
        while True:
            now = time.monotonic()
            if now >= next_feedback:
                phase = (now - started) * 0.2
                yaw = 0.5 * math.sin(2.0 * math.pi * phase)
                pitch = 0.3 * math.sin(2.0 * math.pi * phase * 0.7 + 0.5)
                os.write(master_fd, encode_feedback(seq, yaw, pitch))
                seq = (seq + 1) & 0xFF
                feedback_sent += 1
                next_feedback += 1.0 / args.rate

            ready, _, _ = select.select([master_fd], [], [], 0.005)
            if ready:
                chunk = os.read(master_fd, 4096)
                if chunk:
                    decoder.feed(chunk)

            if now >= next_report:
                elapsed = now - (next_report - 1.0)
                frames = decoder.frames
                crc_errors = decoder.crc_errors
                ignored = decoder.ignored
                last = decoder.last
                if last is None:
                    detail = 'no command frames yet'
                else:
                    status, yaw_target, pitch_target, yaw_rate, pitch_rate = last
                    detail = (f'status={status} yaw_target={yaw_target:+.4f} '
                              f'pitch_target={pitch_target:+.4f} '
                              f'yaw_rate={yaw_rate:+.3f} pitch_rate={pitch_rate:+.3f}')
                print(f'command rx {(frames - last_frames) / elapsed:5.1f} Hz '
                      f'(total {frames}, crc {crc_errors - last_crc}, '
                      f'ignored {ignored - last_ignored}) | {detail}')
                last_frames = frames
                last_crc = crc_errors
                last_ignored = ignored
                next_report += 1.0
    except KeyboardInterrupt:
        pass
    finally:
        os.close(master_fd)
        os.close(slave_fd)

    print(f'sent {feedback_sent} feedback frames, received {decoder.frames} command frames, '
          f'{decoder.crc_errors} CRC errors')
    return 0


if __name__ == '__main__':
    sys.exit(main())
