#!/usr/bin/env python3
"""Core-Y100M DTU 固件串口测试：protobuf -> AA55/长度/序号/CRC -> UART。

DTU 须预先设置 MQTT bitfsae.com:1883，发布 fsae/telemetry，订阅
fsae/command，串口 115200 8N1，保持原始二进制透传。认证在 DTU 配置。
脚本不发送 AT、不建立 MQTT 连接、不自动切换服务器、不添加 CRLF。
PC 通过匹配电平的 USB-UART 接入模块 UART；测试时断开 MCU TX，避免争用。
默认 10 帧GPS 速度及待行驶状态（ready_to_drive=1）；不会自动消耗测试套餐。--count 0 持续发送。
依赖：pip install pyserial protobuf；另需 protoc（可用 --protoc 指定）。
"""
import argparse
import math
import random
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import time

SERIAL_PORT = "COM16"
SERIAL_BAUD = 115200
MAX_PAYLOAD = 6216  # 当前 Core/Inc/fsae_telemetry.pb.h 的 TelemetryFrame_size
PROTO = Path(__file__).with_name("fsae_telemetry.proto")


def load_frame_class(protoc="protoc"):
    """从项目实际 .proto 构造类型，不手写字段号，不修改生成文件。"""
    from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
    compiler = shutil.which(protoc)
    if not compiler:
        raise ValueError("找不到 protoc，请安装或用 --protoc 指定可执行文件")
    with tempfile.TemporaryDirectory(prefix="y100m_proto_") as directory:
        descriptor = Path(directory) / "telemetry.pb"
        subprocess.run([compiler, "--proto_path=" + str(PROTO.parent),
                        "--descriptor_set_out=" + str(descriptor),
                        "--include_imports", PROTO.name], check=True, capture_output=True)
        files = descriptor_pb2.FileDescriptorSet.FromString(descriptor.read_bytes())
    pool = descriptor_pool.DescriptorPool()
    for file in files.file:
        pool.Add(file)
    return message_factory.GetMessageClass(pool.FindMessageTypeByName("fsae.TelemetryFrame"))


def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def build_wire_frame(payload, sequence):
    if not 0 < len(payload) <= MAX_PAYLOAD:
        raise ValueError(f"Protobuf 长度必须为 1..{MAX_PAYLOAD} 字节")
    body = struct.pack("<HB", len(payload), sequence & 0xFF) + payload
    return b"\xaa\x55" + body + struct.pack("<H", crc16_modbus(body))


def gps_payload(frame_class, timestamp_ms, speed):
    """直接编码 GPS 速度和已准备好行驶状态，不附加其他遥测字段。"""
    frame = frame_class()
    frame.timestamp_ms = timestamp_ms & 0xFFFFFFFF
    frame.header.timestamp_ms = frame.timestamp_ms
    frame.motion.gps_speed_kmh = speed
    frame.ready_to_drive = 1
    return frame.SerializeToString()


def show_rx(port):
    # DTU 下行是原始串口字节，可能任意分片；这里不把每次 read 当作一条消息。
    waiting = port.in_waiting
    if waiting:
        data = port.read(min(waiting, 4096))
        print(f"RX chunk {len(data)} bytes HEX={data.hex(' ')} TEXT={data!r}", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=SERIAL_BAUD)
    parser.add_argument("--count", type=int, default=10, help="帧数；0 持续，Ctrl+C 退出")
    parser.add_argument("--interval", type=float, default=0.21,
                        help="串口预计发送完成后的间隔秒数；默认模拟 MCU 的 10+200ms 延迟")
    parser.add_argument("--speed", type=int, default=None, help="固定车速 km/h；不指定则每帧随机 1..120，连续帧不重复")
    parser.add_argument("--sequence", type=int, default=0, help="外层起始序号 0..255")
    parser.add_argument("--payload-file", type=Path, help="已有的裸 TelemetryFrame Protobuf 二进制；不可带 AA55 或 MQTT 头")
    parser.add_argument("--protoc", default="protoc")
    parser.add_argument("--dry-run", action="store_true", help="仅编码和打印，不打开串口")
    parser.add_argument("--listen-only", action="store_true", help="只显示 DTU 下行串口字节，Ctrl+C 退出")
    parser.add_argument("--listen-seconds", type=float, default=2, help="有限发送完成后继续监听的秒数")
    args = parser.parse_args(argv)
    if (args.count < 0 or not 0 <= args.sequence <= 255 or args.interval < 0
            or args.listen_seconds < 0 or not math.isfinite(args.interval)
            or not math.isfinite(args.listen_seconds) or args.baudrate <= 0 or (args.speed is not None and not 0 <= args.speed <= 0xFFFFFFFF)):
        parser.error("帧数/时间必须非负，波特率为正，序号 0..255，车速为 uint32")
    if args.dry_run and (args.listen_only or args.count == 0):
        parser.error("dry-run 不支持无限发送或 listen-only")
    if args.listen_only and args.payload_file:
        parser.error("listen-only 不能同时指定 payload-file")
    port = None
    sent = total = 0
    previous_speed = None
    start = time.monotonic()
    try:
        frame_class = None if args.listen_only else load_frame_class(args.protoc)
        payload = None
        if args.payload_file:
            payload = args.payload_file.read_bytes()
            build_wire_frame(payload, args.sequence)  # 在打开串口前检查长度
            parsed = frame_class()
            parsed.ParseFromString(payload)
            print("输入文件已通过 Protobuf 解码；将原样发送，不改内部字段。")
        if not args.dry_run:
            import serial
            port = serial.Serial(args.port, args.baudrate, bytesize=8, parity="N",
                                 stopbits=1, timeout=0.05, write_timeout=2,
                                 xonxoff=False, rtscts=False, dsrdtr=False)
        print("DTU MQTT 模式：bitfsae.com:1883，发布 fsae/telemetry，订阅 fsae/command")
        print("串口写入成功不代表 MQTT 发布成功；请在服务器确认接收。")
        if not args.payload_file and not args.listen_only:
            print("直接发送 motion.gps_speed_kmh 和 ready_to_drive=1；frame_id/header.seq/source_id 保持当前 MCU 默认值 0。")
        while args.listen_only or args.count == 0 or sent < args.count:
            if args.listen_only:
                show_rx(port)
                time.sleep(0.02)
                continue
            speed = args.speed
            if payload is None and speed is None:
                speed = random.choice([value for value in range(1, 121) if value != previous_speed])
                previous_speed = speed
            data = payload if payload is not None else gps_payload(
                frame_class, int((time.monotonic() - start) * 1000), speed)
            if payload is None:
                print(f"motion.gps_speed_kmh={speed} km/h，ready_to_drive=1", flush=True)
            sequence = (args.sequence + sent) & 0xFF
            wire = build_wire_frame(data, sequence)
            tx_start = time.monotonic()
            if port is not None:
                written = port.write(wire)
                if written != len(wire):
                    raise OSError(f"串口部分写入 {written}/{len(wire)}，停止发送")
            sent += 1
            total += len(wire)
            print(f"{'DRY' if args.dry_run else 'UART-TX'} #{sent} seq={sequence} "
                  f"protobuf={len(data)} wire={len(wire)} HEX={wire.hex(' ')}", flush=True)
            # 按 8N1 线速留足时间，避免将 USB 写入完成误当作 UART 物理发送完成。
            deadline = tx_start + len(wire) * 10 / args.baudrate + args.interval
            while port is not None and time.monotonic() < deadline:
                show_rx(port)
                time.sleep(min(0.02, max(0, deadline - time.monotonic())))
        deadline = time.monotonic() + args.listen_seconds
        while port is not None and time.monotonic() < deadline:
            show_rx(port)
            time.sleep(0.02)
        return 0
    except KeyboardInterrupt:
        print("停止测试")
        return 0
    except Exception as exc:
        print(f"测试失败：{exc}")
        return 1
    finally:
        if port is not None:
            port.close()
        print(f"累计 {sent} 帧 / {total} 串口字节（不等于运营商计费流量）")


if __name__ == "__main__":
    raise SystemExit(main())
