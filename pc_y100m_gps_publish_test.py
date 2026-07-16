#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Core-Y100M 标准 AT 固件 MQTT Publish 测试脚本。

最终上云路线对齐 reference/protobuf-master/local_sim2.py:
1. MQTT broker: 82.157.204.124:1883
2. MQTT topic:  fsae/telemetry
3. MQTT payload: 直接发送 TelemetryFrame protobuf 二进制

本脚本为最小验证版：
- 仅编码 TelemetryFrame.header.timestamp_ms
- 仅编码 TelemetryFrame.motion.gps_speed_kmh

依赖:
    pip install pyserial
"""

import io
import struct
import sys
import threading
import time

try:
    import serial
except ImportError as exc:
    print("Missing dependency: pyserial")
    print("Install it with: python -m pip install pyserial")
    raise SystemExit(1) from exc


SERIAL_PORT = "COM16"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.2

APN = "CMNET"

MQTT_BROKER = "82.157.204.124"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "CoreY100M_GPS_Test_001"
MQTT_TOPIC = "fsae/telemetry"

GPS_SPEED_KMH = 42

AT_GUARD_S = 0.2
RESP_POLL_S = 0.05

# 来自 fsae_telemetry.proto / .pb.h
TELEMETRY_FRAME_HEADER_TAG = 26
TELEMETRY_FRAME_MOTION_TAG = 33
PACKET_HEADER_TIMESTAMP_MS_TAG = 1
MOTION_GPS_SPEED_KMH_TAG = 1

ser = None
recv_buffer = bytearray()
recv_lock = threading.Lock()
recv_thread_running = True


def log(msg, level="INFO"):
    ts = time.strftime("%H:%M:%S", time.localtime())
    prefix = {
        "INFO": "   ",
        "TX": "[TX]",
        "RX": "[RX]",
        "OK": "[OK]",
        "WARN": "[WARN]",
        "ERR": "[ERR]",
    }.get(level, "   ")
    print(f"[{ts}] {prefix} {msg}")


def recv_worker():
    global recv_buffer, recv_thread_running

    while recv_thread_running:
        try:
            if ser and ser.is_open:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    with recv_lock:
                        recv_buffer.extend(data)
            else:
                time.sleep(0.1)
        except Exception:
            time.sleep(0.1)


def drain_text():
    global recv_buffer

    with recv_lock:
        data = bytes(recv_buffer)
        recv_buffer.clear()
    return data.decode("utf-8", errors="replace")


def drain_bytes():
    global recv_buffer

    with recv_lock:
        data = bytes(recv_buffer)
        recv_buffer.clear()
    return data


def send_raw(data: bytes):
    ser.write(data)
    ser.flush()


def at_send(cmd: str):
    drain_bytes()
    log(cmd, "TX")
    send_raw((cmd + "\r\n").encode("utf-8"))
    time.sleep(AT_GUARD_S)


def at_send_expect_any(cmd: str, expected_list, timeout_s=5.0):
    at_send(cmd)
    deadline = time.time() + timeout_s
    accumulated = ""

    while time.time() < deadline:
        chunk = drain_text()
        if chunk:
            accumulated += chunk
            log(repr(chunk), "RX")
            lowered = accumulated.lower()
            for expected in expected_list:
                if expected.lower() in lowered:
                    return True, expected, accumulated
        time.sleep(RESP_POLL_S)

    return False, "", accumulated


def at_send_expect(cmd: str, expected="OK", timeout_s=5.0):
    ok, matched, response = at_send_expect_any(cmd, [expected, "ERROR", "FAIL"], timeout_s)
    if not ok:
        log(f"{cmd} timeout, expected={expected}, response={response!r}", "ERR")
        return False, response

    if matched.upper() in ("ERROR", "FAIL"):
        log(f"{cmd} failed, response={response!r}", "ERR")
        return False, response

    return True, response


def encode_vlq(value: int) -> bytes:
    encoded = bytearray()
    while True:
        digit = value & 0x7F
        value >>= 7
        if value:
            digit |= 0x80
        encoded.append(digit)
        if not value:
            break
    return bytes(encoded)


def pb_key(field_number: int, wire_type: int) -> bytes:
    return encode_vlq((field_number << 3) | wire_type)


def pb_varint_field(field_number: int, value: int) -> bytes:
    return pb_key(field_number, 0) + encode_vlq(value)


def pb_length_field(field_number: int, payload: bytes) -> bytes:
    return pb_key(field_number, 2) + encode_vlq(len(payload)) + payload


def encode_packet_header(timestamp_ms: int) -> bytes:
    return pb_varint_field(PACKET_HEADER_TIMESTAMP_MS_TAG, timestamp_ms)


def encode_motion_telemetry(gps_speed_kmh: int) -> bytes:
    return pb_varint_field(MOTION_GPS_SPEED_KMH_TAG, gps_speed_kmh)


def encode_telemetry_frame(gps_speed_kmh: int) -> bytes:
    timestamp_ms = int(time.time() * 1000) & 0xFFFFFFFF
    header = encode_packet_header(timestamp_ms)
    motion = encode_motion_telemetry(gps_speed_kmh)

    frame = bytearray()
    frame.extend(pb_length_field(TELEMETRY_FRAME_HEADER_TAG, header))
    frame.extend(pb_length_field(TELEMETRY_FRAME_MOTION_TAG, motion))
    return bytes(frame)


def build_mqtt_connect(client_id: str, keepalive=60) -> bytes:
    variable_header = bytearray()
    variable_header.extend(b"\x00\x04MQTT")
    variable_header.append(0x04)
    variable_header.append(0x02)
    variable_header.extend(struct.pack(">H", keepalive))

    client_id_bytes = client_id.encode("utf-8")
    payload = struct.pack(">H", len(client_id_bytes)) + client_id_bytes
    remaining = len(variable_header) + len(payload)

    stream = io.BytesIO()
    stream.write(b"\x10")
    stream.write(encode_vlq(remaining))
    stream.write(variable_header)
    stream.write(payload)
    return stream.getvalue()


def build_mqtt_publish(topic: str, payload: bytes) -> bytes:
    topic_bytes = topic.encode("utf-8")
    remaining = 2 + len(topic_bytes) + len(payload)

    stream = io.BytesIO()
    stream.write(b"\x30")
    stream.write(encode_vlq(remaining))
    stream.write(struct.pack(">H", len(topic_bytes)))
    stream.write(topic_bytes)
    stream.write(payload)
    return stream.getvalue()


def check_basic():
    log("Checking basic AT communication")
    if not at_send_expect("AT", "OK", 3)[0]:
        return False
    if not at_send_expect("ATE0", "OK", 3)[0]:
        return False
    if not at_send_expect("AT+CPIN?", "READY", 5)[0]:
        return False
    if not at_send_expect("AT+CSQ", "+CSQ:", 5)[0]:
        return False
    return True


def check_lte_register():
    log("Checking LTE registration")

    for _ in range(10):
        ok, response = at_send_expect("AT+CEREG?", "+CEREG:", 5)
        if ok:
            compact = response.replace("\r", "").replace("\n", "")
            if ",1" in compact or ",5" in compact:
                log(f"LTE registered: {compact}", "OK")
                return True
            log(f"LTE not ready yet: {compact}", "WARN")
        time.sleep(2)

    return False


def check_attach():
    log("Checking packet attach")

    for _ in range(5):
        ok, response = at_send_expect("AT+CGATT?", "+CGATT:", 5)
        if ok:
            compact = response.replace("\r", "").replace("\n", "")
            if "+CGATT: 1" in compact:
                log("Packet domain attached", "OK")
                return True
            log(f"Not attached yet: {compact}", "WARN")
        time.sleep(2)

    return False


def close_old_session():
    log("Closing previous TCP session if any")
    at_send("AT+CIPCLOSE")
    time.sleep(1)
    at_send("AT+CIPSHUT")
    time.sleep(2)
    drain_bytes()


def activate_pdp():
    log("Activating PDP context")

    ok, _ = at_send_expect(f'AT+CSTT="{APN}"', "OK", 8)
    if not ok:
        log("AT+CSTT failed, APN may need adjustment or command may differ", "WARN")

    if not at_send_expect("AT+CIICR", "OK", 15)[0]:
        return False

    ok, response = at_send_expect("AT+CIFSR", ".", 8)
    if not ok:
        return False

    log(f"IP info: {response.strip()}", "OK")
    return True


def open_tcp():
    log(f"Opening TCP to {MQTT_BROKER}:{MQTT_PORT}")
    command = f'AT+CIPSTART="TCP","{MQTT_BROKER}",{MQTT_PORT}'
    ok, _, response = at_send_expect_any(
        command,
        ["CONNECT OK", "ALREADY CONNECT", "OK", "ERROR", "FAIL"],
        30,
    )

    if not ok:
        log(f"CIPSTART timeout: {response!r}", "ERR")
        return False

    response_upper = response.upper()
    if "ERROR" in response_upper or "FAIL" in response_upper:
        log(f"CIPSTART failed: {response!r}", "ERR")
        return False

    log(f"TCP connected: {response.strip()}", "OK")
    return True


def cipsend_packet(packet: bytes, expect_send_ok=True):
    ok, response = at_send_expect(f"AT+CIPSEND={len(packet)}", ">", 8)
    if not ok:
        log(f"CIPSEND prompt missing: {response!r}", "ERR")
        return False, response

    log(f"Sending {len(packet)} raw bytes", "TX")
    send_raw(packet)

    deadline = time.time() + 10
    accumulated = b""

    while time.time() < deadline:
        chunk = drain_bytes()
        if chunk:
            accumulated += chunk
            log(repr(chunk.decode("utf-8", errors="replace")), "RX")
            if expect_send_ok and b"SEND OK" in accumulated:
                return True, accumulated
            if b"ERROR" in accumulated or b"FAIL" in accumulated:
                return False, accumulated
        time.sleep(RESP_POLL_S)

    if expect_send_ok:
        log(f"SEND OK not observed, received={accumulated!r}", "WARN")
    return True, accumulated


def mqtt_connect():
    log("Sending MQTT CONNECT")
    packet = build_mqtt_connect(MQTT_CLIENT_ID)
    ok, _ = cipsend_packet(packet, expect_send_ok=True)
    if not ok:
        return False

    deadline = time.time() + 10
    accumulated = bytearray()

    while time.time() < deadline:
        chunk = drain_bytes()
        if chunk:
            accumulated.extend(chunk)
            for index in range(len(accumulated) - 3):
                if accumulated[index] == 0x20 and accumulated[index + 1] == 0x02:
                    return_code = accumulated[index + 3]
                    if return_code == 0x00:
                        log("MQTT CONNACK success", "OK")
                        return True
                    log(f"MQTT CONNACK rejected, code=0x{return_code:02X}", "ERR")
                    return False
        time.sleep(RESP_POLL_S)

    log("Waiting for CONNACK timed out", "ERR")
    return False


def mqtt_publish_telemetry_frame(gps_speed_kmh: int):
    protobuf_payload = encode_telemetry_frame(gps_speed_kmh)
    mqtt_packet = build_mqtt_publish(MQTT_TOPIC, protobuf_payload)

    log(f"Publishing MQTT topic={MQTT_TOPIC}")
    log(f"Broker={MQTT_BROKER}:{MQTT_PORT}")
    log(f"TelemetryFrame bytes={len(protobuf_payload)}")
    log(f"GPS speed encoded into TelemetryFrame.motion.gps_speed_kmh={gps_speed_kmh}")

    ok, _ = cipsend_packet(mqtt_packet, expect_send_ok=True)
    if not ok:
        return False

    log("TelemetryFrame MQTT publish sent", "OK")
    return True


def main():
    global ser, recv_thread_running

    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUD,
            timeout=SERIAL_TIMEOUT,
        )
    except Exception as exc:
        log(f"Failed to open serial port: {exc}", "ERR")
        return 1

    log(f"Serial opened: {SERIAL_PORT} @ {SERIAL_BAUD}", "OK")

    receiver = threading.Thread(target=recv_worker, daemon=True)
    receiver.start()

    try:
        if not check_basic():
            return 2
        if not check_lte_register():
            log("LTE registration failed", "ERR")
            return 3
        if not check_attach():
            log("Packet attach failed", "ERR")
            return 4

        close_old_session()

        if not activate_pdp():
            log("PDP activation failed", "ERR")
            return 5
        if not open_tcp():
            log("TCP open failed", "ERR")
            return 6
        if not mqtt_connect():
            log("MQTT CONNECT failed", "ERR")
            return 7
        if not mqtt_publish_telemetry_frame(GPS_SPEED_KMH):
            log("MQTT publish failed", "ERR")
            return 8

        log("Full publish flow completed", "OK")
        return 0
    finally:
        recv_thread_running = False
        time.sleep(0.2)
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
