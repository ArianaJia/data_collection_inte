"""串口到 MQTT 字节流稳定性测试；凭据由环境变量提供，不使用 protobuf。"""
import argparse
import json
import os
from pathlib import Path
import struct
import threading
import time
import uuid
import zlib
import serial
import paho.mqtt.client as mqtt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='test.yinerda.com')
    parser.add_argument('--port', type=int, default=1002)
    parser.add_argument('--serial', default='COM16')
    parser.add_argument('--topic', default='test_pub')
    parser.add_argument('--count', type=int, default=60)
    parser.add_argument('--interval', type=float, default=1)
    parser.add_argument('--report', type=Path, default=Path('build/dtu_stability.json'))
    args = parser.parse_args()
    if args.count <= 0 or args.interval <= 0:
        parser.error('count and interval must be positive')
    run = uuid.uuid4().hex[:12].encode()
    magic = b'DTUCHK:' + run + b':'
    frame_size = len(magic) + 4 + 128 + 4
    sent = {}
    received = {}
    latencies = []
    buffer = bytearray()
    lock = threading.Lock()
    subscribed = threading.Event()
    failed = threading.Event()
    stats = dict(mqtt_messages=0, mqtt_bytes=0, crc_bad=0, content_bad=0,
                 duplicates=0, disconnects=0, out_of_order=0)
    last_seq = -1
    stopping = False
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id='pc-check-' + run.decode(), protocol=mqtt.MQTTv311)
    client.username_pw_set(os.environ['DTU_TEST_USER'], os.environ['DTU_TEST_PASSWORD'])
    def on_connect(c, u, flags, reason, props):
        print('MQTT CONNACK:', reason, flush=True)
        if reason.is_failure:
            failed.set()
        else:
            c.subscribe(args.topic, qos=0)
    def on_subscribe(c, u, mid, reasons, props):
        if not reasons or any(r.is_failure for r in reasons):
            failed.set()
        else:
            subscribed.set()
    def on_disconnect(c, u, flags, reason, props):
        if not stopping:
            stats['disconnects'] += 1
            print('PC MQTT disconnected:', reason, flush=True)
    def on_message(c, u, msg):
        nonlocal last_seq
        with lock:
            stats['mqtt_messages'] += 1
            stats['mqtt_bytes'] += len(msg.payload)
            buffer.extend(msg.payload)
            while True:
                pos = buffer.find(magic)
                if pos < 0:
                    if len(buffer) >= len(magic):
                        del buffer[:len(buffer) - len(magic) + 1]
                    break
                del buffer[:pos]
                if len(buffer) < frame_size:
                    break
                packet = bytes(buffer[:frame_size])
                if zlib.crc32(packet[:-4]) != struct.unpack('<I', packet[-4:])[0]:
                    stats['crc_bad'] += 1
                    del buffer[0]
                    continue
                del buffer[:frame_size]
                seq = struct.unpack_from('<I', packet, len(magic))[0]
                if packet[len(magic)+4:-4] != bytes((seq+i)%256 for i in range(128)):
                    stats['content_bad'] += 1
                    continue
                if seq in received:
                    stats['duplicates'] += 1
                else:
                    received[seq] = time.monotonic()
                    if seq < last_seq:
                        stats['out_of_order'] += 1
                    last_seq = max(last_seq, seq)
                    if seq in sent:
                        latencies.append((received[seq]-sent[seq])*1000)
                    print('RX valid seq=', seq, flush=True)
    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    error = None
    try:
        client.connect(args.host, args.port, keepalive=20)
        client.loop_start()
        deadline = time.monotonic()+12
        while not subscribed.is_set() and not failed.is_set() and time.monotonic()<deadline:
            time.sleep(.1)
        if not subscribed.is_set():
            raise RuntimeError('MQTT login/subscription failed; no serial data sent')
        with serial.Serial(args.serial,115200,timeout=.1,write_timeout=2) as port:
            for seq in range(args.count):
                data = magic + struct.pack('<I',seq) + bytes((seq+i)%256 for i in range(128))
                data += struct.pack('<I',zlib.crc32(data))
                with lock:
                    sent[seq] = time.monotonic()
                if port.write(data) != len(data):
                    raise RuntimeError('Partial serial write')
                print('TX seq=',seq,'bytes=',len(data),flush=True)
                time.sleep(args.interval)
            time.sleep(8)
    except Exception as exc:
        error = str(exc)
        print('ERROR:',error,flush=True)
    finally:
        stopping = True
        client.disconnect()
        client.loop_stop()
        missing = sorted(set(sent)-set(received))
        report = dict(run=run.decode(), host=args.host, topic=args.topic,
                      sent=len(sent), received=len(set(sent)&set(received)), missing=missing,
                      error=error, **stats,
                      latency_ms=dict(min=min(latencies), max=max(latencies),
                                      mean=sum(latencies)/len(latencies)) if latencies else None)
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2),flush=True)
    return 1 if error or missing or stats['crc_bad'] or stats['content_bad'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
