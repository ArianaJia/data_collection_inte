#!/usr/bin/env python3
"""
Core-Y100M 4G DTU MQTT 通讯测试脚本
=====================================
通过 AT 指令配置银尔达 Core-Y100M DTU 模块，
连接到 broker.hivemq.com 进行 MQTT 收发测试。

硬件连接:
  USB转TTL       Core-Y100M
  ─────────      ──────────
  VCC (5V)  -->  VIN
  GND       -->  GND
  TXD       -->  RXD
  RXD       -->  TXD

使用方法:
  1. 安装依赖: pip install pyserial
  2. 修改下方 SERIAL_PORT 为你的串口号 (Windows: COMx, Linux: /dev/ttyUSBx)
  3. 插入 SIM 卡、接好天线，给模块上电
  4. 运行: python core_y100m_mqtt_test.py
"""

import serial
import time
import sys
import threading

# ============================================================
#  配置参数 —— 按实际情况修改
# ============================================================
SERIAL_PORT = "COM12"          # 串口号
SERIAL_BAUD = 115200           # 波特率 (Core-Y100M 默认 115200)
SERIAL_TIMEOUT = 1.0

# MQTT 连接参数
# 免费公共 Broker 列表 (按优先级排列，连接失败自动切换)
MQTT_BROKER_LIST = [
    ("broker.hivemq.com",    1883),
    ("test.mosquitto.org",   1883),
    ("mqtt.eclipseprojects.io", 1883),
    ("broker.emqx.io",       1883),
]
MQTT_BROKER = MQTT_BROKER_LIST[0][0]   # 当前使用的 broker (运行时可能切换)
MQTT_PORT = MQTT_BROKER_LIST[0][1]
MQTT_BROKER_IDX = 0                     # 当前 broker 索引

MQTT_CLIENT_ID = "CoreY100M_Test_001"
MQTT_TOPIC_PUB = "greenhouse/stm32/prediction"
MQTT_TOPIC_SUB = "greenhouse/stm32/prediction"
MQTT_USER = ""
MQTT_PASS = ""

# AT 指令超时 (ms)
AT_TIMEOUT = 10000

# ============================================================
#  DTU AT 指令集 (银尔达 Core-Y100M / 合宙 Air780EP 方案)
#  注意: 不同固件版本指令可能有差异，请以实际模块为准
# ============================================================

ser: serial.Serial = None
recv_buffer = b""
recv_lock = threading.Lock()


def log(msg: str, level: str = "INFO"):
    """统一日志输出"""
    ts = time.strftime("%H:%M:%S", time.localtime())
    prefix = f"[{ts}]"
    if level == "OK":
        print(f"{prefix} ✅ {msg}")
    elif level == "ERR":
        print(f"{prefix} ❌ {msg}")
    elif level == "WARN":
        print(f"{prefix} ⚠️  {msg}")
    elif level == "RX":
        print(f"{prefix} 📩 {msg}")
    elif level == "TX":
        print(f"{prefix} 📨 {msg}")
    else:
        print(f"{prefix}    {msg}")


def recv_callback():
    """后台线程：持续读取串口数据并放入缓冲区"""
    global recv_buffer
    while True:
        try:
            if ser and ser.is_open:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    with recv_lock:
                        recv_buffer += data
            else:
                time.sleep(0.1)
        except (serial.SerialException, OSError):
            break
        except Exception:
            time.sleep(0.5)


def drain_buffer() -> str:
    """读出缓冲区中已有的全部数据（字符串）"""
    global recv_buffer
    with recv_lock:
        data = recv_buffer.decode("utf-8", errors="replace")
        recv_buffer = b""
    return data


def drain_buffer_bytes() -> bytes:
    """读出缓冲区中已有的全部数据（原始字节）"""
    global recv_buffer
    with recv_lock:
        data = bytes(recv_buffer)
        recv_buffer = b""
    return data


def wait_for_response(timeout_s: float = 5.0) -> str:
    """等待模块返回数据，直到超时"""
    deadline = time.time() + timeout_s
    all_data = ""
    while time.time() < deadline:
        chunk = drain_buffer()
        if chunk:
            all_data += chunk
        time.sleep(0.1)
    return all_data


def at_send(cmd: str, wait_ms: int = 500) -> str:
    """发送 AT 指令并等待回复"""
    # 先清空缓冲区
    drain_buffer()

    full_cmd = cmd + "\r\n"
    log(cmd, "TX")
    ser.write(full_cmd.encode("utf-8"))
    ser.flush()

    time.sleep(wait_ms / 1000.0)
    resp = drain_buffer()
    if resp:
        log(resp.strip(), "RX")
    return resp


def at_send_expect(cmd: str, expected: str = "OK", timeout_s: float = 5.0) -> bool:
    """发送 AT 指令并等待期望的响应"""
    drain_buffer()

    log(cmd, "TX")
    ser.write((cmd + "\r\n").encode("utf-8"))
    ser.flush()

    deadline = time.time() + timeout_s
    accumulated = ""
    while time.time() < deadline:
        chunk = drain_buffer()
        if chunk:
            accumulated += chunk
            log(chunk.strip(), "RX")
            if expected.lower() in accumulated.lower():
                return True
        time.sleep(0.2)

    log(f"等待 '{expected}' 超时，已收到: {accumulated.strip()}", "ERR")
    return False


def at_send_expect_any(cmd: str, expectations: list, timeout_s: float = 10.0):
    """
    发送 AT 指令并等待多个可能的响应之一。
    返回 (success, matched_string, full_response)
    """
    drain_buffer()

    log(cmd, "TX")
    ser.write((cmd + "\r\n").encode("utf-8"))
    ser.flush()

    deadline = time.time() + timeout_s
    accumulated = ""
    while time.time() < deadline:
        chunk = drain_buffer()
        if chunk:
            accumulated += chunk
            log(chunk.strip(), "RX")
            accum_lower = accumulated.lower()
            for exp in expectations:
                if exp.lower() in accum_lower:
                    return True, exp, accumulated
        time.sleep(0.2)

    return False, "", accumulated


def check_module_ready() -> bool:
    """检查模块是否响应 AT"""
    log("检查模块 AT 响应...")
    drain_buffer()

    ser.write(b"AT\r\n")
    ser.flush()
    time.sleep(0.3)

    # 先检查是否有数据
    ser.write(b"AT\r\n")
    ser.flush()
    time.sleep(1.5)

    resp = drain_buffer()
    if "OK" in resp or "AT" in resp:
        log("模块 AT 响应正常", "OK")
        return True

    # 可能处于透传模式，尝试退出
    log("无 AT 响应，尝试退出透传模式...", "WARN")
    ser.write(b"+++")
    ser.flush()
    time.sleep(1.5)
    drain_buffer()

    ser.write(b"AT\r\n")
    ser.flush()
    time.sleep(1.5)
    resp = drain_buffer()

    if "OK" in resp:
        log("退出透传成功，AT 响应正常", "OK")
        return True

    log("模块无响应! 请检查: 1)接线 2)供电 3)串口号/波特率", "ERR")
    return False


def step1_check_sim_network():
    """步骤1: 检查 SIM 卡和网络注册"""
    log("=" * 50)
    log("步骤1: 检查 SIM 卡和网络状态")
    log("=" * 50)

    # 查询 SIM 卡状态
    resp = at_send_expect("AT+CPIN?", "READY", 3)
    if resp:
        log("SIM 卡就绪", "OK")
    else:
        log("SIM 卡异常! 请检查 SIM 卡是否插好", "ERR")
        return False

    # 查询信号强度
    resp = at_send("AT+CSQ", 2000)
    # 解析 CSQ: +CSQ: <rssi>,<ber>
    if "+CSQ:" in resp:
        try:
            parts = resp.split("+CSQ:")[1].strip().split(",")
            rssi = int(parts[0])
            if rssi == 99:
                log(f"信号: 无信号 (CSQ={rssi})", "ERR")
                return False
            else:
                log(f"信号强度: CSQ={rssi} (约 {113 + rssi*2} dBm)", "OK")
        except Exception:
            log(f"信号响应: {resp.strip()}", "WARN")

    # 查询网络注册状态
    resp = at_send_expect("AT+CREG?", "+CREG: 0,1", 5)
    if not resp:
        resp = at_send_expect("AT+CREG?", "+CREG: 0,5", 5)
    if resp:
        log("已注册到 4G 网络", "OK")
    else:
        log("网络注册失败! 检查天线和 SIM 卡资费", "ERR")
        return False

    return True


def step1b_query_apn():
    """查询/设置 APN"""
    log("查询 APN...")
    resp = at_send("AT+CGDCONT?", 2000)
    # 大多数物联网卡自动配置 APN, 无需手动设置
    if "+CGDCONT:" in resp:
        log(f"APN 信息: {resp.strip()}")
    else:
        log("未获取到 APN 信息 (可能自动配置)", "WARN")


def step2_configure_mqtt():
    """步骤2: 检测固件类型, 决定 MQTT 连接方式"""
    log("=" * 50)
    log("步骤2: 检测固件类型")
    log("=" * 50)

    # 先查询固件版本
    resp = at_send("AT+CGMR", 1000)
    log(f"固件版本: {resp.strip()}")

    # 固件是 AirM2M_780EP_V1011_LTE_AT → 合宙标准 AT 固件
    # 不支持 AT+MQTTCFG 系列 DTU 指令
    # 只能用 TCP 透传 + 手工 MQTT 报文
    if "LTE_AT" in resp or "AT_" in resp:
        log("检测到合宙标准 AT 固件, 将使用 TCP 透传模式", "WARN")
        return "transparent"

    # 尝试 DTU 固件 MQTT 配置
    log("尝试 DTU 固件 MQTT 配置...")

    cfg_cmd = (f'AT+MQTTCFG="{MQTT_BROKER}",{MQTT_PORT},'
               f'"{MQTT_CLIENT_ID}",'
               f'"{MQTT_USER}","{MQTT_PASS}",60')

    ok, match, resp = at_send_expect_any(cfg_cmd, ["OK", "ERROR"], 5)

    # 必须明确返回 OK 才算 DTU 固件模式
    if ok and "OK" in match.upper() and "ERROR" not in match.upper():
        log("MQTT 配置成功 (DTU固件风格)", "OK")
        return "dtu_fw"
    else:
        log("DTU 固件 MQTT 指令不支持，使用透传模式", "WARN")
        return "transparent"


def step3_connect_mqtt_dtu():
    """步骤3: 用 DTU 固件指令连接 MQTT"""
    log("=" * 50)
    log("步骤3: 连接 MQTT Broker (DTU 固件)")
    log("=" * 50)

    # 打开 MQTT 连接通道
    ok, match, resp = at_send_expect_any("AT+MQTTOPEN=0", ["OK", "ERROR"], 10)
    if not ok or "ERROR" in match.upper():
        log("MQTT 通道打开失败", "ERR")
        return False
    log("MQTT 通道已打开")

    time.sleep(2)

    # 连接到 Broker
    ok, match, resp = at_send_expect_any("AT+MQTTCONN=0", ["OK", "ERROR"], 15)
    if not ok or "ERROR" in match.upper():
        log("MQTT 连接 Broker 失败", "ERR")
        return False
    log("MQTT 已连接到 Broker", "OK")

    return True


def step3_connect_mqtt_transparent():
    """步骤3: 用合宙标准 AT 固件 TCP 透传连接 MQTT Broker

    合宙 Air780EP 标准 AT 固件 TCP 指令流程:
      1. AT+CSTT 或 APN 已自动配置
      2. AT+CIICR  激活 PDP 上下文
      3. AT+CIFSR  获取 IP
      4. AT+CIPSTART="TCP","host",port  建立 TCP 连接
      5. AT+CIPSEND=<len>  进入发送模式
      6. 发送原始 MQTT 报文

    支持多 broker 自动切换：连接失败或 CONNACK 失败时自动尝试下一个。
    """
    global MQTT_BROKER, MQTT_PORT, MQTT_BROKER_IDX

    log("=" * 50)
    log("步骤3: TCP 透传连接 MQTT Broker")
    log("=" * 50)

    # ---- 3a. 激活 PDP 上下文 (获取 IP) ----
    log("激活 PDP 上下文...")
    # 先查询是否已激活
    at_send("AT+CIFSR", 2000)

    # 尝试激活
    ok = at_send_expect("AT+CIICR", "OK", 10)
    if not ok:
        log("CIICR 未返回 OK, 检查 IP 状态...", "WARN")

    time.sleep(1)

    # 获取本机 IP
    resp = at_send("AT+CIFSR", 2000)
    log(f"IP 状态: {resp.strip()}")
    if "." not in resp:
        log("未获取到 IP 地址, 尝试设置 APN...", "WARN")
        at_send_expect('AT+CSTT="CMNET"', "OK", 5)
        at_send_expect("AT+CIICR", "OK", 10)
        time.sleep(1)
        resp = at_send("AT+CIFSR", 2000)
        log(f"IP 状态(重试): {resp.strip()}")
        if "." not in resp:
            log("无法获取 IP, TCP 透传失败", "ERR")
            return False

    # ---- 3b. 遍历 broker 列表尝试连接 ----
    for idx, (broker, port) in enumerate(MQTT_BROKER_LIST):
        MQTT_BROKER = broker
        MQTT_PORT = port
        MQTT_BROKER_IDX = idx

        log(f"--- 尝试 Broker [{idx+1}/{len(MQTT_BROKER_LIST)}]: {broker}:{port} ---")

        # 先关闭之前的 TCP 连接 (避免 ALREADY CONNECT)
        at_send("AT+CIPCLOSE", 1000)
        at_send("AT+CIPSHUT", 1000)
        time.sleep(0.5)

        # 建立 TCP 连接
        log(f"建立 TCP 连接到 {broker}:{port}...")
        cmd = f'AT+CIPSTART="TCP","{broker}",{port}'
        ok_tcp, match_tcp, resp_tcp = at_send_expect_any(
            cmd, ["CONNECT", "OK", "ERROR", "ALREADY"], 30
        )

        if not ok_tcp:
            log(f"TCP 连接超时 ({broker})", "WARN")
            continue
        if "ERROR" in match_tcp.upper():
            log(f"TCP 连接失败 ERROR ({broker})", "WARN")
            continue

        if "CONNECT" in match_tcp.upper() or "OK" in match_tcp.upper() or "ALREADY" in match_tcp.upper():
            log(f"TCP 连接成功 ({broker})", "OK")
        else:
            log(f"TCP 返回未知状态: {match_tcp}", "WARN")

        time.sleep(1)

        # ---- 3c. 发送 MQTT CONNECT 报文 ----
        pkt = build_mqtt_connect(MQTT_CLIENT_ID)
        log(f"构建 MQTT CONNECT 报文 ({len(pkt)} bytes)")

        cmd = f"AT+CIPSEND={len(pkt)}"
        ok = at_send_expect(cmd, ">", 5)
        if not ok:
            log("CIPSEND 未收到 '>' 提示, 尝试直接发送...", "WARN")

        drain_buffer()
        ser.write(pkt)
        ser.flush()
        log(f"已发送 MQTT CONNECT ({len(pkt)} bytes)", "TX")

        # ---- 3d. 等待 CONNACK ----
        log("等待 CONNACK...")
        deadline = time.time() + 15
        accumulated = bytearray()
        while time.time() < deadline:
            chunk = drain_buffer_bytes()
            if chunk:
                accumulated.extend(chunk)

            # CONNACK: 固定头 0x20 + 剩余长度 0x02 + 确认标志 + 返回码
            for i in range(len(accumulated) - 3):
                if accumulated[i] == 0x20 and accumulated[i+1] == 0x02:
                    return_code = accumulated[i+3]
                    if return_code == 0x00:
                        log(f"收到 CONNACK: 连接 {broker} 成功!", "OK")
                        return True
                    else:
                        log(f"收到 CONNACK: {broker} 拒绝, 返回码=0x{return_code:02X}", "WARN")
                        # 尝试下一个 broker
                        break
            else:
                time.sleep(0.3)
                continue
            break  # 内层 for 的 else 没执行 → 找到了 CONNACK，跳出 while

        # 检查是否收到 CLOSED
        accum_str = accumulated.decode("latin-1", errors="replace")
        if "CLOSED" in accum_str:
            log(f"TCP 被 {broker} 关闭 (CLOSED)", "WARN")
        else:
            log(f"等待 CONNACK 超时 ({broker}), 收到 {len(accumulated)} bytes", "WARN")

        # 继续尝试下一个 broker

    # 所有 broker 都失败
    log("所有 Broker 都无法连接!", "ERR")
    return False


def build_mqtt_connect(client_id: str) -> bytes:
    """构建 MQTT 3.1.1 CONNECT 报文"""
    import struct
    import io

    buf = io.BytesIO()

    # 固定头
    buf.write(b"\x10")  # CONNECT

    # 可变头: Protocol Name + Level + Flags + KeepAlive
    var_hdr = bytearray()
    var_hdr.extend(b"\x00\x04MQTT")  # Protocol Name
    var_hdr.append(0x04)              # Protocol Level (3.1.1)
    var_hdr.append(0x02)              # Connect Flags: Clean Session
    var_hdr.extend(struct.pack(">H", 60))  # Keep Alive 60s

    # Payload: Client ID
    cid_bytes = client_id.encode("utf-8")
    payload = bytearray()
    payload.extend(struct.pack(">H", len(cid_bytes)))
    payload.extend(cid_bytes)

    # 剩余长度 = 可变头 + Payload
    remaining = len(var_hdr) + len(payload)
    rml_bytes = encode_vlq(remaining)
    buf.write(rml_bytes)
    buf.write(var_hdr)
    buf.write(payload)

    return buf.getvalue()


def encode_vlq(value: int) -> bytes:
    """MQTT 变长字节编码"""
    result = bytearray()
    while True:
        digit = value % 0x80
        value //= 0x80
        if value > 0:
            digit |= 0x80
        result.append(digit)
        if value == 0:
            break
    return bytes(result)


def build_mqtt_publish(topic: str, payload: str) -> bytes:
    """构建 MQTT PUBLISH 报文 (QoS 0)"""
    import struct
    import io

    buf = io.BytesIO()
    buf.write(b"\x30")  # PUBLISH, QoS 0

    topic_bytes = topic.encode("utf-8")
    payload_bytes = payload.encode("utf-8")
    remaining = 2 + len(topic_bytes) + len(payload_bytes)

    buf.write(encode_vlq(remaining))
    buf.write(struct.pack(">H", len(topic_bytes)))
    buf.write(topic_bytes)
    buf.write(payload_bytes)

    return buf.getvalue()


def step4_publish_dtu(payload: str):
    """步骤4: 用 DTU 固件指令发布 MQTT 消息"""
    log("=" * 50)
    log("步骤4: 发布 MQTT 消息 (DTU 固件)")
    log("=" * 50)

    log(f"Topic: {MQTT_TOPIC_PUB}")
    log(f"Payload: {payload}")

    cmd = f'AT+MQTTPUB=0,"{MQTT_TOPIC_PUB}",0,"{payload}"'
    ok = at_send_expect(cmd, "OK", 5)
    if ok:
        log("消息发布成功", "OK")
        return True
    else:
        log("消息发布失败", "ERR")
        return False


def step4_publish_transparent(payload: str):
    """步骤4: 用透传模式发布 MQTT 消息 (合宙标准 AT 固件 CIPSEND)"""
    log("=" * 50)
    log("步骤4: 发布 MQTT 消息 (透传模式)")
    log("=" * 50)

    log(f"Topic: {MQTT_TOPIC_PUB}")
    log(f"Payload: {payload}")

    pkt = build_mqtt_publish(MQTT_TOPIC_PUB, payload)
    log(f"构建 PUBLISH 报文 ({len(pkt)} bytes)")

    # 用 AT+CIPSEND 发送
    cmd = f"AT+CIPSEND={len(pkt)}"
    ok = at_send_expect(cmd, ">", 5)
    if ok:
        log("进入 CIPSEND 发送模式", "OK")
    else:
        log("未收到 '>' (可能已处于透传模式), 直接发送", "WARN")

    drain_buffer()
    ser.write(pkt)
    ser.flush()
    log(f"已发送 PUBLISH 报文 ({len(pkt)} bytes)", "TX")

    # 等待 SEND OK
    time.sleep(1)
    resp = drain_buffer()
    if resp:
        log(f"响应: {resp.strip()}", "RX")

    log("消息已发送 (QoS 0, 无确认)", "OK")
    return True


def build_mqtt_subscribe(topic: str, packet_id: int = 1) -> bytes:
    """构建 MQTT SUBSCRIBE 报文 (QoS 0)"""
    import struct
    import io

    buf = io.BytesIO()
    buf.write(b"\x82")  # SUBSCRIBE, 固定头

    topic_bytes = topic.encode("utf-8")
    # 剩余长度 = PacketID(2) + TopicLen(2) + Topic + QoS(1)
    remaining = 2 + 2 + len(topic_bytes) + 1
    buf.write(encode_vlq(remaining))
    buf.write(struct.pack(">H", packet_id))     # Packet Identifier
    buf.write(struct.pack(">H", len(topic_bytes)))  # Topic Length
    buf.write(topic_bytes)                      # Topic
    buf.write(b"\x00")                          # QoS 0

    return buf.getvalue()


def step6_subscribe_transparent():
    """步骤6: 透传模式订阅 Topic"""
    log("=" * 50)
    log("步骤6: 订阅 Topic (透传模式)")
    log("=" * 50)

    log(f"订阅 Topic: {MQTT_TOPIC_SUB}")

    pkt = build_mqtt_subscribe(MQTT_TOPIC_SUB)
    log(f"构建 SUBSCRIBE 报文 ({len(pkt)} bytes)")

    cmd = f"AT+CIPSEND={len(pkt)}"
    ok = at_send_expect(cmd, ">", 5)
    if not ok:
        log("CIPSEND 未收到 '>' 提示, 尝试直接发送...", "WARN")

    drain_buffer()
    ser.write(pkt)
    ser.flush()
    log(f"已发送 SUBSCRIBE 报文 ({len(pkt)} bytes)", "TX")

    # 等待 SUBACK (0x90)
    log("等待 SUBACK...")
    deadline = time.time() + 10
    accumulated = bytearray()
    while time.time() < deadline:
        chunk = drain_buffer_bytes()
        if chunk:
            accumulated.extend(chunk)

        # SUBACK: 0x90 + 剩余长度(0x03) + PacketID(2 bytes) + ReturnCode(1 byte)
        for i in range(len(accumulated) - 3):
            if accumulated[i] == 0x90 and accumulated[i+1] == 0x03:
                return_code = accumulated[i+4]
                if return_code in (0x00, 0x01, 0x02):
                    log(f"收到 SUBACK: 订阅成功 (返回码=0x{return_code:02X})", "OK")
                    return True
                elif return_code == 0x80:
                    log("收到 SUBACK: 订阅失败 (服务器拒绝)", "ERR")
                    return False
        time.sleep(0.3)

    log("等待 SUBACK 超时 (可能已成功，QoS 0 不保证送达)", "WARN")
    return True  # 不阻塞流程


def step6_subscribe_dtu():
    """步骤6: DTU 固件模式订阅 Topic"""
    log("=" * 50)
    log("步骤6: 订阅 Topic (DTU 固件)")
    log("=" * 50)

    log(f"订阅 Topic: {MQTT_TOPIC_SUB}")

    cmd = f'AT+MQTTSUB=0,"{MQTT_TOPIC_SUB}",0'
    ok = at_send_expect(cmd, "OK", 5)
    if ok:
        log("Topic 订阅成功", "OK")
        return True
    else:
        log("Topic 订阅失败", "ERR")
        return False


def parse_mqtt_publish(data: bytes):
    """
    从原始 TCP 数据流中解析 MQTT PUBLISH 报文。
    返回 (topic, payload_bytes) 或 (None, None)
    """
    if len(data) < 4:
        return None, None

    # 检查固定头: PUBLISH 是 0x30 ~ 0x3F
    byte0 = data[0]
    if (byte0 & 0xF0) != 0x30:
        return None, None

    # 解析剩余长度 (VLQ)
    idx = 1
    remaining = 0
    multiplier = 1
    while idx < len(data) and idx < 5:
        b = data[idx]
        remaining += (b & 0x7F) * multiplier
        idx += 1
        if (b & 0x80) == 0:
            break
        multiplier *= 128

    if idx >= len(data):
        return None, None

    # 跳过 QoS > 0 时的 Packet Identifier (不处理，直接跳过)
    qos = (byte0 >> 1) & 0x03
    payload_start = idx

    # 解析 Topic
    if payload_start + 2 > len(data):
        return None, None
    topic_len = (data[payload_start] << 8) | data[payload_start + 1]
    payload_start += 2

    if payload_start + topic_len > len(data):
        return None, None
    topic = data[payload_start:payload_start + topic_len].decode("utf-8", errors="replace")
    payload_start += topic_len

    # QoS 1/2 有 Packet Identifier (2 bytes)
    if qos > 0:
        payload_start += 2

    payload = data[payload_start:]

    return topic, payload


# 接收计数器 (线程安全)
recv_msg_count = 0
recv_msg_lock = threading.Lock()

# 控制后台接收线程退出
recv_running = True


def background_mqtt_receiver():
    """
    后台线程：持续监听串口数据，解析 MQTT PUBLISH 报文并打印。
    按 Ctrl+C 退出。

    合宙 AT 固件在收到 TCP 数据时会打印:
      +IPD,<len>:<data>
    例如: +IPD,55:0x30 0x1D ... (MQTT PUBLISH 报文)
    普通 AT 回显和 +IPD 需要过滤，只提取 MQTT 报文部分。
    """
    global recv_msg_count, recv_running

    log("📡 后台接收监听已启动 (按 Ctrl+C 停止)...")
    log(f"   监听 Topic: {MQTT_TOPIC_SUB}")

    buf = bytearray()
    last_ping = time.time()
    ping_interval = 30  # 30 秒发一次 PINGREQ

    while recv_running:
        try:
            # 读取新数据
            chunk = drain_buffer_bytes()
            if chunk:
                buf.extend(chunk)

            # ========================================================
            # 过滤 AT 回显和 +IPD 头，只保留 MQTT 报文
            # ========================================================
            # 策略: 扫描 buffer，找到每个 0x30 (PUBLISH 固定头)，
            # 从该位置开始尝试解析完整报文

            while True:
                if len(buf) < 4:
                    break

                # 查找第一个 PUBLISH 固定头 (0x30 ~ 0x3F)
                publish_start = -1
                for i in range(len(buf)):
                    b = buf[i]
                    if (b & 0xF0) == 0x30:
                        # 确保前面是行首或 +IPD 数据段开始
                        publish_start = i
                        break

                if publish_start < 0:
                    # 没有 PUBLISH 报文，只保留最后几个字节(可能是不完整的帧头)
                    if len(buf) > 64:
                        # 丢弃旧的无关数据
                        buf = buf[-16:]
                    break

                # 从这个位置开始尝试解析
                candidate = bytes(buf[publish_start:])
                publish_len = _calc_publish_len(candidate)

                if publish_len <= 0 or publish_len > len(candidate):
                    # 报文不完整，等待更多数据
                    # 保留从 publish_start 开始的数据
                    buf = buf[publish_start:]
                    break

                # 报文完整！提取并解析
                full_pkt = candidate[:publish_len]
                topic, payload = parse_mqtt_publish(full_pkt)

                # 丢弃已处理的数据 (publish_start + publish_len)
                consumed = publish_start + publish_len
                buf = buf[consumed:]

                if topic is not None:
                    with recv_msg_lock:
                        recv_msg_count += 1
                        count = recv_msg_count

                    ts = time.strftime("%H:%M:%S", time.localtime())
                    print(f"\n[RECV #{count}] ─────────────────────────────")
                    print(f"  Topic : {topic}")
                    try:
                        payload_str = payload.decode("utf-8")
                        print(f"  Payload: {payload_str}")
                    except UnicodeDecodeError:
                        print(f"  Payload (hex): {payload.hex()}")
                    print(f"  Time  : {ts}")
                    print(f"──────────────────────────────────────────\n")
                # 继续循环，处理 buffer 中剩余的数据

            # 定期发送 PINGREQ 保活
            now = time.time()
            if now - last_ping > ping_interval:
                ser.write(b"\xC0\x00")
                ser.flush()
                last_ping = now

            time.sleep(0.3)

        except (serial.SerialException, OSError):
            break
        except Exception as e:
            log(f"接收线程异常: {e}", "WARN")
            time.sleep(1)


def _calc_publish_len(data: bytes) -> int:
    """计算一个 MQTT PUBLISH 报文的总字节数"""
    if len(data) < 4:
        return 0

    byte0 = data[0]
    if (byte0 & 0xF0) != 0x30:
        return 0

    # 解析 VLQ 剩余长度
    idx = 1
    remaining = 0
    multiplier = 1
    while idx < len(data) and idx < 5:
        b = data[idx]
        remaining += (b & 0x7F) * multiplier
        idx += 1
        if (b & 0x80) == 0:
            break
        multiplier *= 128

    total = idx + remaining  # 固定头 + 剩余长度 = 总报文长度
    return total


def step5_ping_test():
    """步骤5: PING 保活测试"""
    log("=" * 50)
    log("步骤5: PING 保活测试")
    log("=" * 50)

    # 发送 PINGREQ (0xC0 0x00)
    drain_buffer()
    ser.write(b"\xC0\x00")
    ser.flush()

    deadline = time.time() + 5
    accumulated = bytearray()
    while time.time() < deadline:
        chunk = drain_buffer_bytes()
        if chunk:
            accumulated.extend(chunk)
        if len(accumulated) >= 2 and accumulated[0] == 0xD0:
            log("收到 PINGRESP，保活正常", "OK")
            return True
        time.sleep(0.3)

    log("未收到 PINGRESP", "WARN")
    return False


def main():
    global ser

    print("\n" + "=" * 55)
    print("  Core-Y100M 4G DTU MQTT 通讯测试工具")
    print("=" * 55)
    print(f"  串口: {SERIAL_PORT}  @ {SERIAL_BAUD} bps")
    print(f"  Broker: {MQTT_BROKER}:{MQTT_PORT}")
    print(f"  Topic:  {MQTT_TOPIC_PUB}")
    print("=" * 55 + "\n")

    # ---------- 打开串口 ----------
    log(f"打开串口 {SERIAL_PORT}...")
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=1,
        )
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except serial.SerialException as e:
        log(f"无法打开串口: {e}", "ERR")
        log("请检查: 1)串口号是否正确 2)串口是否被其他程序占用", "WARN")
        sys.exit(1)

    # 启动后台接收线程
    recv_thread = threading.Thread(target=recv_callback, daemon=True)
    recv_thread.start()

    try:
        # ---- 1. 检查模块 ----
        if not check_module_ready():
            log("模块无响应，测试终止", "ERR")
            return

        # ---- 2. SIM + 网络 ----
        if not step1_check_sim_network():
            log("网络注册失败，测试终止", "ERR")
            return

        step1b_query_apn()

        # ---- 3. 配置 MQTT ----
        mode = step2_configure_mqtt()

        # ---- 4. 连接 Broker ----
        if mode == "dtu_fw":
            connected = step3_connect_mqtt_dtu()
        else:
            connected = step3_connect_mqtt_transparent()

        if not connected:
            log("MQTT 连接失败", "ERR")
            return

        time.sleep(2)

        # ---- 5. 发布测试消息 ----
        import json
        test_payload = json.dumps({
            "device_id": MQTT_CLIENT_ID,
            "type": "test",
            "message": "Hello from Core-Y100M!",
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }, ensure_ascii=False)

        if mode == "dtu_fw":
            pub_ok = step4_publish_dtu(test_payload)
        else:
            pub_ok = step4_publish_transparent(test_payload)

        if pub_ok:
            time.sleep(1)
            # 再发一条
            test_payload2 = json.dumps({
                "device_id": MQTT_CLIENT_ID,
                "type": "test",
                "T": 25.0,
                "H": 60.0,
                "UV": 3.5,
                "VPD": 1.2,
                "pred": 0,
            }, ensure_ascii=False)

            if mode == "dtu_fw":
                step4_publish_dtu(test_payload2)
            else:
                step4_publish_transparent(test_payload2)

        # ---- 6. PING 测试 ----
        if mode == "transparent":
            step5_ping_test()

        # ---- 7. 订阅 Topic ----
        if mode == "dtu_fw":
            sub_ok = step6_subscribe_dtu()
        else:
            sub_ok = step6_subscribe_transparent()

        # ---- 8. 进入接收监听模式 ----
        log("=" * 50)
        if sub_ok:
            log("订阅成功，进入接收监听模式...", "OK")
        else:
            log("订阅可能未成功，仍尝试监听接收...", "WARN")

        log(f"本机订阅 Topic: {MQTT_TOPIC_SUB}")
        log(f"本机发布 Topic: {MQTT_TOPIC_PUB}")
        log("")
        log("💡 提示: 用 MQTTX 或在线工具向该 Topic 发消息即可在本窗口看到")
        log("   在线工具: http://www.hivemq.com/demos/websocket-client/")
        log("   按 Ctrl+C 退出监听")
        log("=" * 50)

        # 启动后台接收线程
        global recv_running
        recv_running = True
        recv_thread2 = threading.Thread(target=background_mqtt_receiver, daemon=True)
        recv_thread2.start()

        # 主线程阻塞，等待用户 Ctrl+C
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        log("用户中断测试")
        recv_running = False
        with recv_msg_lock:
            total = recv_msg_count
        log(f"共收到 {total} 条消息")
    except Exception as e:
        log(f"测试异常: {e}", "ERR")
        import traceback
        traceback.print_exc()
    finally:
        recv_running = False
        if ser and ser.is_open:
            ser.close()
            log("串口已关闭")
        log(f"本次使用 Broker: {MQTT_BROKER}:{MQTT_PORT}")


if __name__ == "__main__":
    main()
