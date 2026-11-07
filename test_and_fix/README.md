# 测试与修复工具归档

本目录统一保存工程自有的测试代码、编译验证入口和历史迁移脚本。命令默认从工程根目录执行，固件、硬件工程和第三方库仍保留原有目录。

## 目录结构

```text
test_and_fix/
├── communication/  # PC 串口、AT、MQTT 通信测试
├── firmware/       # 固件侧回归测试
├── build/          # 编译验证入口
├── migrations/     # 历史协议迁移脚本
└── README.md
```

## 通信与回归测试

- `communication/core_y100m_mqtt_test.py`：Core-Y100M DTU 固件串口封包测试；不发 AT 或 MQTT 报文，默认 COM16 / 115200 8N1，发送 10 帧测试 GPS 速度，ready_to_drive=0。
- `communication/pc_y100m_gps_publish_test.py`：Y100M + TelemetryFrame GPS 发布链路测试。
- `firmware/publish_presence_test.c`：使用真实发布编码与 nanopb 解码检查字段存在性。它不属于正常固件；需要在 HAL tick 初始化后、采集及 RTOS 任务启动前调用 `Publish_PresenceRegressionTest()`，返回 0 表示成功，否则返回失败行号。测试会覆盖遥测缓存，不可在运行中的车辆上调用。
- `build/4g_verify.mk`：基于 CubeIDE Debug makefile 验证串口、DTU 通信及日志模块，兼容新模块尚未加入生成列表的情况。
- `build/verify_firmware.py`：不依赖 IDE 生成文件，按 `STM32Make.make` 列表并自动补齐 `Core/Src` 源码进行完整编译与链接；输出在 `build/refactor_verify/`，不烧录硬件。需 ARM GCC 11 或更新版本；DTU 后端不需要本地 MQTT 凭据头文件。

Python 通信测试依赖 `pyserial`，串口和连接参数见对应脚本。使用示例：

```powershell
python test_and_fix/communication/core_y100m_mqtt_test.py
python test_and_fix/communication/pc_y100m_gps_publish_test.py
```

编译验证需要先由 STM32CubeIDE 生成 `Debug/makefile`，并将 GNU Make 与 `arm-none-eabi-gcc` 加入 PATH。此入口依赖 Debug 工作目录及其生成规则：

```powershell
make -C Debug -f ../test_and_fix/build/4g_verify.mk Core/Src/usart.o Core/Src/modem_dtu.o Core/Src/app_log.o
```

完整编译与链接验证（可用 `--cc` 指定 ARM GCC 可执行文件）：

```powershell
python test_and_fix/build/verify_firmware.py
```

## 历史协议迁移工具

- `migrations/update_can_protocol.py`：CAN 协议宏、数据结构与过滤器的迁移辅助脚本。
- `migrations/update_can_decode.py`：CAN 解码与缓存迁移辅助脚本。
- `migrations/finish_can_protocol.py`：CAN 数据新鲜度与发布映射收尾辅助脚本。

这三个脚本会修改工程源文件，仅用于历史协议迁移复现，不是日常测试入口，也不应在当前工程上自动重跑。它们使用工程根目录的相对路径，运行前应先检查 Git 工作区。

## 归档约定

- 新增自有测试、验证及修复工具统一放入本目录，并在本文登记用途与执行条件。
- 编译产物与运行日志留在构建目录，不作为测试源码提交。
- 硬件备份继续放在 `hardware/`，协议源文件和固件源码继续保留原位置。

## Core-Y100M DTU 固件测试

安装 `pyserial`、`protobuf` 并将 `protoc` 加入 PATH（或通过 `--protoc` 指定）。脚本读取同目录实际 `.proto`，在临时目录生成描述符，不修改生成代码。

```powershell
python test_and_fix/communication/core_y100m_mqtt_test.py --dry-run --count 2
python test_and_fix/communication/core_y100m_mqtt_test.py --port COM16 --count 10
python test_and_fix/communication/core_y100m_mqtt_test.py --port COM16 --count 0
python test_and_fix/communication/core_y100m_mqtt_test.py --port COM16 --listen-only
python test_and_fix/communication/core_y100m_mqtt_test.py --payload-file capture.pb --count 1
python -m unittest discover -s test_and_fix/communication -p test_dtu_transport.py
```

`capture.pb` 必须是裸 TelemetryFrame 二进制；脚本校验可解码后原样加封包，不能传 HEX 文本、已封包数据或 MQTT 报文。默认每帧将 1..120 km/h 随机数直接写入 motion.gps_speed_kmh，连续帧不重复；ready_to_drive=0（没有 ECU 状态源）。不填充 fast_telemetry、vehicle_state、电压、电流或温度。速度是指定的测试数值，并非从 GPS 硬件采集。--speed 可指定固定车速；--payload-file 仍原样发送文件，不改内部字段。

固件与脚本统一使用 frame_id=header.seq、header.source_id=1，外层序号取帧号低 8 位（255 后回绕到 0），默认首帧编号为 1。固件的所有发布主题共用一个计数器；构建或发送失败也消耗编号，重试时重新采集缓存并使用新编号。固件 timestamp_ms/header.timestamp_ms 使用 HAL 启动毫秒数，PC 使用 Unix 毫秒数截断为 uint32；两者不代表相同时间基准。固件保留真实 CAN/BMS/温度映射，不使用脚本中的随机测试车速。

默认发送后间隔 0.21 秒，另计预计 UART 线速耗时；固件等待 UART 完成后延迟 10+200 ms，PC/USB 调度不保证与 MCU 精确同步。离线 Python 回归覆盖 CRC、长度边界及内外序号回绕。firmware/publish_presence_test.c 另覆盖所有主题的帧头字段及真实缓存车速；需按上述条件在测试固件中运行。

DTU 的 MQTT 配置在设备端保存：bitfsae.com:1883，发布 fsae/telemetry、订阅 fsae/command。脚本不保存密码，不联网验证、不更改模块设置。RX 显示原始字节分片，不视为服务器确认。串口写入成功只证明 PC 已交给串口驱动；端到端须在服务器检查 framed_ok、CRC 和解码值。测试时 PC 与 MCU 不可同时驱动模块 RX。Ctrl+C 关闭串口退出。
