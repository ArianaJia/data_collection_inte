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

- `communication/core_y100m_mqtt_test.py`：Core-Y100M 串口、AT 和 MQTT 基础连通性测试。
- `communication/pc_y100m_gps_publish_test.py`：Y100M + TelemetryFrame GPS 发布链路测试。
- `firmware/publish_presence_test.c`：使用真实发布编码与 nanopb 解码检查字段存在性。它不属于正常固件；需要在 HAL tick 初始化后、采集及 RTOS 任务启动前调用 `Publish_PresenceRegressionTest()`，返回 0 表示成功，否则返回失败行号。测试会覆盖遥测缓存，不可在运行中的车辆上调用。
- `build/4g_verify.mk`：基于 CubeIDE Debug makefile 验证串口、AT 通信及日志模块，兼容新模块尚未加入生成列表的情况。
- `build/verify_firmware.py`：不依赖 IDE 生成文件，按 `STM32Make.make` 列表并自动补齐 `Core/Src` 源码进行完整编译与链接；输出在 `build/refactor_verify/`，不烧录硬件。需 ARM GCC 11 或更新版本；仅启用上传时需要本地 MQTT 凭据头文件。

Python 通信测试依赖 `pyserial`，串口和连接参数见对应脚本。使用示例：

```powershell
python test_and_fix/communication/core_y100m_mqtt_test.py
python test_and_fix/communication/pc_y100m_gps_publish_test.py
```

编译验证需要先由 STM32CubeIDE 生成 `Debug/makefile`，并将 GNU Make 与 `arm-none-eabi-gcc` 加入 PATH。此入口依赖 Debug 工作目录及其生成规则：

```powershell
make -C Debug -f ../test_and_fix/build/4g_verify.mk Core/Src/usart.o Core/Src/modem_at.o Core/Src/app_log.o
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
