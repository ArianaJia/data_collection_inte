# Python 测试与修复工具

本目录集中保存工程自有的 Python 脚本。命令默认从工程根目录执行。

- `core_y100m_mqtt_test.py`：Core-Y100M 串口、AT 和 MQTT 基础连通性测试。
- `pc_y100m_gps_publish_test.py`：Y100M + TelemetryFrame GPS 发布链路测试。
- `update_can_protocol.py`：CAN 协议宏、数据结构与过滤器的迁移辅助脚本。
- `update_can_decode.py`：CAN 解码与缓存迁移辅助脚本。
- `finish_can_protocol.py`：CAN 数据新鲜度与发布映射收尾辅助脚本。

前两个脚本用于实际通信测试，需要 `pyserial`。后三个脚本会修改工程源文件，仅用于本次协议迁移复现，运行前应先检查 Git 工作区。
