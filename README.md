# 硬件制作

有 CDC 的加入，很多信号不再需要全部集中在同一块板子上，因此蔬菜盒子 v2 的硬件按功能重新拆分。当前设计同时承担两件事：一是完成胎温、CAN、485、4G 等数据采集与转发；二是保留 24V、5V、3.3V 的电源转换和分配能力，为后续把它扩展成小型电源转换平台留余量。

本版硬件工程主要由以下几块组成：

- `Base`：母板和连接分配板，负责 STM32F407 最小系统板、信号预处理板、电源板、4G 模块、调试接口和外部线束之间的互连。
- `MINSYS`：STM32F407ZET6 最小系统板，提供主控、时钟、复位、SWD 调试和基础测试 LED。
- `pre-process`：信号预处理板，集中处理胎温 I<sup>2</sup>C 多路复用、CAN 控制器、CAN 收发保护和 4 个 RS485 输出拉力传感器的数据读取。
- `power`：电源板，完成 24V 到 5V、5V 到 3.3V 的 DC-DC 转换，并把各路电源送回母板。
- `I2C-CONVERTER`：胎温传感器转接小板，用于把 MLX90640 模块接口转换成统一的 4Pin 线束接口。

## 硬件选型与设计

### 4G远程发送模块

目前的选型最终确定为银尔达公司的合宙 Core-Y100M-B 套餐，AT 组件，也就意味着上电后需要依赖 MCU 通过 AT 指令完成模块配置。

Core-Y100m 资料链接：
https://yinerda.yuque.com/yt1fh6/4gdtu/hpv7ne23txp3lu21

银尔达DTU/RTU产品技术售后处理：
https://yinerda.yuque.com/yt1fh6/4gdtu/iov6crmokc8ywu4n

这些资料已经涵盖该模块的硬件和软件调试说明。当前固件中 4G 模块使用 `USART1` 走标准 AT 指令、TCP 和 MQTT PUBLISH 路径，MCU 直接通过 `AT+CIPSEND` 发送 protobuf payload，不再使用额外的自定义外包头。硬件上 4G 模块通过母板接口接入，关键连接以嘉立创工程文件 `hardware/data_collection_inte_v2.eprj2` 为准：`USART1_TX/USART1_RX` 已按 MCU 与模块交叉收发关系修正，`RTS/CTS` 接线已修正但当前固件 `USART1` 配置为 `UART_HWCONTROL_NONE`，因此硬件流控暂不启用；`PF7` 用作 `RST_4G` 控制脚。

为了方便外部模块和调试线束取电，当前硬件工程还新增了一路 `VOUT_5V` 输出及对应地线引出。制作线束时应区分 `VIN_5V` 模块输入、系统内部 5V 电源网和对外 `VOUT_5V` 输出，不要把 4G 串口地和电源地断开。

### 胎温传感器选型

完全成熟的胎温传感器价格不菲（￥500+，最高￥2000），自制实属无奈之举。

目前的选型为 MLX90640-BAB 红外热成像仪，32 × 24 像素，最快刷新率 1Hz，此时的分辨率为 0.1℃，降低刷新率的话精度可以更高。

选型资料：[使用MLX90640自制红外热像仪(一)：MLX90640介绍与API库移植-CSDN博客](https://blog.csdn.net/qlexcel/article/details/119417088)

选购的是这个：[MLX90640ESF-BAA/BAB 红外测温点阵传感器模块 红外成像/IR热像仪-淘宝网](https://item.taobao.com/item.htm?from=cart&id=980006433807&mi_id=0000_2AftXHKfk5v_sHWc2JIWJMf568yjVcRuB39joTpY1w&skuId=5937090505499&spm=a1z0d.6639537%2F202410.item.d980006433807.52177484umm1i5&upStreamPrice=13400)

商家说明模块板上已经带 4.7k I<sup>2</sup>C 上拉电阻，因此传感器转接板不再额外放上拉。主板侧只需要保证总线电平、线序和多路复用通道选择正确。

### 最小系统板——STM32F407ZET6

最小系统板使用 STM32F407ZET6 作为主控，外设资源从两组 2 × 20 排针和一组 2 × 20 排针引出，再由母板分配给各个功能板。当前引脚分配里常用外设如下：

- USART1：4G 模块通信。
- USART2：预留串口或调试通信。
- USART3 / I<sup>2</sup>C2：该组管脚存在复用冲突，原理图中已标注不能同时使用。
- USART5：与 SPI3 复用相关，实际使用时需要按固件配置确认。
- SPI1 / SPI3：用于连接 MCP2518FD 等 SPI 外设。
- I<sup>2</sup>C1 / I<sup>2</sup>C2 / I<sup>2</sup>C3：用于胎温传感器、外设扩展或调试。
- CAN1 / CAN2：保留 MCU 原生 CAN 资源，和外部 MCP2518FD 扩展 CAN 共同使用。

供电方面，核心板只接入 3.3V，不再单独接 5V；3.3V 由底板 DC-DC 提供。每个 VDD 附近按原理图布置 100nF 去耦电容，VDDA 通过 BLM21PG121SN1D 磁珠与数字 3.3V 隔离，并配 1uF 与 100nF 滤波，降低模拟电源噪声。

时钟部分包含 8MHz 主晶振和 32.768kHz 低速晶振。BOOT0 通过 10k 下拉，默认从主闪存启动，不适合直接依赖串口下载；BOOT1 对应 PB2，高低电平都可以，建议悬空。复位电路由 10k 上拉、100nF 电容和按键组成，SWD 接口引出 3.3V、GND、SWDIO、SWCLK，方便下载和调试。板上还放置了红、蓝两个测试 LED，分别串 1k 电阻到 MCU 控制脚。

### 母板与板间连接——Base

Base 板承担整个系统的连接分配职责，本身尽量少做复杂信号处理，主要把核心板、预处理板、电源板和外部接口可靠地接在一起。

核心板的排针信号在 Base 上被重新命名和分组：胎温相关信号按 `DA_FL`、`CK_FL`、`DA_FR`、`CK_FR`、`DA_RL`、`CK_RL`、`DA_RR`、`CK_RR` 引出；CAN 信号按 `CANA`、`CANB`、`CANC`、`CAN1` 分类；SPI 信号分成 `SCK_A/MOSI_A/MISO_A/CS_A` 和 `SCK_B/MOSI_B/MISO_B/CS_B` 两组；RS485 拉力传感器总线与 4G 模块各自使用独立连接器。

外部供电入口使用 24V 和地输入，电源板输出的 5V、3.3V 和地再回到母板分配给各模块。当前嘉立创硬件工程 `hardware/data_collection_inte_v2.eprj2` 中保留了 `LV_IN+`、`LV_IN-`、`LV_`、`VOUT_5V`、`VOUT_GND` 等低压侧网络，其中 `VOUT_5V` 已作为一路对外 5V 输出使用，方便后续接入调试模块或外部小负载。

### CAN信号扩展——MCP2518FD

CAN 控制器扩展使用 MCP2518FDTE/SL。它本质上是 SPI 到 CAN FD 控制器的转换芯片，适合在 MCU 原生 CAN 数量不够时扩展更多可控 CAN 通道。当前设计放置两片 MCP2518FD，分别作为 CANA 和 CANB/CDC 扩展控制器；当前固件已打通 MCP2518FD-A 初始化，MCP2518FD-B 相关初始化和轮询任务仍保持注释/预留状态：

- 每片 MCP2518FD 使用 40MHz 晶振。
- 与 MCU 之间通过 SPI 通信，信号包括 `SCK`、`MOSI`、`MISO`、`CS` 和 `INT`。
- 芯片供电范围支持 2.7V 到 5.5V。当前原理图标注为了缓解 3.3V 供电压力，MCP2518FD 供电使用 5V，但 SPI 侧仍需注意与 MCU 3.3V 逻辑电平兼容。
- CANA 作为控制器相关 CAN，对应固件中的 `FDCAN_BUS_A`，使用 `SPI1 + CS_A`。
- CANB/CDC 作为 CDC 报文监视预留，对应固件中的 `FDCAN_BUS_B`，使用 `SPI3 + CS_B`，当前默认未启用。
- `MCP2518_CS_A/B` 为低有效片选，GPIO 初始化阶段必须保持空闲高电平。若读寄存器出现 `0xFFFFFFFF`，优先检查芯片供电、CS、MISO 是否悬空以及 SPI/片选映射，而不是先怀疑 CAN 总线。

MCP2518FD 后级接 SN65HVD233DR CAN 收发器。系统里共预留四路 CAN 收发：CANA、CANB、CANC 和 CAN1，其中 CANC 面向整车 CAN，CAN1 面向电箱 CAN。每路 CAN 收发器都带 100nF 去耦，RS 脚附近预留电阻用于斜率或模式设置，CANH/CANL 出口前加入共模电感 DLW43SH510XK2L、PESD1CAN ESD 防护和 47pF 到地电容，以提升线束环境下的抗干扰和静电防护能力。

终端电阻按实际总线位置决定是否焊接。原理图中针对 CANB 预留了 120Ω 终端，其余通道也需要在整车线束拓扑确认后再决定是否补焊，避免在同一条总线上形成过多终端。MCP2518FD-A 当前按 40MHz 晶振、经典 CAN 500kbps 初始化，`OSC=0x00000400` 表示 `OSCRDY` 已置位，不能再按旧位定义误判为晶振未就绪。

### 胎温传感器I<sup>2</sup>C 多路信号处理——TCA9548APWR

MLX90640 模块地址相同，不能直接并到同一条 I<sup>2</sup>C 总线上，因此主板使用 TCA9548APWR 做 1 路到 8 路的 I<sup>2</sup>C 多路复用。MCU 只访问 TCA9548A，再通过软件选择对应通道，分别读取每个轮位的 MLX90640。

当前原理图中 A0、A1、A2 全部接地，因此 TCA9548A 地址为 `0x70`。上游 `SDA`、`SCL` 由 4.7k 电阻上拉到 3.3V，芯片旁边放置 100nF 去耦电容。下游实际使用了四组通道，对应四个轮位：

- `SD0/SC0`：左前轮胎温。
- `SD1/SC1`：右前轮胎温。
- `SD2/SC2`：左后轮胎温。
- `SD3/SC3`：右后轮胎温。

每一路下游 I<sup>2</sup>C 都串了 22Ω 电阻。这里的 22Ω 主要用于较长线束和电磁环境下的边沿缓和与保护；如果后续实测发现波形过钝、上升沿变慢或通信裕量不足，可以把这组电阻改成 0Ω 进行对比。

### 胎温传感器转接板——I2C-CONVERTER

I2C 转接板本身不做信号处理，只负责接口转换。板上同时放置 XD-GX12-4P-M 航插和 2.54mm 1 × 4 母座，FL线序统一为：

```text
1: 5V
2: GND
3: SDA
4: SCK
```
之后的接线，请电气组同学完成，接不好赖他们（bushi）
这里原理图沿用了 `SCK` 命名，实际对应 I<sup>2</sup>C 的 `SCL`。制作线束时要按功能确认，不要把它和 SPI 的 `SCK` 混在一起。由于 MLX90640 模块板上已有 4.7k 上拉，转接板保持无源连接即可。

### 4个拉力传感器RS485读取

RS485 部分负责读取 4 个带 RS485 输出的拉力传感器。硬件上使用 MAX485ESA(UWM) 完成 MCU 串口电平与差分 A/B 总线之间的转换，通过 `UART_485_TX`、`UART_485_RX` 和 `DIR` 与 MCU 侧连接，传感器侧共用 `485_A`、`485_B` 两根差分线接入。

4 个拉力传感器挂在同一条 RS485 总线上，软件上需要给每个传感器配置不同地址，再由 MCU 轮询读取。`DIR` 用于控制 MAX485 收发方向：发送查询帧前切到发送态，发送完成后及时切回接收态，等待对应地址的传感器返回数据。调试时如果出现只有一个传感器能读到、多个传感器同时回复或数据帧粘连，优先检查传感器地址、波特率、校验位和方向切换时序。

总线侧预留 120Ω 终端电阻、上下拉偏置电阻和 SM712 防护器件，并配 100nF 电容做局部去耦。终端电阻只应按实际线束拓扑焊在 RS485 总线两端，不能因为有 4 个传感器就每个节点都焊 120Ω，否则总线负载会过重，通信稳定性反而下降。

**PS:我们和商家谈崩了，人不卖我们，但是回路保留，接下来还是有可能上RS485通信类型传感器的。**

### 供电平台设计

#### 24V转5V

24V 到 5V 使用 LM2596S-ADJ。原理图参考成熟模块电路实现，输入端放置 470uF 电解电容和 100nF 高频去耦，开关输出后接 47uH 电感，续流二极管使用 SS34-MS，输出端放置 220uF 电解电容和 100nF 去耦。（不要想着用24V-5V固定类型替代，会烧穿的，且会波及到5V-3.3V芯片，大家一起烧的那种）

反馈网络由 3.09k 和 1k 电阻构成，并并联 3.3nF 电容改善环路响应。24V 和 5V 轨分别放置指示 LED，便于上电后快速判断电源链路是否工作。

经过多方测试与调整，该路在维持 ±5% 输出电压的情况下，输出电流可达 2.58A，足够当前系统使用。但 LM2596S 在较高压差和较大电流下发热明显，PCB 制作和装配时要优先保证铜皮散热、过孔导热和壳体通风。

#### 5V转3.3V

5V 到 3.3V 使用 TPS62A02DRLR。原理图中输入端放置 4.7uF 和 100nF 电容，开关节点后接 1uH 电感，输出端使用多颗 22uF、10uF 和 100nF 电容并联，反馈网络由 453k 和 100k 电阻设定输出电压，PG 脚通过 499k 电阻接入。

[[tps62a02-5V转3.3V选型.pdf]]

该芯片方案标注最大负载电流 2A，但系统实测时按当前板载负载，在维持 ±5% 输出电压的情况下输出电流达到 400mA，已经足够给核心板、TCA9548A、CAN 收发器逻辑侧和传感器上拉电平供电。后续如果 3.3V 侧继续增加模块，优先重新核算 3.3V 电流预算和温升。

电源板输出通过母板连接器回灌到系统：5V 用于 4G 模块、部分扩展芯片和外设供电；3.3V 用于 MCU、I<sup>2</sup>C 上拉、CAN 收发器和低压逻辑；所有外设地最终回到 `VIN_GND`，调试时需要重点检查电源地与信号地的连续性。

# 代码的设计

## 胎温传感器
### MLX90640单体调试经验记录

本节记录的是单个 MLX90640 器件在STM32F103C8T6上的的调试实现，不包含 TCA9548A 多路复用。当前阶段的目标是先验证单个红外阵列传感器的 I<sup>2</sup>C 通信、官方 API 移植、温度矩阵计算和串口输出链路，确认单器件工作稳定后，再考虑扩展到多传感器方案。

#### 硬件与外设配置

MLX90640 通过 I<sup>2</sup>C 与 MCU 通信，模块商家已说明板上带有 4.7k 上拉电阻，因此单体调试时没有额外添加 I<sup>2</sup>C 上拉。串口调试输出使用 USART1，用于打印 32 × 24 的温度矩阵。为了减少大量文本输出时对主循环的阻塞，USART1_TX 开启 DMA，DMA 通道为 DMA1_Channel4，工作模式为 memory-to-peripheral、normal mode、memory increment enable。

CubeMX/CubeIDE 生成代码时需要注意 DMA 初始化顺序：`MX_DMA_Init()` 要在 `MX_USART1_UART_Init()` 之前调用。否则 USART 初始化时虽然配置了 DMA 句柄，但 DMA 时钟和中断还没有准备好，后续 `HAL_UART_Transmit_DMA()` 容易表现异常。

#### 官方库初始化流程

MLX90640 的初始化依赖官方 API 的 EEPROM 参数提取流程。程序上电后先读取 MLX90640 EEPROM 数据，再调用官方库解析参数。当前单体调试中，初始化成功后输出：

```text
MLX90640 initialized
```

如果初始化失败，例如返回 `MLX90640 init failed` 或官方库错误码，优先检查 I<sup>2</sup>C 地址、供电、SCL/SDA 接线、I<sup>2</sup>C 时序和上拉。该阶段不要引入多路复用器逻辑，避免把单器件通信问题和总线切换问题混在一起。

#### 完整温度帧计算

MLX90640 的 32 × 24 阵列一共 768 个像素，但官方库计算温度时不是一次 I<sup>2</sup>C 读取就填满全部像素。`MLX90640_GetFrameData()` 每次读取的是一个 subpage，`MLX90640_CalculateTo()` 只会根据当前 subpage 更新对应的一半棋盘格像素。

因此如果只读取并计算一次就直接输出，会出现隔一个像素为 `0.00` 的棋盘状结果。这不是 DMA 发送导致的数据损坏，而是温度数组中另一半 subpage 还没有被写入。当前实现采用连续读取并计算两个不同 subpage 的方式，使用 `subPageMask` 记录 subpage 0 和 subpage 1 是否都已经完成计算；只有两个 subpage 都写入 `tempMap[768]` 后，才执行坏点修正和矩阵输出。

当前主循环的核心顺序为：

```text
读取 subpage 0/1
计算当前 subpage 对应像素温度
继续读取直到两个 subpage 都完成
执行 broken/outlier bad pixel correction
按行输出 24 行温度矩阵
以行分组，计算四组平均数
从publish和整车CAN发送
```

#### 串口 DMA 输出策略

最初如果直接把每个温度值格式化后立即 DMA 发送，会出现报文交叠、缺字、顺序错乱等现象。原因是 DMA 发送是异步过程，发送函数返回时底层仍可能正在读取源缓冲区。如果短时间内复用同一个小 buffer，后一次格式化会覆盖前一次尚未发送完成的数据。

当前实现保留原始数据数组不变，仅新增一个 512 字节的行缓冲区。程序先把一整行 32 个温度值格式化到 `uartLineBuffer[512]`，再调用一次 `HAL_UART_Transmit_DMA()` 发送整行。`UART_SendString()` 内部通过 `uartTxDone` 等待上一笔 DMA 发送完成，并在 `HAL_UART_TxCpltCallback()` 中置位完成标志。这样既减少了 768 次小包发送的开销，也避免了 DMA 源缓冲被提前覆盖。

这个设计的重点是：DMA 只负责降低串口发送阻塞和减少碎片化输出，不能改变 MLX90640 的采样和计算逻辑。温度矩阵必须先由两个 subpage 完整计算得到，再进入发送阶段。

#### 当前刷新率与精度取舍

目前完整输出一帧温度矩阵大约 2 秒左右，与连续读取两个 subpage、计算 768 点温度、串口输出 24 行文本以及主循环延时有关。降低 MLX90640 刷新率通常可以让单帧噪声更小、画面更稳定，但不会直接消除发射率、环境反射、传感器热稳定和安装视场带来的绝对误差。

后续如果需要进一步提高显示稳定性，优先考虑对完整 32 × 24 温度帧做 2 到 4 帧滑动平均，而不是一开始就大幅降低采样频率。对于静态目标，可以降低刷新率并配合平均；对于胎温这类存在动态变化的目标，过低的刷新率会带来明显滞后。

### 基于 TCA9548APWR 的多路复用调试

本节记录四个 MLX90640 通过 TCA9548APWR 接入同一条 I<sup>2</sup>C 总线后的调试过程。四个 MLX90640 模块的器件地址均为 `0x33`，无法直接并联访问，因此使用 TCA9548A 作为上游多路复用器。TCA9548A 的 A0、A1、A2 均接地，地址固定为 `0x70`；当前使用通道 0 到通道 3，对应四个轮位传感器。

软件上通道选择不再传入通道掩码，而是传入通道号 `0` 到 `3`，由底层写入 `1 << channel` 到 TCA9548A。这样可以避免把 `0x01/0x02/0x04/0x08` 误认为通道编号，也方便后续按传感器序号维护通道映射。每次访问 MLX90640 前先选择对应 TCA 通道，再按官方 MLX90640 API 完成 EEPROM 参数读取、帧数据读取和温度计算。

调试初期曾出现 TCA9548A 全地址扫描均无响应、I<sup>2</sup>C 总线忙、初始化失败等现象。实际排查中发现，问题不一定来自地址设置本身，而可能来自下游模块供电、上拉网络和总线状态。四个 MLX90640 同时供电时，普通电脑 USB 口供电余量不足，容易造成下游器件状态异常；断电重上电后通信恢复，说明电源裕量和上电时序对该系统影响明显。最终硬件侧应保证 5V 供电能力足够，3.3V 上拉电平稳定，且 TCA9548A 上下游 SDA/SCL 没有被任一模块长期拉低。

软件恢复机制采用保守策略：上电后执行一次 I<sup>2</sup>C 总线恢复，之后仅在连续多次出现忙或超时后再恢复总线，避免正常运行中因为偶发 busy 判断而频繁复位总线。恢复动作包括临时释放 I<sup>2</sup>C 外设、用 GPIO 开漏方式给 SCL 输出若干脉冲、产生 STOP 条件，再重新初始化 I<sup>2</sup>C。这个策略可以处理上电阶段的总线卡死，同时减少运行时对正常采样的打断。

多路版本稳定运行后，主要瓶颈转移到 I<sup>2</sup>C 总线负载。四个 MLX90640 需要轮询读取 EEPROM、状态寄存器、像素 RAM 和辅助数据，若 I<sup>2</sup>C 仍使用 100kHz standard mode，总线占用会很高，表现为输出卡顿、帧间隔变长，甚至触发恢复逻辑。将 I<sup>2</sup>C 调整为 fast mode 后，通信明显顺滑，说明卡顿主要来自总线吞吐压力，而不是单个传感器失效。

在 fast mode 基础上，进一步引入 I2C1_RX DMA。CubeMX/CubeIDE 中 I2C1 RX 使用 `DMA1_Channel7`，normal mode、peripheral-to-memory、memory increment enable，并开启 `DMA1_Channel7_IRQn`。代码中 `MLX90640_I2CRead()` 保持对上层同步返回，但底层每个读取分块使用 `HAL_I2C_Mem_Read_DMA()` 启动传输，再通过 `HAL_I2C_MemRxCpltCallback()` 和 `HAL_I2C_ErrorCallback()` 等待完成或错误。这样不改变官方 MLX90640 API 的调用方式，也不改变温度帧计算逻辑，只降低大块 I<sup>2</sup>C 读操作对 CPU 的占用。

采样频率调试中验证了一个重要结论：不要直接修改官方 `MLX90640_API.c`，应优先通过官方提供的 `MLX90640_SetRefreshRate()` 接口设置刷新率。按照当前参考文档，`MLX90640_SetRefreshRate(0)` 对应约 1Hz 测量速率，参数 `0` 到 `7` 分别对应 0.5、1、2、4、8、16、32、64Hz。实际工程中将刷新率宏定义为 `MLX90640_REF_1HZ`，并调用 `MLX90640_SetRefreshRate(MLX90640_ADDR, MLX90640_REF_1HZ)`。若后续发现不同 API 版本的参数映射存在差异，应优先以手头官方例程和实测输出节奏为准，不在库文件中直接修补位操作。

当前可工作的组合为：TCA9548A 地址 `0x70`，四路 MLX90640 地址均为 `0x33`，I<sup>2</sup>C fast mode，I2C1_RX DMA 负责大块读数据，USART1_TX DMA 负责按行输出温度矩阵。该组合已经验证比纯轮询 I<sup>2</sup>C 更顺滑，且不会改变上层 MLX90640 官方 API 的使用边界。

## 2+2 结构：两路标准CAN+两路CANFD解析实现——MCP2518FD

当前代码采用的是典型的“2+2”结构：两路由 STM32F407 原生 bxCAN 外设承担，两路由外接的 MCP2518FD 经 SPI 承担。这里的“CANFD”更准确地说是“具备 CAN FD 控制器能力的外设接口”，但在当前固件实现中，业务层统一按标准帧处理，FD 报文能力只在抽象层和底层驱动中预留，没有进入上层业务解码流程。

当前四路总线在代码中的职责划分如下：

- `CAN1`：电池箱 CAN，对应电池包扩展帧、单体电压、温度、状态量等报文。
- `CAN2`：整车 `CANB`，对应整车状态、轮边电机状态、IMU、IVT/FS 能量计等报文，并承担当前整车 CAN 发送链路。
- `SPI1 + MCP2518FD-A`：控制器 `CANA`，当前初始化已通过；运行期 `SPICanControl` 任务仍默认注释，需要使用时再恢复轮询与解码。
- `SPI3 + MCP2518FD-B`：当前规划为 CDC（半主动电磁阻尼悬架）监视总线，初始化和 `SPICanCDC` 任务均默认注释，建议保持只监听、不主动发控制报文。

代码层面的分层非常明确，建议后续也保持这种边界：

- `can.c`：仅负责 STM32 原生 CAN1/CAN2 的初始化、滤波器、发送和中断收包。
- `mcp2518fd.c/.h`：仅负责 MCP2518FD 芯片级寄存器、RAM、FIFO、SPI 读写，不承载业务语义。
- `fdcan.c/.h`：历史命名保留为 `fdcan`，但在 F407 上它本质只是“SPI-CAN 抽象层”，负责把 `mcp2518fd` 的原始帧包装成统一的 `FDCAN_StdFrame_t`。
- `can_decode.c/.h`：真正的报文业务解码层，所有邮箱 ID、字节拼接、物理量换算、轮位映射、IVT/FS 判源等都应集中在这里。
- `telemetry_data.c/.h`：统一数据缓存层，所有任务最终都往这里写，`publish` 只从这里读。

这种设计的最大价值是：底层通信方式和上层业务解码解耦。后续如果更换 CAN 控制器、修改 SPI 引脚、增加一条新的 MCP2518FD 链路，理想情况下只应改 `mcp2518fd/fdcan/freertos` 的接线和轮询部分，不应改动 `publish`、`protobuf` 和业务变量。

在当前实现中，原生 CAN 和 MCP2518 的接入方式并不相同：

- 原生 CAN 走中断。`HAL_CAN_RxFifo0MsgPendingCallback()` 在中断里按 `hcan1/hcan2` 分发，只做“收帧 + 入队”，避免在中断上下文里做复杂解码；双 bxCAN 滤波中，CAN1 使用 `0~13` bank，CAN2 使用 `14+` bank。
- MCP2518 走轮询。`SPICanControl` 周期性调用 `FDCAN_Poll()`，读取 `SPI1/CANA` 的 FIFO 帧后直接完成控制器报文解码；`SPICanCDC` 作为 `SPI3/CDC` 的独立监听任务，负责只读监视，并将最近一帧写入 `g_CDCInfo`，不并入电池箱链路。当前这两个运行期任务默认未创建，但相关代码入口保留。

这意味着原生 CAN、`SPI1/CANA` 和 `SPI3/CDC` 在进入业务解码层之前，都应当先被归一化成统一的帧结构，再由各自任务决定是否直接解码或入队：

```text
CAN_Msg_Queue_t
├─ can_channel
├─ msg_id
├─ msg_data[8]
├─ dlc
└─ is_ext_id
```

因此，后续新增标准 CAN 报文时，优先考虑的不是“它来自 bxCAN 还是 MCP2518FD”，而是“它属于哪个业务网络，由哪个任务或 `can_channel` 进入哪个解码函数”。当前 `CANA` 已经明确归属于 `SPICanControl`，不再并入 `CanForViehcle`。

MCP2518FD 当前初始化流程位于 `mcp2518fd.c`，其关键步骤包括：

1. SPI 片选拉高并复位芯片；
2. 切入 configuration mode；
3. 等待晶振 ready；
4. 初始化 ECC 和 RAM；
5. 配置 nominal bit timing；
6. 建立 RX FIFO / TX FIFO；
7. 配置 filter 0 全通到 RX FIFO；
8. 切回 normal mode。

当前配置策略是“保守优先”：FIFO 大小固定、经典 CAN 优先、滤波器先全部放开，保证移植和联调阶段先能稳定收包，再逐步缩窄过滤条件。对于车队日常维护来说，这种策略虽然不够极致，但更利于快速定位问题。

需要特别说明的是：`fdcan.h` 文件头已经写明，这个模块名只是历史保留名，不代表当前 MCU 真有原生 FDCAN 外设。后续任何人维护本工程时，都应把 `fdcan.c` 理解为“外置 MCP2518FD 的统一抽象层”，而不是 STM32G4/H7 系列那种硬件 FDCAN 驱动。

在业务解码层，当前 `can_decode.c` 已经形成三条明确入口：

- `CAN_DecodeVehicleCanbMessage()`：整车 `CANB`
- `CAN_DecodeBatteryBoxMessage()`：电池箱 CAN
- `CAN_DecodeControllerCanaMessage()`：控制器 `CANA`

后续若继续完善 `CDC CAN` 这一路，推荐仍然遵守同样模式：先在 `can.h` 增加物理含义明确的邮箱宏，再在 `can_decode.c` 完善专属解码入口，不要把新的业务解码混写回 `can.c` 或 `fdcan.c`。

## 悬架传感器——基于RS485总线实现的数据读取

RS485 相关代码目前属于“硬件链路打通、软件框架预留”的状态。也就是说，任务、串口、使能脚、数据入口都已经安排好了，但具体传感器协议解析还没有像 CAN 那样完全落到业务层。

当前 RS485 侧的代码结构如下：

- `USART2`：作为 RS485 串口口径。
- `EN` 引脚：用于 485 芯片方向/使能控制，该引脚在 `TensionSensorTask` 启动时被初始化到默认接收状态。
- `Rs485_StartDma()` / `Rs485_DrainDmaBuffer()`：USART2 ReceiveToIdle DMA 接收和环形缓冲处理入口。
- `Rs485_Dispatch()`：预留的协议解析入口。
- `Peripheral_Rx_Frame_t`：通用外设接收帧封装，用于承接字节流或未来的完整帧。

当前 `TensionSensorTask` 默认未创建，但任务体已经是 USART2 RX DMA 版本：启动后先把 `EN` 拉到接收态，再开启 ReceiveToIdle DMA；收到 idle 事件后由 DMA 缓冲区抽取新增字节，封装进 `Peripheral_Rx_Frame_t` 并送到 `Rs485_Dispatch()`。这个设计的优点是入口已经稳定，后续只需要替换解析逻辑，不需要再改任务调度结构。

但也正因为当前解析入口仍按字节流/片段喂入，所以后续如果正式接入悬架拉力/位移类传感器，建议在 `Rs485_Dispatch()` 内部增加完整的状态机，而不要在任务外层直接拼帧。推荐结构如下：

```text
byte stream
 -> frame state machine
 -> address check / length check / crc check
 -> physical conversion
 -> write telemetry_data
 -> optional publish / debug
```

建议的状态机字段至少包括：

- 当前解析状态（等待帧头 / 收集中 / 校验中）
- 当前缓冲长度
- 目标传感器地址
- 预期数据长度
- 超时计数

如果后续 4 个 RS485 传感器共线工作，务必把“查询”和“解析”分开设计：

- 查询调度由一个上层轮询器完成，决定当前向哪个地址发请求；
- `Rs485_Dispatch()` 只负责解析返回帧，不负责决定轮询顺序。

这样做有两个好处：

1. 轮询周期、超时策略、重发策略可以独立调整；
2. 单体协议解析函数可以单独复用到离线测试工具中。

当前代码里还没有出现完整的 RS485 数据落地结构，因此后续建议沿用现有架构，新建一个明确的悬架数据缓存结构放入 `telemetry_data.h`，例如：

```text
Suspension_InfoDef
├─ sensor_valid_mask
├─ load_raw[4]
├─ load_converted[4]
├─ last_error[4]
└─ frame_counter[4]
```

然后遵循与 CAN 完全一致的模式：

1. `TensionSensor` 负责收字节流；
2. `Rs485_Dispatch()` 负责协议解码；
3. 解码结果写入 `telemetry_data`；
4. `publish.c` 决定是否对外发布。

不要让 `TensionSensor` 直接去拼 protobuf，也不要让 `publish.c` 去碰协议字节。协议层和对外发布层一旦混在一起，后续更换传感器型号时维护成本会非常高。

## 4G传输——基于protobuf的MQTT订阅

当前 MCU 侧的 4G 发送链路，应当理解为“AT 指令配置 + protobuf 序列化 + USART1 发送”，而不是“MCU 内部完整 MQTT 协议栈”。这点对后续维护非常重要。

也就是说，MCU 当前承担的职责只有三层：

1. 从 `telemetry_data` 中取出已经解码好的业务数据；
2. 通过 nanopb（`pb_encode.c/pb_common.c` 及生成的 `fsae_telemetry.pb.c/.h`）序列化为 protobuf；
3. 通过 `USART1` 直接送给 4G 模块，由模块固件负责 TCP/MQTT 交互。

`publish.c` 是这一层的绝对核心，建议把它视为“MCU 对外遥测协议网关”。当前它承担四类工作：

- 调度发布周期：`fast / medium / slow`
- 将内部缓存映射到 protobuf 结构体
- 调用 `pb_encode()` 完成编码
- 通过 `USART1`/DMA 发送 AT 数据和 MQTT payload

当前定义的 Topic 如下：

- `fast_telemetry`
- `bms_summary`
- `vehicle_state`
- `bms_detail`
- `thermal_summary`

其中 `Publish_Process()` 当前保留为空的调度入口；真正的 topic 入队由 CAN、MLX90640 等数据生产任务在更新缓存后主动调用 `Publish_QueueTopic()` 完成，编码和串口发送由 `Publish4GTask` 负责执行。这样做的好处是把“数据采集”和“发送”拆开，避免主调度逻辑因为一次编码或串口忙而失控。

当前完整链路如下：

```text
 decode result in telemetry_data
  -> producer task calls Publish_QueueTopic()
  -> PublishQueueItem stores PublishQueueItem_t
  -> Publish4GTask dequeues one topic
  -> Publish_BuildFrame() fills protobuf payload
  -> Y100M_MqttPublish()
  -> AT+CIPSEND over USART1
  -> 4G module
```

这里还需要强调一个已经落地的设计点：当前 `publish` 路径不再依赖额外的 485 使能脚控制，发送就是直接走 `USART1`。因此以后更换 4G 模块时，只要模块仍然接受 UART 数据，就优先保持 `publish.c` 不动，把 AT 配置、联网、MQTT 上线等控制逻辑留在模块初始化或外部网关侧。

当前 Route-A 上云路径发送的是原始 protobuf payload：`Publish_BuildFrame()` 只负责按 topic 编码 protobuf 字节，`Y100M_MqttPublish()` 再组 MQTT PUBLISH 包并通过 `AT+CIPSEND` 交给 Y100M。云端或 PC 订阅端应按 MQTT topic 区分 payload 类型，不再解析旧的 `MAGIC0/MAGIC1` 自定义 UART 外包头。

在 protobuf 映射策略上，当前工程遵循“业务缓存优先、发布视图后映射”的思路。例如：

- CAN 解码先写入 `g_CANB_LoopData`
- 电池箱解码先写入 `g_BatteryInfo`
- 热成像先写入 `g_MLX90640_Frame`
- `publish.c` 最后再把它们组装成 `TelemetryFrame / VehicleState / ThermalSummary`

这种架构的最大好处是：协议变化与业务逻辑分离。以后如果 MQTT topic 命名改了、protobuf 字段改了、上位机要求某些字段删掉或换单位，只需要主要改 `publish.c` 和生成后的 protobuf 文件，不需要把底层传感器采集链路全部重写。

当前 `fsae_telemetry.pb.h/.c` 是生成文件，不建议手改。推荐维护方式是：

1. 修改 `.proto` 源文件；
2. 重新生成 `.pb.h/.pb.c`；
3. 再修正 `publish.c` 中的字段映射。

后续若新增一个全新的 4G 发布内容，优先顺序建议如下：

1. 先判断是否属于现有 topic；
2. 若现有 topic 能承载，就只在 protobuf 结构中补字段；
3. 只有当数据节奏、订阅对象、体量差异都明显不同时，才新增 topic。

不要轻易为每一种新传感器都开一个独立 topic，否则后续云端订阅关系、带宽占用、版本兼容都会迅速失控。

## 基于FreeRTOS多线程数据采集架构

当前固件的 FreeRTOS 结构已经从“单点轮询式工程”演进成了“多任务 + 队列 + 统一缓存”的采集框架。这部分是整套系统后续继续扩展的基础。

当前任务划分如下：

- `defaultTask`：系统基线任务，负责调试串口队列输出和状态灯翻转；`Publish_Process()` 当前为空调度入口，实际 publish 请求由数据生产任务主动入队。
- `CanForViehcle`：处理 `CAN_Msg_Queue`，负责整车 `CANB` 解码入口，对应物理 `CAN2`。
- `CanForBMS`：处理 `CAN_Msg_Queue_2`，负责电池箱 CAN 解码入口，对应物理 `CAN1`。
- `MLX90640`：热成像采集、CAN 摘要生成和 `thermal_summary` publish 请求任务。
- `TensionSensor`：RS485 入口任务，当前默认未创建。
- `SPICanControl`：`SPI1 + MCP2518FD-A` 轮询与 `CANA` 解码任务，当前默认未创建，但 MCP2518FD-A 会在启动任务中初始化。
- `SPICanCDC`：`SPI3 + MCP2518FD-B` 轮询与 CDC 监视任务，当前默认未创建。
- `Publish4G`：publish topic 出队、protobuf 编码和 `Y100M_MqttPublish()` 发送任务。
- `VehicleCanTx`：整车 CAN 发送任务，最终调用 `CAN_Send_Msg(&hcan2, ...)`。
- `initTaskBoot`：启动初始化任务，依次启动原生 CAN、初始化 MLX90640、初始化 MCP2518FD-A；MCP2518FD-B 和 4G bootstrap 当前保持注释。

对应的队列设计如下：

- `App_Uart_Tx`：通用 UART 发送队列，当前主要保留框架。
- `CAN_Msg_Queue`：整车 `CANB` 业务帧队列，来源为物理 `CAN2`。
- `CAN_Msg_Queue_2`：电池箱业务帧队列，来源为物理 `CAN1`。
- `PublishQueueItem`：publish topic 队列，由 `Publish4G` 消费。
- `VehicleCan_Tx_Queue`：整车 CAN 发送队列，最终走物理 `CAN2`。
- `task01DebugQueue`：调试日志队列

这套设计的核心思想是：**采集、解码、缓存、发布四层分离**。

```text
ISR / poll
 -> queue
 -> parser / decoder task
 -> telemetry_data cache
 -> publish scheduler
 -> encode / transmit
```

这种结构相比“哪里收到数据就在哪里直接处理并发送”的老办法，有几个非常现实的优势：

1. 中断时间短，不容易因为大量运算拖垮总线接收；
2. 各传感器和总线的节奏彼此解耦；
3. 上层发布策略可独立演进；
4. 任何一路新功能都能找到明确的接入位置。

当前任务之间最重要的约定是：**只有解码层写缓存，只有发布层读缓存对外发包**。例如：

- `CanForViehcle/CanForBMS` 当前已经稳定写入 `g_CANB_LoopData` / `g_BatteryInfo`
- `SPICanControl` 对应 `CANA` 控制器报文解码，任务恢复后写入控制器相关缓存
- `SPICanCDC` 当前作为 CDC 监视入口存在，任务恢复后写入独立的 `g_CDCInfo` 通用缓存；后续若明确 CDC 报文语义，再在该缓存基础上补充协议专属字段，而不是复用电池箱或整车缓存
- `MLX90640` 最终写 `g_MLX90640_Frame`
- `publish.c` 只读取这些全局缓存，不反向影响采集过程

当前没有为这些全局缓存额外加锁，原因是本工程默认采用“单写多读、最新值覆盖”的实时遥测思路，而不是“每一帧都要绝对一致快照”的事务性思路。对于遥测系统来说，这样的取舍通常是合理的，但后续如果引入大块复合结构并要求一致性极高，可以再考虑快照拷贝或临界区保护。

调试链路也值得单独记录。当前系统区分了两种“发”：

- 调试输出：走 `task01DebugQueue -> huart3`
- 业务发布：走 `PublishQueueItem -> Publish4G -> huart1`

这样可以避免调试信息污染正式对外数据口，也避免发布链路阻塞时把调试一起拖死。

热成像任务 `MLX90640` 是现阶段多任务设计最完整的一个示例。它做了这些事：

1. 顺序轮询 4 个传感器；
2. 对每个就绪传感器执行完整采样；
3. 输出调试矩阵和摘要；
4. 生成 CANB 摘要报文；
5. 所有传感器一轮采集后，统一触发一次 `thermal_summary` publish。

这说明当前工程已经不再是“采一帧就立刻到处发”，而是开始出现“任务内部聚合后再发布”的设计风格。这种风格非常适合以后继续做复杂传感器融合。

对未来维护者来说，判断一个新功能该放在哪个任务里，可以按下面这个简单原则：

- 如果它是“原生 CAN 中断收包”，放到 `CanForViehcle/CanForBMS` 的既有链路里；
- 如果它是“SPI 外接控制器轮询收包并且业务上独立于原生 CAN”，优先放到 `SPICanControl/SPICanCDC`；
- 如果它是“I2C 主动轮询采集”，优先仿照 `MLX90640`；
- 如果它是“串口字节流协议”，优先仿照 `TensionSensor`；
- 如果它只是“已有缓存的另一种对外表达”，只改 `publish.c`，不要新开采集任务。

# 功能新增与代码维护提要
## 功能新增——增加任务独立运行，增加队列实现信息传输

当前工程的新增功能方式，推荐尽量遵守“任务独立、数据入队、缓存归一、发布分离”的思路，而不要回退到“一个函数里全部做完”的写法。只要继续坚持这一原则，后续无论是再加一条 CAN、再加一类传感器，还是更换 4G 模块，都不会把整个工程拖进不可维护状态。

推荐的新增流程如下：

1. 先明确数据来源属于哪一类外设：CAN、SPI-CAN、I2C、RS485、UART 直连等；
2. 再决定它应该进入哪个任务，或者是否需要新建一个任务；
3. 为该功能设计专属数据缓存结构，放入 `telemetry_data.h`；
4. 在采集/解码层写入缓存，不直接发布；
5. 在 `publish.c` 中决定是否对外发布、如何发布。

对于“是否要新建任务”，建议不要机械地追求一个模块一个任务，而是看它是否具备独立节奏和独立阻塞特征。以下情况通常值得新建任务：

- 需要主动轮询；
- 访问周期明显不同于已有任务；
- 采集耗时较长；
- 容易因为超时或重试阻塞别的链路。

以下情况则优先复用现有任务：

- 只是同一条 CAN 总线上的新增邮箱；
- 只是同一 topic 中新增几个字段；
- 只是已有缓存结构的补充量。

当前队列设计已经给了一个比较健康的参考：任何“外设入口”和“业务出口”之间都至少隔着一层队列。这样做的意义不只是代码好看，而是能降低阻塞传染。比如某一路解析突然变慢，最先积压的是它自己的队列，而不是整个系统所有采集入口一起卡住。

需要提醒的是，队列化设计也带来两个维护要求：

1. 队列元素必须足够小、足够稳定，不要把超大结构反复拷贝进队列；
2. 队列深度要按最坏突发情况估算，而不是按“平时差不多够用”估算。

当前工程已经做得比较好的一点是：CAN 队列里传的只是 `CAN_Msg_Queue_t`，publish 队列里传的只是 `PublishQueueItem_t`，真正的大对象仍然放在全局缓存区。这种做法建议保留，不要把完整 protobuf 结构体之类的大块对象直接塞进 FreeRTOS 队列。

如果以后需要新增一个周期性后台任务，推荐模板如下：

```text
StartTaskXX()
{
    init once
    for (;;)
    {
        read or poll
        parse
        update telemetry_data
        optionally queue publish topic
        osDelay(period)
    }
}
```

其中最重要的一条经验是：**先让任务稳定采集，再决定怎么发**。很多功能设计阶段最容易犯的错误，是一边写采集一边塞发布，结果最后既难调试，也难验证链路边界。

## 新增传感器建议
### 基于RS485总线进行新增

如果未来继续新增基于 RS485 的传感器，建议优先复用当前的串口和任务框架，而不是重新新建第二套串口协议工程。比较推荐的落地步骤如下：

1. 明确新传感器协议格式：地址、功能码、数据区、校验方式；
2. 判断它能否和现有 4 个传感器共总线；
3. 在 `Rs485_Dispatch()` 内增加独立解析分支；
4. 在 `telemetry_data.h` 中新增对应缓存结构；
5. 根据需求再决定是否进入 `publish.c`。

对于 RS485 新传感器，最容易出问题的地方不是代码，而是总线组织方式。接入前应先确认：

- 波特率是否一致；
- 地址是否唯一；
- 校验方式是否一致；
- 供电和地参考是否稳定；
- 终端电阻是否只有总线两端存在。

代码上建议始终坚持“一种协议一个解析器”的原则。即便都走 RS485，也不要把不同型号传感器的字节解析硬塞进同一个 if-else 巨兽里。可以采用：

```text
RS485 dispatcher
 -> protocol A parser
 -> protocol B parser
 -> protocol C parser
```

如果未来确实会混挂多种 RS485 设备，建议在 `telemetry_data.h` 中把悬架类、拉力类、位移类等缓存结构分开，不要用一个大而混杂的结构把所有 RS485 设备揉在一起。通信方式相同，不代表业务含义相同。

另外，若后续需要通过 RS485 主动发送配置命令或标定指令，也建议新增“RS485 发送函数 + 请求调度器”，而不要让解析函数反向直接控制发送。这样职责边界更清晰，也更容易做超时重发。

### 基于I2C总线进行新增

I2C 新增设备时，首先要判断它属于哪一类：

- 与 MLX90640 一样，需要通过 TCA9548A 走多路复用；
- 直接挂在主 I2C 总线上；
- 地址固定且可能与现有器件冲突；
- 需要 DMA 大块读取，还是只需小寄存器轮询。

如果是“单地址、低频、小数据量”的器件，优先不要复制 MLX90640 那一整套复杂框架。可以采用更轻量的设计：

```text
TaskXX poll
 -> HAL_I2C_Mem_Read / Write
 -> decode register values
 -> write telemetry_data
```

如果是“同地址多器件并联”，那就应优先复用 `TCA9548A_SelectChannel()` 这一套通道选择逻辑，而不是重新写一套新的 I2C 复用方案。此时建议新增一层设备驱动文件，例如：

- `new_sensor_i2c.c/.h`
- `new_sensor_app.c/.h`

其中：

- `*_i2c.c` 只做寄存器访问；
- `*_app.c` 只做初始化、采样、物理量处理；
- 任务层只负责调度。

这样做的好处是，底层驱动将来可以单独移植到别的 MCU 或离线测试工程，而不会和 FreeRTOS 调度耦合在一起。

对于新增 I2C 设备，最容易忽视的维护点是“是否真的需要 DMA”。当前 MLX90640 使用 DMA，是因为它读 EEPROM 和 Frame RAM 的数据块大、访问频繁、总线负担重。普通小型 I2C 传感器如果也强行上 DMA，复杂度会上去，但收益可能很小。建议只有在以下场景再考虑 DMA：

- 数据块明显大于普通寄存器读写；
- 采样频率较高；
- CPU 已明显被 I2C 轮询占用。

如果以后新增的 I2C 传感器也需要在 `publish` 中上报，同样建议坚持当前模式：先写 `telemetry_data`，再由 `publish.c` 做统一 topic 映射，而不是让传感器驱动直接接触 protobuf。

## DBC文件的覆写与4G相关文件的更新覆写

这部分是后续最容易“改完能跑，但别人完全不知道改了什么”的区域，因此建议形成固定维护顺序。

如果发生 CAN 协议变更，通常至少会牵涉四层内容：

1. 协议来源文件：DBC、赛会协议文档、手写 `md` 说明；
2. MCU 侧邮箱定义：`can.h`；
3. MCU 侧解码逻辑：`can_decode.c`；
4. 对外发布映射：`publish.c` 与 protobuf 文件。

比较推荐的改动顺序是：

```text
先更新协议文档 / DBC
 -> 再更新 can.h 中的 ID 宏和名字
 -> 再更新 can_decode.c 中的字节解析
 -> 再更新 telemetry_data 里的缓存字段
 -> 最后更新 publish.c / protobuf
```

不要反过来先改 `publish.c`，否则经常会出现“想发一个字段，但底层还没真正解出来”的情况。

邮箱命名方面，当前工程已经从参考工程里那种 `APP_DEBUG2`、`APP_DEBUG5` 一类偏历史残留的名字，逐步改成了更有物理意义的命名方式，例如“四轮实际扭矩”“电机温度”“热成像摘要 FL/FR/RL/RR”。后续继续修改 DBC 时，建议坚持这一原则：**邮箱宏名优先反映物理意义，而不是历史调试名字**。

对于 4G 相关文件，当前工程分成两类：

- 协议定义文件：`fsae_telemetry.pb.h/.c` 及其源 `.proto`
- MCU 发送逻辑文件：`publish.c/.h`

如果新增字段但不新增 topic，通常要做的是：

1. 修改 `.proto`
2. 重新生成 `.pb.h/.pb.c`
3. 在 `publish.c` 中填新字段
4. 与上位机或云端订阅端同步更新解析逻辑

如果新增 topic，则除了上面几步外，还要额外更新：

- `PublishTopic_t`
- `Publish_Process()` 的调度策略
- `Publish_GetTopicName()`
- `Publish_EncodeTopicPayload()`

并同步确认云端是否需要新增订阅或转发规则。

建议以后每次改动 DBC 或 protobuf 时，都在本文档或配套变更记录中至少补三条信息：

- 修改日期
- 修改人
- 影响范围（MCU / 4G / 上位机 / 云端）

这样做看起来繁琐，但会极大减轻后续联调时“到底是谁先改了协议”的扯皮成本。

另外，如果今后你准备把“协议变更通知”做成真正的自动化流程，技术上最适合挂钩的位置其实不是 MCU，而是版本管理和文档仓库。也就是说，真正该自动推送的，不是固件运行时发现协议变了，而是 DBC / `.proto` / 对接文档被提交修改时，就由仓库侧自动通知相关成员。

#对未来的希望
在更新对接文件的时候能不能说一声，我什么都不知道，就发现通信协议天翻地覆了，我需要一个能自动推送的功能，不然真的会一直蒙在鼓里
