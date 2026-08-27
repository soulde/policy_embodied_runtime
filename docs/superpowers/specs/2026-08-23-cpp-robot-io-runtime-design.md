# C++ Robot I/O Runtime Design

## 1. 目标与范围

本设计将现有 Python runtime 迁移为 C++20，并新增基于 IgH EtherCAT Master 的 Elmo Gold CiA 402 控制。首期支持 ARM64 与 x86_64 Linux、最多 12 个配置轴，以及 CSP、CSV、CST 三种模式。每轴模式由配置文件确定，进程运行期间不可切换；切换必须安全停止、修改配置并重启。

正式 1 kHz 部署要求 PREEMPT_RT。无实时补丁环境可用于开发与诊断，但不承诺最坏周期。首期不要求跨轴命令严格同步，也不包含多轴轨迹插补。现有模拟器保持独立，不迁移进 runtime。

## 2. 进程架构

系统分为两个 C++ 进程：

- `policy-runtime-host` 是非实时策略控制面，负责现有 ZMQ/JSON RPC、policy profile、策略推理和 pre/postprocess。
- `robot-io-daemon` 是设备执行面，负责所有设备侧 Transport、Protocol、Sensor/Actuator、调度与安全停止。

```text
Policy Client
    | ZMQ + JSON
policy-runtime-host
    | versioned Robot I/O IPC
robot-io-daemon
    | device transports
EtherCAT / Serial / CAN / virtual devices
```

ZMQ policy RPC 不属于设备 Transport，留在 host。用户进程崩溃或断开时，daemon 继续监视设备并执行安全停止。

## 3. 分层与模块

EtherCAT 属于 `transport`，CiA 402 属于 `protocol`，受控轴属于 robot device。建议目录为：

```text
src/
├── apps/
├── robot_io/              # daemon、IPC、scheduler、safety supervisor
├── robot/devices/cia402/  # axis、sensor、actuator、config
├── protocol/cia402/       # object dictionary、state machine、mode、units
├── transport/ethercat/    # IgH master、domain、slave、PDO、DC、SDO
├── transport/serial/
├── runtime/               # policy-runtime-host
├── policy/
├── profiles/
└── protocol/rpc/          # 现有 ZMQ/JSON 协议
tests/
```

`Cia402Axis` 由 `AxisSensor` 和 `AxisActuator` 共享。Device 产生或消费位置、速度、力矩等业务数据；CiA 402 Protocol 编解码控制字、状态字和对象字典字段；EtherCAT Transport 管理 PDO 过程映像、SDO mailbox 与实际收发。

## 4. Transport 抽象与调度

Transport 共享生命周期、状态和健康接口，但按通信模型提供不同能力：

- `FrameTransport`：Serial、CAN 等报文或字节流通道。
- `CyclicTransport`：EtherCAT 周期过程数据。
- `ObjectDictionaryTransport`：EtherCAT CoE SDO 等非周期对象访问。

Device/Protocol 调用 `write()` 只更新待发送数据，不能直接触发硬件 I/O。每个 Transport 的调度器在统一位置执行 `cycle()`。EtherCAT 使用独立的 `SCHED_FIFO` 线程；串口使用普通阻塞或事件线程；不同实时等级不得共享一个会相互阻塞的执行线程。

EtherCAT 每周期顺序为：

```text
ecrt_master_receive
→ ecrt_domain_process
→ 读取 TxPDO 并检查 WKC/slave/DC
→ CiA 402 状态机与安全检查
→ 将 staged command 写入 RxPDO
→ ecrt_domain_queue
→ ecrt_master_send
```

活动 PDO 内存仅由实时线程访问。Protocol 使用启动阶段绑定的类型化 PDO field，不持有任意偏移裸指针。SDO 只在启动、停止或非实时诊断路径使用。

## 5. 实时线程与数据所有权

轴数量由 robot profile 决定，首期校验不超过 12。所有轴、PDO offset、命令和反馈缓冲在启动阶段一次性分配；IgH 激活后拓扑冻结，实时循环内禁止容器扩容、动态分配、异常、JSON、日志、文件 I/O 和普通互斥锁。

host 与 daemon 使用完整快照交换命令和反馈。daemon 内部采用双槽快照：写者填充非活动槽后，以 release 顺序发布槽索引和序号；读者以 acquire 顺序读取并在序号稳定时接受快照。共享内存所用原子类型必须在启动时验证为 lock-free。周期线程使用绝对时间休眠、`mlockall()`、CPU affinity 和实时优先级。运行健康信息报告实际周期、最大抖动、deadline miss、WKC 和 DC 偏差。

## 6. Elmo PDO、模式与状态机

目标设备为 ESI 中的 Elmo Gold EtherCAT，Product Code `0x00030924`、Revision `0x00010420`，支持 DC Sync `AssignActivate=0x0300`。默认按静态模式选择最小 RxPDO：

- CSP：`0x1600`，目标位置与控制字；
- CSV：`0x1601`，目标速度与控制字；
- CST：`0x1602`，目标力矩与控制字。

TxPDO 从 ESI 的 `0x1A00`–`0x1A04` 中选择能提供所需实际位置、速度、力矩、状态字和模式反馈的组合。Elmo 设备描述提供默认 PDO assignment，robot profile 可显式覆盖；启动时验证必需对象及位宽。

启动时通过 SDO 写入静态参数和 `0x6060`，并由 `0x6061` 确认模式。运行期间不接受模式切换。每个轴独立解析 `0x6041` 并生成 `0x6040`，覆盖 Switch On Disabled、Ready to Switch On、Switched On、Operation Enabled、Quick Stop Active 和 Fault。用户只能请求 enable、disable、fault reset 和配置模式对应的 setpoint，不能直接写控制字。

## 7. IPC

Unix Domain Socket 提供控制面：ABI 协商、配置结果、设备发现、enable/disable、fault reset、诊断和共享内存建立。固定大小数据面使用 Linux `memfd_create()`，由 daemon 创建并通过 `SCM_RIGHTS` 将文件描述符传递给 host。

CommandBuffer 由 host 单写、daemon 单读；FeedbackBuffer 由 daemon 单写、host 单读。共享头包含 magic、ABI version、generation、axis count、stride、sequence 和 monotonic timestamp。每个方向使用两个快照槽与原子发布索引，不使用跨进程 mutex。daemon 重启改变 generation，旧映射不得继续使用。

## 8. 配置与兼容性

继续兼容现有 ZMQ/JSON envelope、robot profile 和 policy profile。新增 `cia402` 设备类型，配置 alias、总线位置、Vendor/Product/Revision、静态模式、单位换算、安全限制、命令超时和 safety group。daemon 加载 robot profile；host 加载 policy profile，并从 IPC 查询 robot data schema 以验证字段绑定。

生产运行不动态解析 ESI XML。ESI 用于开发期生成或审核设备描述，运行时使用版本化且经过测试的设备配置。

## 9. 安全与故障处理

- 命令 heartbeat 超时：Quick Stop，确认降速后 Disable。
- WKC 连续异常：总线相关轴进入安全路径。
- 单从站退出 OP：停止该轴；是否连带其他轴由 safety group 配置决定。
- `0x6061` 与配置不符：禁止 Operation Enabled。
- daemon 内限制位置、速度、力矩、变化率和 following error。
- Fault Reset 使用边沿触发并限制自动重试；持续故障要求人工确认。
- `SIGTERM`：停止接受命令，Quick Stop、Disable，然后释放主站。
- daemon 异常或总线失联：Elmo watchdog 和驱动器通信错误反应提供最后防线。

## 10. 工程与测试

工程采用 C++20、CMake、`nlohmann::json`、ZeroMQ/cppzmq、GoogleTest 和 IgH `libethercat`。实时路径使用显式 `Result/Error`，不抛异常。

测试包括 CiA 402 表驱动状态转换、Elmo PDO golden test、单位换算与限幅、Fake Transport 下的掉线/WKC/超时、IPC 撕裂与重启、Python/C++ JSON 兼容性，以及 IgH 硬件在环 CSP/CSV/CST 测试。正式验收在 PREEMPT_RT 上执行长期压力测试，记录周期和安全指标。

## 11. 迁移顺序

1. 建立 C++/CMake 骨架及 JSON/profile 兼容测试。
2. 实现 Transport、Protocol、Device 基础接口与 daemon IPC。
3. 实现 IgH EtherCAT、Elmo PDO、CiA 402 和安全监督。
4. 迁移 ZMQ runtime host、policy 与 pre/postprocess。
5. 将 Serial/ST3215 设备调度迁入 daemon。
6. 达到功能对等后移除 Python runtime；独立模拟器保留。
7. 后续单独实现多轴同步与协调轨迹。
