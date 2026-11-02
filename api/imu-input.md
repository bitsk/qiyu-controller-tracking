# 原厂 SDK 的真实 Nordic IMU 输入入口

`controller_sdk_imu.c` 是独立的 Android ARM32/Bionic helper，读取现有 `../nordic-stream.cpp` 输出的真实 JSONL。它实际封装了工厂、模式设置、惯性输入和位姿查询；没有生成样本、SPI TX、配对操作或相机替身。此前已实机通过的 `controller_sdk_probe.c`、二进制和私有 root 保持不变。

r1完成严格编译和 ELF/依赖检查后，主任务已在真实 A081 Debian 中运行此入口：SDK身份、工厂和 `initial_mode(0)`成功；真实Nordic管道输入4703条传输，3个type-2、0个type-7，正常退出0。新二进制SHA-256为 `706f0130836bf68c47e5e3fdae46233606dff0f539e74cd3c6ba54543035d7f2`，日志见 `reports/device/xr-tracking-r1/controller-input-result.log`。该历史版本带外部同步断言门槛；此次0个type-7，**SDK惯性输入和getPose均未运行，也没有真实运动包或定位结果**。代码不把包解析、返回0、置信度或真实日志回放当作6DoF恢复。

## 构建和输入

```sh
bash xr-desktop/system/tracking/controller/build-imu-helper.sh
```

仅需控制输入时可在A081 Debian直接构建原生ARM64消费者，不加载原厂SDK：

```sh
bash xr-desktop/system/tracking/controller/build-controls.sh /独立目录/controls
```

主机离线交叉构建使用 `CC=aarch64-linux-gnu-gcc`；该构建设置 `A081_CONTROLLER_CONTROLS_ONLY=1`，从编译中排除SDK动态加载/惯性/pose代码，不链接 `libdl`，运行时拒绝 `--sdk`。产物 `a081-controller-controls` 默认只处理控制字段，也支持历史真实日志 `--replay`（有效掩码始终为0）。这不是以ARM64调用ARM32私有ABI。

产物为 `build/xr-tracking-r1/controller/imu-input/controller-sdk-imu`，同目录记录 `helper-elf.txt` 和 `helper.sha256`。NDK API28 ARM32，解释器 `/system/bin/linker`；自身只依赖 `libdl.so`、`libc.so`，使用原厂 SDK 闭包中的 C++ runtime。私有根与部署由主任务处理，本构建脚本不更新既有 root。

在真实 A081 的私有 Bionic 根中，以下是入口参数合同（这些不是本轮执行记录）：

```text
# 实时输入；SDK create + initial_mode(0)后，逐包验证原厂数值/接收时钟门槛再调用IMU。
controller-sdk-imu --sdk /system/lib/libControllerSDK_EXPORT.so --seconds 25

# 真实日志回放；不加载 SDK、不调用工厂、惯性输入或位姿接口。
controller-sdk-imu --replay --seconds 25

# 实时按钮/触摸/手指量/摇杆字段；不加载或调用SDK，不接收--sdk。
controller-sdk-imu --controls-only --seconds 25
```

stdin 接入原生 Nordic 采集进程的管道，或在 `--replay` 模式读真实采集文件。实时模式要求记录来自**同一次启动、同一内核时钟域**；不能把历史文件接入实时模式再修改时间。所有模式有1–120秒 `alarm`，默认20秒；输入 EOF 正常退出。外层仍应使用独立进程/namespace 超时。不要并发启动第二个无线采集者。

输入识别的是 `nordic-stream.cpp` 生成的固定字段顺序：`kind=nordic_rx`、`irq_ns`、`completion_ns`、`irq_sequence`、`rx`。`rx` 必须是72个十六进制字符。其后的派生字段不参与判断；helper 自行解码并重算校验。允许采集器的 `kind=summary` 行；未知前缀、超长行、溢出时间、非递增 completion 时间会退出73。它是该生产者的记录接口，不是任意格式 JSON 解析器。

## 从帧到真实 ABI 调用

`a081_controller_wire.h` 可在 Bionic C 代码中复用，分成线协议解码和 type-7 字段封装两个函数：

1. 严格 `A5 5A` 头；不修正 `A4 5B/A7 58/A6 59` 等异常头。
2. 对34字节 payload：保存 `key=p[1]`，`p[0]^=key`，`p[2..33]^=key`，保留 `p[1]`。
3. `sum8(p[0..32]) == p[33]`，校验包含保留的 `p[1]`。之后才取 `type=(p[0]>>4)&7`、`hand=p[0]>>7`（0左、1右）。
4. 每只手须先收到校验通过的 type-2 PatternID，学习 head version `{p[24],p[23],p[22]}`。这个版本属于原厂 head version callback，不能称为已确认的手柄本体固件版本。版本变化要求退出并重新建立同步；不默认使用当前观察到的32。
5. type-7 字段按下表原样封装。不归一化四元数，不差分/旋转累计向量，不交换坐标轴。

| 字段 | ABI 内容及单位 |
| --- | --- |
| `values[0..3]` | `s16le(p+7,+9,+11,+13) / 10000.0`，double wxyz，无量纲 |
| `values[4..6]` | `f32le(p+15,+19,+23)` 原值转 double；累计惯性量，**不是位置**；具体物理单位未闭合 |
| `sideband[0]` | `p[2]>>7` 零扩展为 int16，原状态位 |
| `sideband[1..2]` | `p[31]`、`p[32]` 分别零扩展为 int16；SDK 内部组合16位 tick |
| `wire_clock48` | `LE32(p+3) \| (LE16(p+29)<<32)` |
| `scaled_time_ns` | head version第三字节 `<25`：乘30500；`>=25`：乘1000。仅缩放不等于同步 |

通过下面的时钟门槛后，实际调用固定 Build ID 的 `vtable+0x18`（ELF `0x384df8`）：

```c
int result = imu(sdk_object, values, sideband, timestamp_ns, hand);
```

`timestamp_ns` 是真正的 `uint64_t` 参数，NDK ARM AAPCS 自动处理跳过r3、对齐栈上64位值，hand随后入栈。没有发现名为 `ControllerSDK_UpdateIMU` 的可直接使用 C 导出，不能按这个名字 `dlsym`。这里调用的是已验证私有虚表 ABI，并在运行时核对 SDK Build ID、工厂、虚表及三个目标方法入口，保留 Thumb bit。

工厂仍为 `_Z11getInstance4HANDi(2,2)`，随后调用 `vtable+4(0)`，与 JNI 的数字参数一致。该方法返回模式值；本例预期0。`+0x18` 的通常返回值也是0，但内部可能丢弃时间/状态不合格的样本，因此输出 `fusion_acceptance:"unknown"`。

## 时钟与当前仍未恢复的状态

主线 `kernel/drivers/misc/a081-nordic.c` 用 `ktime_get_ns()` 记录 IRQ 和 completion，即 **CLOCK_MONOTONIC**；原厂 JNI 时间回调使用 **CLOCK_BOOTTIME**。helper 不把两者当成同一个时钟。

实时模式在初始化后和每个 type-7 输入时夹读 `MONOTONIC → BOOTTIME → MONOTONIC`，报告中点的 `boottime_minus_monotonic_ns`。不确定度为夹读区间半宽向上取整，加两个时钟的 `clock_getres` 分辨率；这是主机时间域转换的不确定度，**不包含尚未建立的无线时钟同步误差**。收到的 completion 加该实测 offset 才得到接收 BOOTTIME。

此次实机anchor报告 `offset=156ns`、`uncertainty=8000573ns`。后者是上述保守公式的结果，包含操作系统报告的 `clock_getres`，**不能把它直接称为硬件计时精度、实际抖动或实测无线同步误差**。后续原生arm64交叉核对确认两个clock_getres均4ms，当前内核未启用HIGH_RES_TIMERS；详见文末实测证据；本入口100µs门槛不放宽。没有type-7，故本次也没有执行逐样本时钟门槛。三个状态包均为右手 `head_version=[0,0,32]`、`head_power_state=4`、`gzll_raw_state=3`；其余4700条被严格帧头过滤，不能视为手柄运动数据。

新REA逐指令证据纠正了原先的ready假设：此A081构建的 `TimeCalibrator::Avaliable()` 和 `LR::Available()` **恒返回1**；`record+0x13=1` 来自 `AddHandle` 的初始半字 `0x0101`，不是无线握手结果。type-7 callback时间直接是原48位值乘1000或30500，没有经过LR拟合/ConvertIMUToSys。证据集中于 [radio-clock-contract.json](radio-clock-contract.json)。因此新版helper删除 `--stock-clock-synchronized`，不再接受以一个人为开关代替协议证据。

实时调用要求：版本已知、所有数值有限、四元数非零、每手时间严格递增；实测anchor不确定度不超过100µs，offset相对初始变化不超过1ms，completion相对当前MONOTONIC小于33ms；实际completion转换后的BOOTTIME与当前SDK分发BOOTTIME分别减sample time，整个不确定区间均严格落在 `(0,33ms)`。增加当前分发时间门槛避免管道中已滞留的旧样本进入SDK。100µs、1ms和传输新鲜度属于适配器限制；原厂33ms窗口及直接传递缩放时间有独立证据。数值通过只允许真实IMU入口验收，不能称无线协议、相机/HMD或定位已经成立。

原厂还有回调注册以及JNI状态5/6等上层生命周期分支；此入口直接调用已确认的SDK边界，没有模拟完整JNI对象。sideband保留原值，没有相机/HMD替身。

## 输出与 validity

r1新增 `controller_controls` version1行，由 `a081_controller_controls.h` 更新原厂serial callback输入字段。所有模式都可输出该行，`--controls-only` 跳过工厂/IMU/getPose，仅消费真实包；`--replay` 的 `valid_mask` 和 `input_active` 始终为0/false。按键来自mode1的 `p[27]&0xf0`，不是旧误读的 `p[1]&15`。原始数值尚未经过JNI左右手映射，`openxr_actions_mapped` 固定false。

`valid_mask` 的bit0/1/2/3/4分别为buttons/touch/index finger/middle finger/joystick。mode1四项共用本次MONOTONIC接收时间，mode2摇杆单独计时；未知mode不能刷新旧值。每项必须已经实际出现、收到本手真实PatternID且原厂head power callback对应enable=1，并在当前MONOTONIC下未超过100ms。100ms是本适配器消费策略，不声称为原厂无线超时；不把GZLL raw=3解释为配对成功。收到关闭enable的状态包或重启状态转换清除旧控制值，不能把重新enable当作旧按钮仍按下。

JSONL输出有效位只描述发出时刻。消费者必须在自己的每次取样重新检查 `*_receipt_monotonic_ns` 与 `max_age_ns`，字段超时、EOF、进程终止均撤销对应按键/触摸/轴状态；不能缓存最后一次 `input_active=true` 无限使用。helper在持续传输中发 `expired` 更新，正常EOF显式输出两手零掩码/零值 `eof` 行。异常进程终止无法保证尾行，因此EOF和本地失效检查不可省。时钟只表示真实传输接收时间，不冒充同步后的无线采样时间。完整合同见 [controls-contract.json](controls-contract.json)。

每个通过调用前门槛的真实输入先写完整 `controller_imu_input` 行，再调用 SDK、输出 `controller_sdk_imu_result`。即使 SDK 调用中崩溃也不会留下半行 JSON。随后用当前 BOOTTIME 查询 `vtable+0x34`（ELF `0x3865b0`）：输入的 `uint64_t *time` 是查询时间，返回时另外记录它的值；返回类型为 **float confidence**，ARM softfp 通过r0传回。

`controller_sdk_pose` 保留 SDK 的 position、wxyz、四组 motion vector、confidence、查询/返回时间；不做 OpenXR 世界坐标变换。未写/非有限数组值输出 JSON null。四组运动量在 JNI 日志中称线速度/线加速度/角速度/角加速度，但未完成物理单位和坐标核实，因此字段仍用中性名称。

`orientation_valid`、`orientation_tracked`、`position_valid`、`position_tracked` **始终 false**，它们是当前适配器的验收策略，**不是原厂位域**。原因：原厂 JNI 的 `getPoseAndState` 用 confidence threshold 和状态7的缓存分支生成自身 state；SDK还会在内部函数返回非零时把 confidence 最小钳到0.1。没有完成视觉/HMD/坐标/同步/lifecycle验收，不能把 confidence>0 或返回0直接映射成 OpenXR tracked/valid。

EOF 摘要包括真实记录数、错误头/校验数、type-2、type-7、已返回的 SDK 调用次数。`no_type7_input:true` 表示本输入没有 type-7，`motion_verified`、`position_tracking` 保持 false。退出0表示本输入处理结束，退出124表示超时；退出64参数、65–69加载/身份/工厂/虚表、70信号设置、71模式返回、72系统时钟、73输入记录、74版本变化错误。

## 证据

沿用 [input-contract.md](input-contract.md) 和 [startup-contract.md](startup-contract.md) 的成功 REA 证据，本轮没有重复导入大库或启动 REA 服务。地址均为 ELF VA；REA observed 地址加 `0x10000`：

- serial META `0x121d2`、checksum/hand/type7读取 `0x1229c/0x1233e/0x123e0`：`build/xr-tracking-r1/controller/startup-contract/serial-decode-meta-disassembly.txt`、`checksum-disassembly.txt`；`build/xr-rea-analysis/controller/serial-packets-verified.json`，Evidence `ev_218e54e528f8293c4df429eaf2e05f4a5362ff3a5370798c41b23e8bd49e0508`。
- parser `0xf6ac`、PatternID版本和时间比例：`serial-flow.json` / `ev_561b24c3d811ec47924a763e59d4fd54a774bc2ef28cb0a7f8585cb834853c1b`；`input-abi/serial-pattern-{version,fields}-disassembly.txt`；各自 `.c` 是便于查阅的伪代码，不替代原始 JSON。
- JNI左/右IMU回调 `0xfbd0/0xfce0`：`input-abi/jni-left-imu-disassembly.txt`、`build/xr-rea-analysis/controller/jni-callbacks.json`（右回调成功），Evidence `ev_d90cbca2883db2240ee46d2b4dc8739fcd747be8e58026ed8507e68b0f6eb85e`。
- SDK `+4/+0x18/+0x34`：`build/xr-tracking-r1/controller/sdk-interface-disassembly.txt` 的 `0x38451c`、`0x384df8`、`0x3865b0`；`0x386660..0x386680` 证实 confidence钳位和r0返回；literal `0x3866a0` 为 binary32 0.1。
- JNI输出 `0x10fa0`：`build/xr-rea-analysis/controller/jni-entrypoints.json` / `ev_94f6f5fbc1d8acb52a1d9c9d1a38e523a4f68e6296b2e25925a17eb5205d7e66`；`jni-entrypoints.c` 中 `getPoseAndState` 的 confidence/state7分支。没有从 SDK getPose ABI恢复出单独的 OpenXR validity位域。

SDK固定 SHA-256/Build ID 继续由 `a081_controller_abi.h` 给出；原厂库未复制到源码目录。子任务离线验证只运行 NDK 编译器和 ELF工具，没有 x86主机执行、QEMU或假输入；上文实机验收由主任务在独占设备窗口完成，私有namespace将冻结的新helper绑定到既有probe路径，未修改原root/manifest。原厂SDK会向stdout输出自身文本，消费结果时应筛选本helper的 `kind` JSON记录，不能把整个stdout假定为纯JSONL。

实机完整原始目录为 `reports/device/xr-tracking-r1/pose-raw/runs/controller-input-DL7Vx4rS/`，由 `../run-controller-input.sh` 在独立namespace里运行10秒真实Nordic流完成。原root的SHA256SUMS在临时只读bind前后均通过。

后续已用原生arm64读取交叉核实两个clock_getres均4000000ns，时钟源arch_sys_counter，夹读span469–7187ns；当前基线CONFIG_HIGH_RES_TIMERS未启用、HZ250。约8ms来自保守公式中两个报告分辨率相加，不是8ms硬件时间戳误差。证据 `reports/device/xr-tracking-r1/controller-clock-resolution.log`，已推送新内核对话处理；没有放宽门槛，也未将此项归因为当前缺少type7。

本次Nordic采集另报告299次IRQ通知合并；本轮只验收原生入口/状态解析，没有证明连续运动无丢包或启动阶段无延迟，原始summary保留。

r1后续内核会话在新HRTIMER基线 `b96aea58cecf3ab89b2de7e9e91468b964b751605d40b15a3df6c82a11e34173` 原样运行旧helper：clock_getres均1ns，ARM32 anchor uncertainty=627ns，仍0个type-7。报告见 `reports/device/hrtimer-debian-r1/results.json`。新版逐包门槛已ARM32交叉编译，待新的独占XR硬件窗口验收，不能把旧helper的结果归给新版。
