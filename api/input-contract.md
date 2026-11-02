# A081 原厂手柄定位输入契约

本轮补齐原厂 JNI → SDK 的像素、几何、帧调度和 type-7 惯性字段，供后续接真实 V4L2/Nordic 数据。**未完成设备相机映射、同步协议或定位实测**，没有向 SDK 送入任何假图像/模拟输入，也没有修改已打包 helper/root。`a081_controller_input_contract.hpp` 是字段整理参考，不会调用 SDK 或设备；原先 `a081_controller_image_abi.hpp` 的中性参数名，以本文恢复的语义为准。

## 图像参数：两组逻辑流，每组两个视图

实际 attach 名称为 `tracking` 和 `ctrl-tracking`；`initCamera` 的两条日志标签与 attach 参数相反，应以参数为准。对象 `QvrCameraEngine+0x0c/+0x14` 分别是 tracking helper/frame，`+0x08/+0x10` 是 ctrl-tracking helper/frame。这只确认逻辑流与视图顺序，**尚不确认它们对应哪些物理传感器或 XML 相机编号**。

公共 SDK `+0x1c` / ELF `0x3850d8` 的按值 C++ ABI 为：

```cpp
int push_images(
    void *self,
    std::vector<void*> tracking_pixels,
    std::vector<void*> tracking_release_tokens,
    unsigned view_width, unsigned view_height, unsigned stride_bytes,
    unsigned long long ctrl_sof_boottime_ns, int tracking_frame_id,
    int hand, unsigned char channels,
    std::function<void(int,std::vector<void*>,std::vector<void*>)> release_tracking,
    std::vector<void*> ctrl_pixels,
    std::vector<void*> ctrl_release_tokens, unsigned ctrl_frame_id,
    std::function<void(int,std::vector<void*>,std::vector<void*>)> release_ctrl);
```

`hand=0` 左、`hand=1` 右。JNI 调单手 wrapper `+0x48/+0x60`，由 wrapper 插入 hand；不能在 wrapper ABI 再加一次 hand。vector/function 仍是 Android ARM32 libc++ 按值对象，不能传宿主 glibc 对象或跨 IPC 指针。

| 参数 | 原厂实际来源 / 消费 |
| --- | --- |
| `tracking_pixels`, `ctrl_pixels` | 各两个 CPU 可读图像指针；SDK worker 会检查两组各两个指针非空 |
| `*_release_tokens` | 原厂是对应 `AHardwareBuffer*` 数组，供原厂 release callback 解锁；不是 IMU、曝光或标定元数据 |
| `view_width` | `ctrl_frame.width >> 1` |
| `view_height` | `ctrl_frame.height` |
| `stride_bytes` | JNI 实传 `view_width << 1`；SDK 构建 `cv::Mat` 时把它作为每行字节数 |
| `channels` | JNI 传 `1`；SDK 此分支创建 `CV_8UC1`，每像素 1 byte；另有 3 通道分支，本机 JNI 未走它 |
| 图像时间 | `ctrl_frame.start_of_exposure_ts`，纳秒；本机引用的 QVR header 定义为 kernel BOOTTIME |
| 两个 frame ID | 分别保留 tracking 和 ctrl-tracking 原 frame number，不能替换成同一个本地递增值 |
| 曝光时长 | QVR caller 传入 `setImgFrame`，但该函数没有继续交给 SDK 图像接口，也没有把 SOF 改成曝光中点 |

尺寸**不是固定 640×480**。`channels=1` 的消费证据为 SDK `0x2de122..0x2de190`：选择分支后设置 Mat flags `0x42ff0000`、rows=对象 `+4`、cols=`+0`、step[0]=`+0x0c`、step[1]=1。`0x2de6b2..0x2de70e` 消费 tracking 第二指针，`0x2de7e8..0x2de84a` 消费 ctrl 第一指针；入口已检查 ctrl 第二指针存在。这里证明的是 unsigned 8 位单通道内存消费，不是某个 Debian V4L2 节点已经提供了正确格式。

JNI `0xe270` 通过 QVR API 6 的 `FrameToHardwareBuffer` 取得 AHB：

- 一个 AHB：`AHardwareBuffer_lock(usage=2, fence=-1, rect=null)` 后，pixels 为 `{base, base + AHB_desc.width/2}`，tokens 为 `{AHB}`。第二指针按**字节横向偏移**，不是 `base + width*height`。
- 两个 AHB：分别 lock，pixels 为 `{base0,base1}`，tokens 为 `{AHB0,AHB1}`；第二次 lock 失败会先 unlock 第一个。
- 此代码没有用 `AHB_desc.stride`、QVR `frame.stride`、AHB 格式或 `qvrcamera_hwbuffer_t.offset` 修正几何；不能据此假定任意 V4L2 缓冲都满足原厂布局。

若真实每视图为 640×480 且按原厂单 AHB 方式提交，等价布局才是每行 `[view0 的640字节][view1 的640字节]`、总宽1280、行步长1280，两个指针差640字节。两组逻辑流都需要满足同一个 SDK 几何。独立 V4L2 平面可按真实 stride 接入该通用接口，或重排成上述原厂布局，但前者偏离原厂 JNI 的固定 `2*width` 规则，须验证；本轮没有实现或执行这两种适配。

## 配对、时序和释放

`getEyeCameraNextFrame` 先取 tracking 帧，使用 `GetFrameParamNum` 的 peer frame number 请求 ctrl-tracking 对应帧，并要求两路 SOF 差绝对值 **小于 5,000,000 ns**。tracking 自动曝光仍活动时会继续取帧。SDK 只收到 ctrl 的 SOF；单靠两个接近的 V4L2 dequeue 时间不足以证明复现了原厂配对。

`setImgFrame` 顺序：

1. 在 ctrl SOF 查询头显 pose，把位置 xyz 和四元数 **xyzw** 从 float 转 double，经 SDK `+0x0c` 推入；仍使用原 ctrl SOF。这里需要真实原厂头显 6DoF 与一致的坐标系。
2. 根据 **ctrl frame ID `% 3`** 选择：余数0→右手 `HAND=1`，余数1→左手 `HAND=0`，余数2→释放两组帧，不提交图像。
3. 余数0/1仅在相应 JNI 手柄状态等于5且 enable 标志非零时提交；否则立即释放两组帧。惯性入口允许状态5或6，不能把两个 gate 混用。
4. 余数0/1在图像提交/丢弃之后调用 `WriteTimeStampCmd(hand, uint64(double(ctrl_sof_ns)*0.001))`；即使手柄尚不满足图像 gate 仍发送。余数2不发送这个命令。

该分相与回传时间是已确认的静态调用顺序；它与实际 LED/无线时钟同步的完整关系尚未闭合，不能凭 `%3` 自创新的照明相位。

原厂 release callback（JNI `0xeb38` tracking、`0xec84` ctrl）先逐个 `AHardwareBuffer_unlock(token,null)`，再按传回 frame ID 调对应 QVR `ReleaseFrame`。像素 vector 只是指针容器，其析构不会释放像素。SDK 把数据/回调保存在共享图像对象并异步处理；函数返回**不表示可以重用图像**。

Debian 适配应在 Bionic helper 内创建指向已映射内存的两个 vector，以及能够释放真实 frame lease 的本进程 callback。原厂 callback 只能接收原厂 AHB，不能把 V4L2 fd/buffer index 强转成 AHB 交给它。自定义 lease token/callback 是否只经原样保存/回传，需要沿所有丢帧/销毁路径实测；本轮仅恢复实际 JNI callback，不宣称完整生命周期已验证。

## Nordic type-7 → SDK 惯性调用

输入仍使用已恢复的 **36 bytes SPI 接收内容 `A5 5A + p[34]`**；事件通知是 `EV_KEY/0xbe`，并非另一个机型的串口帧。以下偏移全部相对于34字节、**已完成 META 解码**的 `p`。后续已确认本机先用原始 `p[1]` XOR `p[0]` 和 `p[2..33]`，保留 `p[1]`，再校验/取字段；精确顺序与启动状态解释见 [startup-contract.md](startup-contract.md)。

| 内容 | 原厂读取 / 送入 SDK |
| --- | --- |
| 包类型、hand | `(p[0] >> 4) & 7 == 7`，`p[0] >> 7`，0左1右 |
| 校验 | `sum8(p[0..32]) == p[33]` |
| data[0..3] | signed LE16 `p+7,+9,+11,+13` 除10000.0，顺序 **w,x,y,z** |
| data[4..6] | IEEE754 LE float32 `p+15,+19,+23` 转 double，原值保留；不是位置 |
| wire clock | `LE32(p+3) | (uint64(LE16(p+29)) << 32)` |
| sideband[0] | parser 最后用 `p[2] >> 7` 写入 int16 |
| sideband[1..2] | `p[31]`, `p[32]` 分别零扩展成 int16；SDK 将其组合为低/高 tick 字节 |

固件时钟分支依据 hand record `+0x230a`。`ParsePatternIDPackageImpl` 将 Pattern ID 包版本数组写到 record `+0x2308`，`GetPatternIDPackageInfo` 的数组顺序是 `{p[24],p[23],p[22]}`，故比较的是第三项 `p[22]`。该数组交给 head version callback，不把它冒充已确认的手柄本体版本：

- `<25`：`time_ns = wire_clock_48 * 30500`。
- `>=25`：`time_ns = wire_clock_48 * 1000`。

此处**没有**再调用 `LR::Inference` 或 `ConvertIMUToSys`。原厂在时间同步已可用的条件下，把经上述缩放的包时间直接交给 callback；这不证明未同步的原始手柄时钟天然就是 Linux BOOTTIME。旧参考的 affine fit 是其他时钟同步环节的机制，不能不加分辨再对该 type-7 值做一次 affine 变换。

具体调用序列为 `GetImuPreIntPackageInfo → ParseImuPreIntPackageImpl → 左/右 tracking_sensor_data callback → SDK +0x44/+0x5c → 公共 +0x18(self,data[7],sideband[3],time_ns,hand)`。中间不存在预先差分、改轴、换四元数顺序或取代为3DoF输出。SDK 才把 wxyz 改为内部 Eigen 顺序并执行累计向量差分/逆旋转/时间间隔处理；不要在适配器中重复做一次。

原厂 parser 还有非零四元数、重复/倒退时间、接收时刻与输入时间约33ms差值、record `+0x13==1`、注册回调、`TimeCalibrator::Avaliable()` 等过滤/状态分支。JNI 左回调 `0xfbd0`、右回调 `0xfce0` 再检查手柄状态5或6。新 header 的字段打包函数**不复制这个状态机，也不证明时间已同步**。

## 相机时间回传命令边界

`WriteTimeStampCmd`（serial `0x9ba0`）构造以下12字节命令，再交给其发送函数；这不是36字节 SPI 接收结构，也未据此实现内核 TX：

```text
byte 0     A5
byte 1     0C
byte 2     11 左手 / 12 右手
byte 3..10 uint64 little-endian time_value
byte 11    sum8(byte 1..10)，不含 A5
```

JNI 已把 ctrl SOF 纳秒乘0.001并截断为微秒。serial 对版本第三字节 `<25` 再做 `uint64(double(microseconds)/30.5)`，`>=25` 保持微秒；同一版本条件解释了返回 type-7 的30500ns/1000ns比例。尚未复原完整配对/固件版本获取/同步回复状态机，不能仅不断发送上述命令就宣称同步完成。

## 后续可执行工作与停止点

当前可以基于声明写 Bionic 内的真实数据封装，先保持 `exec/load/create` 已打包探针不变。下一步设备窗口应取得：两逻辑流的物理传感器/校准映射、真实尺寸/stride/SOF来源/帧号与相位、Pattern ID 版本及同步响应、有效 type-7 包；把它们与已提取 `persist/qvr/device_calibration.xml` 核对。SDK 输入还必须接入同一时刻的真实 HMD pose。满足这些才可以用 release lease 提交像素并验证位姿，不以包解析、SDK返回0或单独姿态代替6DoF。

本轮未扩展到完整 QVR 相机服务、无线协议和生命周期恢复；这几项是明确未闭合的剩余条件，而非继续猜测 ABI 字段。

## 证据边界

SHA-256：SDK `7008b975aa23b92a2cc9a43278b626d44512fbce597e92858e18f13b4609fe6a`；JNI `3b05f44cbdf84415fa303472d29e0bf433fd8e83372823276ee8447420302461`；serial `cdedc3c7e18819c53fd70d841a2e9705271c996b4b15f851aa52fe88efbff4ae`。本文地址是 ELF VA，REA observed address 加 `0x10000`，调用指针另保留 Thumb bit。

新增证据位于 `build/xr-tracking-r1/controller/input-abi/`：

- `jni-map-submit.json`：REA `ev_22a7e65f143e8c0a66c0dc057e9c543a0cfc5f178f473a6291c0cc06bb68cbcf`，`0x1e270/0x21b20` 均 resolved/ok。
- `jni-map-disassembly.txt`、`jni-left-imu-disassembly.txt`、`jni-release-disassembly.txt`：LLVM 指令证据。release 的 REA 请求未识别函数，`jni-release-callbacks.json` 是失败记录，**不作为成功反编译证据**。
- `sdk-blob-image-worker{,-2}-disassembly.txt`、`sdk-blob-submit-disassembly.txt`：LLVM 入口/队列/图像消费证据。复用父目录 `sdk-interface-disassembly.txt` 的 `0x3850d8` 和 `0x385888` 图像封装，不重新冷导入 SDK。
- `serial-write-timestamp-disassembly.txt`、`serial-pattern-{version,fields}-disassembly.txt`：LLVM 时间命令及版本字段调用链。

复用 `build/xr-rea-analysis/controller/` 的成功 REA 记录：`jni-entrypoints.json` / `ev_94f6f5fbc1d8acb52a1d9c9d1a38e523a4f68e6296b2e25925a17eb5205d7e66`；`serial-packets-verified.json` / `ev_218e54e528f8293c4df429eaf2e05f4a5362ff3a5370798c41b23e8bd49e0508`；`serial-flow.json` / `ev_561b24c3d811ec47924a763e59d4fd54a774bc2ef28cb0a7f8585cb834853c1b`；`jni-callbacks.json` / `ev_d90cbca2883db2240ee46d2b4dc8739fcd747be8e58026ed8507e68b0f6eb85e` 中右回调成功。

QVR 结构字段命名辅助来源是本地 Snapdragon XR SDK 4.0.6 的 `3rdParty/qvr/inc/QVRCameraClient.h`；目标实际偏移/参数以 JNI 调用指令核对。验证只做新 header 的 NDK ARM32 严格语法编译和证据/既有包摘要检查，没有运行目标代码。
