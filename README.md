# qiyu-controller-tracking

ARM32 SDK/ABI 手柄输入与定位 helper。

实际功能源码首次按仓库所有权整理，来源和导入前后 SHA-256 见 `provenance/source-import.json`。原有历史构建与设备结果不自动等同于拆分后的设备验收。

构建入口和依赖见下方；工程 Wiki/Blog 与跨仓库规划集中在 [qiyu-docs](https://github.com/bitsk/qiyu-docs)。

## 构建

`python3 tools/build.py --ndk /path/to/android-ndk --stock-cpp /path/to/stock/ARM32/libc++_shared.so --output /new/build`

NDK、原厂 SDK、SNPE 与固件由使用者提供；仓库仅包含桥接代码和来源记录。相邻 qiyu-device-service 提供共享输入合同。输出仍是 Bionic ARM32 helper，通过 IPC 接入 ARM64/glibc 消费者。
