# qiyu-controller-tracking

ARM32 SDK/ABI 手柄输入与定位 helper。


构建入口和依赖见下方；工程 Wiki/Blog 与跨仓库规划集中在 [qiyu-docs](https://github.com/bitsk/qiyu-docs)。

## 构建

`python3 tools/build.py --ndk /path/to/android-ndk --stock-cpp /path/to/stock/ARM32/libc++_shared.so --output /new/build`

NDK、原厂 SDK、SNPE 与固件由使用者提供；仓库仅包含桥接代码和来源记录。相邻 qiyu-device-service 提供共享输入合同。输出仍是 Bionic ARM32 helper，通过 IPC 接入 ARM64/glibc 消费者。

已归档的原厂依赖可由 qiyu-rootfs 的 `tools/vendor.py fetch` 获取；构建时可改用 `python3 tools/build.py --ndk /path/to/android-ndk --vendor-cache /path/to/vendor-cache --output /new/build`，直接消费固定版本 controller 包里的 libc++_shared.so。SDK/Bionic 闭包另存于公开 qiyu-vendor Release，正常构建无需提取原厂镜像；每台设备的 persist 校准单独提供。
