# Waveshare ESP32-S3-Touch-AMOLED-2.16 + iPhone setup

本工作区的固件目标是 **ESP32-S3-Touch-AMOLED-2.16**（480×480、16 MB Flash、8 MB PSRAM、CO5300、CST9220）。手机端使用项目内的 iPhone App：手机负责定位和高德路线，圆屏通过 BLE 接收导航状态。

## 编译固件

准备 ESP-IDF 5.5.5 和支持数据传输的 USB-C 线。在仓库根目录执行：

```sh
git submodule update --init --recursive
idf.py -C platforms/esp32 set-target esp32s3
idf.py -C platforms/esp32 build
```

首次构建会获取 LVGL 与 ESP-IDF 组件，固件产物位于 `platforms/esp32/build/`。确认开发板标签为 **ESP32-S3-Touch-AMOLED-2.16**；不要把 1.75C 的旧固件或 Flash 配置烧进来。

## 首次刷写前

刷写会替换原厂固件。先核对 Flash 容量和安全启动状态，并保存原厂备份：

```sh
mkdir -p backups
python -m esptool --chip esp32s3 --port PORT flash_id
python -m esptool --chip esp32s3 --port PORT get_security_info
python -m esptool --chip esp32s3 --port PORT read_flash 0 0x1000000 backups/waveshare-2.16-original.bin
```

将 `PORT` 换成实际串口。仅在确认容量为 16 MB 且未启用 Flash Encryption / Secure Boot 时读取完整备份；文件应为 16,777,216 字节。备份可能含设备凭证，不要上传。

确认备份可用且接受替换原厂应用后再烧录：

```sh
idf.py -C platforms/esp32 -p PORT flash monitor
```

## iPhone 与高德路线

按仓库主 README 配置路线网关和 iOS App。真实路线需要自己的高德 Web 服务 Key、HTTPS 网关和 Apple 开发者签名；设备与 iPhone 首次使用时需完成 BLE 配对。先在桌面上使用演示导航确认 BLE、屏幕和触摸，再在安全静止环境测试实地路线、锁屏恢复与重新连接。

## 适配范围

共享导航界面使用 480×480 画布；固件使用 Waveshare 2.16 BSP、CST9220 触摸和本板引脚。该板没有将 CO5300 TE 信号引至 ESP32，因此不启用 1.75C 固件中的 GPIO13 TE 同步。触摸方向、显示偏移和持续刷新仍需在实物上核验；构建成功本身不代表实机验收完成。
