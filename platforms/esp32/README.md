> **语言 / Language:** 中文 · [English](README.en.md)

# Waveshare ESP32-S3-Touch-AMOLED-2.16 固件

第一次烧录请看[微雪版 DIY 完整教程](../../docs/WAVESHARE_DIY_GUIDE.md)，
已安装后的操作见[功能说明书](../../docs/USER_MANUAL.md)。

适配 **ESP32-S3-Touch-AMOLED-2.16**：CO5300 QSPI、CST9220、480×480、16 MB Flash、8 MB PSRAM。
使用 ESP-IDF **5.5.5**、Waveshare 2.16 BSP 和根目录 LVGL 固定子模块。

## 编译，不会刷板

先按 Espressif 的安装说明准备并激活 ESP-IDF 5.5.5 环境，然后在仓库根目录执行：

```sh
git submodule update --init --recursive
idf.py -C platforms/esp32 set-target esp32s3
idf.py -C platforms/esp32 build
```

首次构建会下载 Component Manager 依赖，产物在 `platforms/esp32/build/`。
`dependencies.lock` 中 LVGL 是项目根目录下的相对路径，若本地工具重写成绝对路径，
不要把个人路径提交回仓库。不要手动修改 `managed_components` 来维持编译。

## 备份和刷写

刷写会替换原厂应用。先确认准确板型、USB 串口、Flash 容量与加密/安全启动状态，
自行保存原厂全片备份；备份可能包含设备凭证，**不要上传 GitHub**。
以下 PORT 是占位符，请换成实际 USB 串口；命令需要已激活 IDF 的 Python 环境。

```sh
python -m esptool --chip esp32s3 --port PORT flash_id
python -m esptool --chip esp32s3 --port PORT get_security_info
mkdir -p backups
# 仅限已确认 16 MB、未启用 Flash 加密/安全启动的开发板：
python -m esptool --chip esp32s3 --port PORT read_flash 0 0x1000000 backups/waveshare-original.bin
```

上述语法对应 ESP-IDF 5.5.5 环境中的 esptool 4.x。若自行使用 esptool 5.x，子命令
改为 `flash-id` / `get-security-info` / `read-flash`，以该版本 `--help` 为准。
安全状态、容量或型号不符时停止；备份文件应为 16,777,216 字节，并另存校验值。
备份成功且接受覆盖原厂固件后才执行：

```sh
idf.py -C platforms/esp32 -p PORT flash monitor
```

不需要 `erase-flash`；串口日志用 Ctrl+] 退出。只使用本机这次编译生成的 flash 参数，
不要套用其他板卡镜像偏移。首次使用在 iPhone 中完成系统蓝牙配对。

## 行为与限制

- 广播名称 `MOTO GPS`；iPhone 通过 BLE 提供定位、路线、场景与音乐状态。
- 页面顺序：导航、马表、行程、航向、时钟、音乐。行程页数据由手机累加下发，长按清零。
- 时钟页优先用手机下发的时间；手机断开后由板载 PCF85063ATL（地址 0x51，共享 I2C）继续走时，
  开机即从芯片恢复，不再显示 `--:--`。手机时间与芯片相差超过 90 秒时回写校正。
  PCF85063 只存两位年份且协议不含年份，因此芯片单独走时期间年份取固件编译年份所在世纪；
  一收到手机时间即恢复为真实年份。
- 停车熄火安静 45 秒（导航中 180 秒）后熄灭面板并暂停渲染；车身振动、手机速度、触摸或 PWR 唤醒。
- 手机通知（ANCS）整屏显示来电/短信/应用通知。板子仍是 BLE 外设：ANCS 走手机自己建立的
  那条连接，板子在同一条连接上作为 GATT 客户端，不影响导航协议。iOS 只在已授权（配对）的
  连接上发布 ANCS，所以能否收到通知取决于 iPhone 的配对与通知权限；未发布时日志会记录
  `ANCS is not published on this link`，导航不受影响。
- 通知标题与正文使用随固件打包的字体：`shared/nav_ui/assets/moto_font_notify_16.c`
  与 `moto_font_notify_20.c` 覆盖 GB2312 一级汉字（约 99.7% 日常中文），由
  `scripts/generate_fonts.sh` 生成；生僻字会显示为占位方块。
- 开机黑白动画后进入连接页；连接成功后等待手机选路线，不把空路线画成 `0 m`。
- 左右滑动换页，指示点五秒后隐藏。音乐页是否可用取决于手机声明的能力。
- 按键：**BOOT 短按音量 −**、**GPIO18 短按音量 +**，四档 100/70/40/0 并存入 NVS；GPIO18 按住约三秒
  请求 AXP2101 软件关机（USB 供电下有深睡兜底）。PWR 仍只负责电源，不参与音量。
  BOOT 接 GPIO0，运行时可读，但**开机时请勿按住**：GPIO0 是下载模式选择脚。
- QMI8658 相对角速度辅助转向；行驶航向由手机定位锚定，不提供静止绝对北向。
- 使用 PSRAM 双缓冲；2.16 未把 CO5300 TE 信号引到 ESP32，因此不使用 1.75C 的 TE 同步。
- `sdkconfig.defaults` 打开了 `CONFIG_BT_NIMBLE_ROLE_CENTRAL` 与 `CONFIG_BT_NIMBLE_GATT_CLIENT`：
  板子不使用中央角色（不扫描、不发起连接），打开它只是因为 ESP-IDF 的 GATT 客户端支持依赖该选项，
  而 ANCS 需要 GATT 客户端。`CONFIG_BT_NIMBLE_MAX_CONNECTIONS` 仍为 1。

关键引脚：QSPI D0–D3 为 GPIO4–7；SCLK38、CS12、RESET39；I2C SDA15/SCL14；
触摸 INT11/RESET40；用户按键 GPIO18。不要把这些引脚当作可任意外接的空闲 GPIO。

上游：[微雪 2.16 示例工程](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-2.16)，
[Waveshare BSP 组件](https://components.espressif.com/components/waveshare/esp32_s3_touch_amoled_2_16)。
已知后台断连和路线问题见根目录 `docs/KNOWN_ISSUES.md`。构建成功不等于骑行验收通过。
