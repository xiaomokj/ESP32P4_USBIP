# ESP32P4_USBIP

把任意 USB 设备变成**无线 USB 设备**：ESP32-P4 作为 USB Host 接入设备，通过板载 ESP32-C6 的热点（直连）或局域网，用标准 **usbip** 协议把设备透传到远端电脑。电脑上执行一条 `usbip attach`，设备就以原生 USB 设备出现——Keil / pyOCD / CMSIS-DAP 等工具无需改动即可使用。

本仓库基于 Waveshare 官方示例 `11_esp_brookesia_phone` 做增量开发：手机壳（桌面 / 状态栏 / 最近任务 / 手势导航）与示例应用保留，新增 **2 个应用** 与 **1 个支撑组件** `mklink_core`。

- **USBIP**：通用版。任意 USB 设备插上即可启动服务透传。
- **MKLink USBIP**：针对 MKLink 探针（CMSIS-DAP）优化，带探针仪表盘。
- **`mklink_core`**：WiFi 双模、USB 主机枚举、usbip 服务端、日志/事件总线。

> 移植 Linux usbip 到 ESP32-P4 的全部踩坑与解决方案，见 **[USBIP_PORTING.md](USBIP_PORTING.md)**（协议线格式、TCP、接口占用、Windows 客户端、硬件与系统层，逐条给出现象/原因/解决）。

---

## 1. 硬件与前提

| 项目 | 说明 |
| --- | --- |
| 开发板 | Waveshare ESP32-P4-WIFI6-Touch-LCD-4B（ST7703 720×720 + GT911 触摸） |
| 主控 | ESP32-P4 **silicon v1.3**（`CONFIG_ESP32P4_REV_MIN_1`，IDF 5.5.5 默认是 v3.x，必须改） |
| 协处理器 | 板载 ESP32-C6，SDIO 4-bit：CLK=18 CMD=19 D0..D3=14..17 RESET=54，Slot1 @40 MHz |
| WiFi 框架 | `espressif/esp_hosted` **2.12.13** + `espressif/esp_wifi_remote` 0.16.3 |
| UI 框架 | `espressif/esp-brookesia` 0.4.2 + `lvgl/lvgl` **8.4.0**（Brookesia 要求 `<9`） |
| IDF | **v5.5.5**（本机 EIM：`/root/.espressif/v5.5.5/esp-idf`，激活脚本 `/root/.espressif/tools/activate_idf_v5.5.5.sh`） |
| PC 客户端 | Windows 端用 **`vadimgrn/usbip-win2`**（见 3.2） |

### USB-OTG 接线注意

- 板子只有一路 P4 USB PHY，当前固件用作 **Host**，所以烧录/日志走 **UART 桥（CH343，`/dev/ttyACM0`）**。
- BSP 注释明确本板**没有 VBUS 使能控制**。设备（尤其探针）不上电时需要 **USB-C 转 A 母的 OTG 线 + 外部供电**，或接**有源 HUB**。
- USB Host 控制传输上限已提到 1024 字节（`CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE`），usbip 转发完整配置描述符必需。

### C6 从机固件（必须与 host 侧 esp_hosted 同版本）

开机日志若出现：

```
W transport: Version mismatch: Host [2.12.0] > Co-proc [0.0.0] ==> Upgrade co-proc ...
```

说明 C6 的 hosted slave 固件不匹配。**不匹配时 WiFi 一旦真正承载流量，P4 会在 1~8 秒后打印 `H_SDIO_DRV: Unrecoverable host sdio state` 并重启**。

```bash
tools/c6_build.sh          # 编译 C6 固件 -> build/c6/slave/build/network_adapter.bin

# 让 C6 进入下载模式（本板没有自动下载电路）：
#   CH340 的 DTR 只接 C6 的 IO9 且反相：DTR 高 = IO9 低 = BOOT
#   最可靠：用杜邦线把 C6 的 IO9 短到 GND，然后复位/上电
tools/c6_flash.sh          # --before no_reset 写 bootloader/分区表/OTA/应用
```

烧写时三个坑：

1. **IO9 必须保持接地**（否则复位后 C6 正常启动，`esptool` 报 `Invalid head of packet (0x49)`）。
2. **烧写期间按住 P4 的 RST**（C6 停在 ROM 时 P4 的 esp_hosted 握手失败会反复重启，每次通过 GPIO54 复位 C6，打断 flash 会话）。
3. 烧完：松开 RST → **拔掉 IO9 短接线** → 再复位，C6 才跑新固件。

成功后日志变为 `Identified slave [esp32c6]`，且不再有 Version mismatch 告警。

---

## 2. 构建与烧录

```bash
tools/build.sh              # 编译
tools/build.sh flash        # 编译 + 烧录（默认 /dev/ttyACM0，PORT=... 可覆盖，烧录 2 Mbps）
tools/build.sh monitor      # 复位并进入串口监视（921600）
tools/build.sh fullflash    # 擦除整片后全量烧录
```

脚本内部两件必须做的事：

1. `source /root/.espressif/tools/activate_idf_v5.5.5.sh`；
2. `unset IDF_COMPONENT_LOCAL_STORAGE_URL`——EIM 会把组件管理器指向本地镜像，那里 `esp_hosted` 只有 2.12.13 / 3.0.7，不禁用会导致依赖求解失败。

手动等价命令：

```bash
source /root/.espressif/tools/activate_idf_v5.5.5.sh
unset IDF_COMPONENT_LOCAL_STORAGE_URL
idf.py set-target esp32p4
idf.py -p /dev/ttyACM0 -b 2000000 build flash
```

### 分区表

IDF 5.5.5 的 bootloader 大于 0x6000 字节，无法放在 `0x8000`，因此：

| 分区 | 偏移 | 大小 |
| --- | --- | --- |
| partition table | `0x9000` | 4 KB |
| nvs | `0xa000` | 24 KB |
| phy_init | `0x10000` | 4 KB |
| factory (app) | `0x20000` | 9 MB |
| storage (spiffs) | `0x920000` | 4 MB |

首次切换请用 `tools/build.sh fullflash`（分区表位置变了）。

---

## 3. 使用

### 3.1 设备侧

1. 把 USB 设备插到 ESP32-P4 的 **USB Host 口**（注意 OTG 供电）。
2. 打开 **USBIP**（通用）或 **MKLink USBIP** 应用。
3. 点 **启动服务**（或开机自启，见 3.3）。
4. 记下屏幕上的 IP（局域网 IP，或热点模式下的 `192.168.4.1`）。

两个应用共用同一个 usbip 服务实例，状态互通。

### 3.2 电脑侧（Windows）

用 **`vadimgrn/usbip-win2`**：https://github.com/vadimgrn/usbip-win2/releases

> 注意：**老的 `cezanne/usbip-win` 0.3.6-dev 不可用**。它对本工程这种「8 接口 / 13 端点 / 3 组 IAD 的复合设备」处理不了，`attach` 读完描述符就断开；usbip-win2（WHLK 签名的 UDE 驱动 + `usbip2_filter`）实测可用。
>
> 另外，usbip 客户端驱动是未签名/测试签名的，Windows 需**关闭 Secure Boot**（并可用 `bcdedit /set testsigning on`）才能加载。

安装后（默认在 `C:\Program Files\USBip\`）：

```powershell
cd "C:\Program Files\USBip"
.\usbip.exe list -r <ESP32-IP>            # 应列出你的设备（busid 固定 1-1）
.\usbip.exe attach -r <ESP32-IP> -b 1-1   # 挂接
.\usbip.exe detach -p 1                   # 用完断开（端口号看 attach 输出）
```

attach 成功后设备会出现在设备管理器，可正常使用（Keil 里 Debug 选 CMSIS-DAP 即调试）。

Linux 端则用内核自带客户端：

```bash
sudo modprobe vhci-hcd
sudo usbip list -r <ESP32-IP>
sudo usbip attach -r <ESP32-IP> -b 1-1
```

### 3.3 配置项

`idf.py menuconfig` → **MKLink Remote**：

| 配置 | 默认 | 说明 |
| --- | --- | --- |
| SoftAP SSID / 密码 / 信道 / 最大连接数 | MKLink-Remote / mklink1234 / 6 / 4 | 热点参数 |
| STA 开机自动重连 | y | 使用 NVS 保存的凭据 |
| usbip 端口 | 3240 | 标准 usbip 端口 |
| usbip 开机自启 | n | 打开后无需点面板 |
| USB 设备 VID / PID 过滤 | 0x0 / 0x0 | **0 = 匹配任意设备** |
| 探针 REPL 桥 | y | **usbip 模式建议关闭**（见下） |
| CDC 回环测试 | n | usbip 模式必须关闭 |

> **`MKLINK_PROBE_ENABLE=n` / `MKLINK_CDC_TEST_ENABLE=n` 的原因**：探针 REPL 读线程会持续轮询命令 CDC 的 bulk IN，该传输在主机控制器上不会超时，导致释放接口时端点清不干净，usbip 再去 claim 就报 `EP with X address already allocated`。usbip 直通场景应关掉两者（`sdkconfig.defaults` 已默认关闭）。

---

## 4. 代码结构

```
MKLINK_REMOTE/
├── CMakeLists.txt                 # project(esp32p4_usbip)
├── partitions.csv                 # 分区表偏移 0x9000 / nvs 0xa000
├── sdkconfig.defaults             # P4 v1.x、hosted SDIO 引脚、USB 1024、CJK 字体、串口 921600、usbip 开关
├── README.md
├── tools/
│   ├── build.sh                   # 构建/烧录封装（flash 2 Mbps / monitor 921600）
│   ├── gen_icon.py                # 生成 112×112 LVGL8 应用图标
│   ├── gen_font.py                # 生成中文字体子集
│   ├── c6_build.sh / c6_flash.sh / c6_download.py
│   └── _idf_env.sh
├── main/main.cpp                  # core 初始化 + 安装应用 + 启动服务
├── components/
│   ├── apps/
│   │   ├── apps.h
│   │   ├── CMakeLists.txt
│   │   ├── mklink_usbip/          # MKLink USBIP 应用（探针仪表盘 + 服务开关）
│   │   │   ├── MkLinkUsbipApp.{hpp,cpp}
│   │   │   ├── MkLinkUi.hpp       # 共享 LVGL 轻量工具（set_text_if_changed 等）
│   │   │   └── assets/            # 图标 + 中文字体
│   │   └── usbip/                 # USBIP 通用应用
│   │       ├── UsbipApp.{hpp,cpp}
│   │       └── assets/img_app_usbip.c
│   └── mklink_core/               # 支撑组件
│       ├── include/{mklink_core,mklink_wifi,mklink_usb,mklink_usb_raw,mklink_usbip}.h
│       └── src/{mklink_core,mklink_wifi,mklink_usb,mklink_usb_raw,mklink_cdc,mklink_net,mklink_usbip}.c
└── 其余（camera / video / music / calculator / draw / bsp_extra / esp_lvgl_port / spiffs）  # 原样保留
```

关键设计：

- **WiFi 单一所有者**：`mklink_core` 建立 `APSTA`，设置应用只做 UI 与监听，连接/断开/凭据全部走 `mklink_wifi_*`。
- **USB 不抢占接口**：枚举只缓存描述符与端点表；usbip 导入时才独占设备（停探针 REPL → claim 实际存在的接口），断开时释放。
- **UI 只读快照**：状态经 `MKLINK_EVENT` 与快照 getter，LVGL 由 200 ms 定时器在 LVGL 任务内刷新，不在其它任务碰 `lv_*`。
- **SDIO 传输缓冲用堆分配**（`CONFIG_ESP_HOSTED_USE_MEMPOOL=n`）：开着 mempool 时 WiFi 一旦真正收发数据，P4 会在 1~8 秒内报 `Unrecoverable host sdio state` 并重启；关掉后 40 MHz / 4-bit 连续跑 90 秒以上无异常。
- **中文用项目自带子集字体**：`mklink_usbip/assets/mklink_font_cjk_16.c`（Noto Sans CJK SC，16 px / 4 bpp）。改中文文案后用 `python3 tools/gen_font.py` 重新生成。
- **应用图标规范**：112×112 满版圆角底板 + glyph；`LV_COLOR_DEPTH=16` 时 `LV_IMG_CF_TRUE_COLOR_ALPHA` 是**逐像素交错** `[RGB565 低][RGB565 高][A]`（3 B/px），不是颜色平面 + alpha 平面——写成平面版能编译但上屏乱码。
- **usbip 线协议**：48 字节头、`op_common`、`usbip_usb_device` 312 字节、OP 编码 0x8005/0x8003，均按 Linux 内核源码逐字节对齐；服务端设置 `TCP_NODELAY`，数据面用 5 ms `select` 轮询以免回包延迟导致客户端超时。

---

## 5. 已验证 / 已知限制

已验证（实机，`/dev/ttyACM0`）：

- 编译烧录启动正常（P4 v1.3，PSRAM 32 MB @200 MHz，QIO 32 MB flash）。
- C6 握手成功（`Identified slave [esp32c6]`），APSTA 启动，STA 接入路由器并拿到 IP，掉线自动重连、凭据掉电保存。
- 40 MHz SDIO 连续运行 90 s 无异常。
- USB Host 枚举到探针 `0D28:0202 "MicroKeenV4 CMSIS-DAP"`（8 接口 / 13 端点 / 3 组 CDC）。
- usbip：`list` 正确列出设备与 8 个接口；`attach` 在 **usbip-win2** 上成功（`successfully attached to port 1`），设备在 Windows 中作为 CMSIS-DAP 出现。

已知限制：

- 单客户端、单设备（busid 固定 `1-1`，只导出当前枚举到的那一个设备）。
- Windows 端 usbip 客户端驱动未签名，需关闭 Secure Boot。
- bulk IN 超时较长（60 s），CMSIS-DAP 长耗时响应会超时重传，调试场景基本够用。
- 老 `cezanne/usbip-win` 0.3.6-dev 对本工程复合设备不可用，请用 usbip-win2。
