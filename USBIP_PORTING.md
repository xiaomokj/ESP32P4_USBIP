# 把 Linux usbip 服务端移植到 ESP32-P4：踩坑全记录

本文记录本项目把标准 usbip 服务端（Linux 内核 `stub`/`usbipd` 那套）移植到 ESP32-P4（ESP-IDF 5.5.5 + `usb_host` DWC2 主机栈 + lwIP）过程中遇到的**全部坑**，每条都写明现象、原因和解决办法。

移植目标：ESP32-P4 作为 USB Host 接任意 USB 设备，用标准 usbip 协议（TCP 3240）把它导出给远端电脑，电脑上 `usbip attach` 后设备以原生 USB 设备出现。

> 结论先行：**协议本身不难，难的是三件事**——① 线格式必须逐字节对齐；② 本地已有消费者（探针 REPL / CDC 测试）会占死接口和端点；③ Windows 端客户端对复合设备很挑剔。

---

## 一、协议层：线格式必须逐字节对齐

### 坑 1：`usbip_header` 结构与字节序

**现象**：客户端连上后 `list` 出乱码或直接断开；`attach` 立刻失败。

**原因**：usbip 的 PDU 头固定 **48 字节**（`usbip_header_basic` 20 字节 + 最大成员 union 28 字节），且所有多字节字段是**网络字节序**。结构体没对齐或漏了某个字段的 `be32` 转换，客户端解析就会错位。

**解决**：

- 按 Linux 内核 `drivers/usb/usbip/usbip_common.h` 的定义逐字段写，并加静态断言：
  ```c
  _Static_assert(sizeof(usbip_header_t) == 48, "usbip_header must be 48 bytes");
  ```
- 收发时对**每个**多字节字段显式转换（`cmd_submit` 里 `transfer_flags`、`transfer_buffer_length`、`start_frame`、`number_of_packets`、`interval` 都要转；`setup[8]` 是原始字节，**不转**）。
- `OP_REQ/REP` 编码是 `0x8005`/`0x8003`（带 `USBIP_DIR_*` 位的高位标志），版本 `0x0111` = 1.1.1。

### 坑 2：`OP_REP_IMPORT` 回包不能带描述符 ⭐ 最坑

**现象**：`import` 看起来成功（服务端打印了 "imported"），客户端发出**第一个** URB 后几毫秒就断开；Windows 端报 `failed to attach`，而 `list`（devlist）却完全正常。

**原因**：按 usbipd 的 `recv_request_import()`，import 回包**只有**：

```
op_common (8 字节) + usbip_usb_device (312 字节) = 320 字节
```

**不含**设备描述符和配置描述符（这是老 usbip 0.x 的格式，内核 1.1.1 协议已经不再随 import 发送）。如果多发 18 + wTotalLength 字节，这些多余字节会留在 TCP 缓冲区里，被客户端当成**下一个 `RET_SUBMIT` 头**去解析 → 直接判为非法 PDU 并断开。

**解决**：import 回包严格只发 320 字节；描述符由客户端在数据面用 `GET_DESCRIPTOR` 现取（所以你会看到客户端的第一批 URB 就是 `GET_DESCRIPTOR config 9` → `GET_DESCRIPTOR device 18` → `GET_DESCRIPTOR config 9` → `GET_DESCRIPTOR config 253`）。

```c
static int send_import_reply(int s)
{
    send_op_common(s, OP_REP_IMPORT, ST_OK);   /* 8 B */
    pack_usb_device(&udev);
    send_all(s, &udev, sizeof(udev));          /* 312 B，到此为止！ */
    return 0;
}
```

### 坑 3：`usbip_usb_device` 的字段与 packed 布局

**原因**：`struct usbip_usb_device` 是 312 字节：`path[256] + busid[32] + busnum/devnum/speed(各 4) + idVendor/idProduct/bcdDevice(各 2) + 6 个 u8`。`pack_usb_device()` 只转换 6 个多字节字段（busnum/devnum/speed/idVendor/idProduct/bcdDevice），u8 字段不动。

**解决**：`devinfo` 的打包方式直接照抄内核；`busid` 用固定字符串（本项目写死 `"1-1"`，与 `path` 无关），PC 端 `-b 1-1` 就是靠它匹配。

### 坑 4：`RET_SUBMIT` 的 direction / ep / actual_length

**原因**：控制传输的 `base.direction` 由 **setup 包的 `bmRequestType` 的 bit7** 决定（不是 `cmd_submit.direction`），`base.ep` 恒为 0（控制端点），`actual_length` 只算**数据阶段**字节数（不含 8 字节 setup）。

**解决**：控制传输单独走一条分支，用它自己的 setup 解析方向；bulk/interrupt 才用 `ep_num | (IN ? 0x80 : 0)`。

---

## 二、传输层：TCP 的两个坑

### 坑 5：必须开 `TCP_NODELAY` ⭐

**现象**：`RET_SUBMIT` 明明发出去了，客户端却等很久（几百毫秒）才收到，甚至超时断开。日志里表现为「完成回调已触发」和「客户端断开」之间有一段不固定的空档（实测 60 ms ~ 313 ms 抖动）。

**原因**：`RET_SUBMIT` 是「48 字节头 + payload」两次 `send()`。Nagle 算法在有未确认数据时会延迟小包，而客户端又可能延迟 ACK，两者叠加就把回包拖到几百毫秒。

**解决**：内核 usbipd 里就有一行 `usbip_net_set_nodelay(sockfd)`。ESP32 端在 accept 之后对连接 socket 设：

```c
int one = 1;
setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
```

### 坑 6：数据面轮询间隔不能太长

**现象**：开了 NODELAY 之后仍偶发「客户端拿到配置描述符就断」。

**原因**：主机传输是**异步完成**的（回调在 USB 客户端任务里跑），而服务端主循环如果 `select(100ms)` 阻塞，完成回调触发后要等最多 100 ms 才回 `RET_SUBMIT`。客户端枚举阶段对控制传输的超时很短，等不及。

**解决**：把数据面 `select` 的超时降到 **5 ms**，让完成回包尽快发出：

```c
struct timeval tv = { .tv_sec = 0, .tv_usec = 5000 };
select(s + 1, &rfds, NULL, NULL, &tv);
```

> 更好的做法是完成回调直接 signal 一个 event/semaphore 唤醒主循环，本项目为简单起见先用了短轮询。

---

## 三、USB 主机资源：本地消费者把接口占死

### 坑 7：`EP with X address already allocated` ⭐

**现象**：
```
E USBH: EP with 5 address already allocated
E USB HOST: EP allocation error ESP_ERR_INVALID_STATE
E USB HOST: Claiming interface error: ESP_ERR_INVALID_STATE
```
然后所有 URB 失败，客户端 `failed to attach`。

**原因**：设备被本地程序先占了。本工程里有两个「消费者」：

1. **探针 REPL 桥**（`mklink_probe.c`）——它 claim 了命令 CDC（if4/if5），并且一直轮询 bulk IN；
2. **CDC 回环测试**（`mklink_cdc_test`）——它 claim 又 release if2/if3，批量 IN 超时中断的时序没清干净。

usbip 再去 `usb_host_interface_claim()` 时就撞车。

**解决**（三层）：

1. **import 前先停本地消费者并归还接口**：`mklink_probe_detach()` 要真正停读线程、abort 在途 IN、`release_interface`；
2. **只 claim 实际存在的接口**（用快照里的 `iface_cnt`），不要盲目 claim 0..31；
3. **配置上关掉它们**（usbip 模式）：
   ```
   CONFIG_MKLINK_PROBE_ENABLE=n
   CONFIG_MKLINK_CDC_TEST_ENABLE=n
   ```
   这样 if4/if5 从启动起就没人碰，usbip 一次性 claim 干净。

### 坑 8：bulk IN 在 DWC2 上「永不超时」

**现象**：即使 `mklink_usb_raw_abort()`（底层是 `usb_host_endpoint_flush()`）也清不干净端点，下一次 claim 仍报 `already allocated`。

**原因**：DWC2 对**没有设备数据**的 bulk IN 传输不会按 `timeout_ms` 超时（只有 internal 超时），探针读线程的轮询 IN 一直挂着；释放接口时端点里还有在途传输，状态不清。

**解决**：

- 释放接口**前**先 `abort` 该端点，并**等待读线程真正退出**（轮询 `s_reader == NULL`，最多等 2.5 s）；
- 更彻底的做法就是坑 7 的方案 3：**干脆不启动本地消费者**。

### 坑 9：控制传输上限默认 512 字节

**现象**：`GET_DESCRIPTOR config` 全量请求超过 512 字节时失败或被截断。

**原因**：`CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE` 默认值偏小，而复合设备的配置描述符可能上百字节。

**解决**：`sdkconfig.defaults` 里提到 1024：
```
CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=1024
```

### 坑 10：复合设备（IAD）的描述符必须完整转发

**现象**：设备是 `bDeviceClass=0xEF / 0x02 / 0x01`（IAD 复合设备），客户端拿到部分描述符后拒绝。

**解决**：不要自己拼描述符，`GET_DESCRIPTOR` 原样转发给设备、原样回给客户端即可。注意长度按设备实际返回的 `actual` 发，`wTotalLength` 只用于校验。

---

## 四、Windows 客户端：兼容性是另一个大坑

### 坑 11：`cezanne/usbip-win` 0.3.6-dev 带不动复合设备 ⭐

**现象**：服务端一切正常（`list` 正确列出 8 个接口），`attach` 时客户端读完描述符就断开；服务端日志能看到 4 个 `GET_DESCRIPTOR` 的 URB 全部成功返回。

**原因**：0.3.6-dev 是 2019 年的老项目，其 vhci 驱动对「8 接口 / 13 端点 / 3 组 IAD」这种复合设备支持不足。

**解决**：换 **`vadimgrn/usbip-win2`**（WHLK 签名的 UDE 驱动 + `usbip2_filter`），实测 `attach -r <IP> -b 1-1` 一次成功（`successfully attached to port 1`）。协议要求 ≥ 1.1.1，本项目正好是 `0x0111`。

### 坑 12：Secure Boot 挡住未签名驱动

**现象**：`attach` 报 `vhci driver is not loaded`；`bcdedit /set testsigning on` 报「该值受安全引导策略保护」。

**原因**：usbip 客户端驱动是未签名/测试签名的，Secure Boot 开启时无法加载。

**解决**：进 BIOS **关闭 Secure Boot**，再 `bcdedit /set testsigning on` + 重启。（usbip-win2 的 release 驱动是签名的，但老版仍需要；无论如何关掉 Secure Boot 最省事。）

### 坑 13：老版 `install` 误报 `already exist`

**现象**：`usbip.exe install` 一直报 `vhci(ude) driver already exist`，但 `pnputil /enum-drivers` 里根本没有 usbip 驱动，`attach` 又报 `not loaded`。

**原因**：机器上装了**向日葵（Oray）的 `orayusbvhci.inf`**（另一个 "USB VHCI" 驱动），老版 usbip-win 的检测被它干扰，误判已安装。

**解决**：绕开 `usbip.exe install`，手动装：
```powershell
pnputil /add-driver .\usbip_vhci_ude.inf /install
# 设备管理器 → 操作 → 添加过时硬件 → 从磁盘安装 → 选上面的 INF
```
或者直接换 usbip-win2 的安装包（它用完全不同的驱动名/设备 ID，不冲突）。

---

## 五、硬件与系统层

### 坑 14：板子没有 VBUS 使能，设备可能不上电

**现象**：插上设备后完全没有枚举日志，`usbip list` 是空的（`no exportable devices`）。

**原因**：Waveshare 这块板 BSP 注释明确 **没有 VBUS 使能控制**（`BSP_USB_HOST_POWER_MODE_USB_DEV "Not used"`），USB Host 口不给设备供电。

**解决**：用 **USB-C 转 A 母 OTG 线 + 外部供电**，或接**有源 HUB**。设备枚举成功后日志会出现 `USB: VID:PID "name"`。

### 坑 15：单路 USB PHY 只能二选一（Host / Device）

**现象**：把 USB 口当 Host 用之后，串口（CDC）没了。

**解决**：烧录和日志走 **UART 桥（CH343，`/dev/ttyACM0`）**，不要指望 USB-Serial-JTAG。

### 坑 16：ESP-Hosted 的 SDIO mempool 会让 WiFi 一跑流量就崩

**现象**：WiFi 连上、刚开始传数据，1~8 秒后 `H_SDIO_DRV: Unrecoverable host sdio state` 并重启。

**解决**：`CONFIG_ESP_HOSTED_USE_MEMPOOL=n`（用堆分配传输缓冲），40 MHz / 4-bit 连续跑 90 s 无异常。

### 坑 17：C6 从机固件必须与 host 侧 esp_hosted 同版本

**现象**：`W transport: Version mismatch: Host [2.12.0] > Co-proc [0.0.0]`，随后 WiFi 流量一上来就 SDIO 崩。

**解决**：用 `tools/c6_build.sh` 编 C6 固件、`tools/c6_flash.sh` 烧写。注意本板**没有自动下载电路**：要把 C6 的 IO9 短到 GND、烧写期间按住 P4 的 RST（详见 README）。

---

## 六、实现细节（容易忽略但会咬人）

### 坑 18：客户端处理不能跑在 accept 循环的任务里

**现象**：第一次 `list` 之后服务端就死了，第二次 `attach` 报 `failed to connect a remote host`（TCP 都连不上）。

**原因**：把 `client_task()` 直接在 `server_task()`（4 KB 栈）里调用，里面的传输槽位表 + `select` 的 `fd_set` 很容易栈溢出；而且客户端结束时 `vTaskDelete(NULL)` 会把服务端任务一起删掉。

**解决**：每个连接 `xTaskCreate(client_task, ..., 8192, ...)` 独立任务，退出时只删自己。

### 坑 19：`stop` 只改标志不会解除 `accept` 阻塞

**解决**：保存监听 socket 句柄，`stop` 时 `shutdown() + close()` 唤醒 `accept()`。

### 坑 20：`devlist`（`list`）不能碰 USB 状态

**现象**：`list` 之后探针桥状态被打乱，后续 `attach` 出错（`Get EP handle error` / `EP command error` 刷屏）。

**原因**：断开时的清理逻辑无差别 `unlink_all()` + `release 所有接口`，把没被 import 的客户端也当成了 usbip 占用者。

**解决**：加 `imported` 标志——**只有真正 import 过的连接**才在断开时 unlink/release；`devlist` 连接只回列表、绝不碰 USB。

### 坑 21：busid 与 devid

**解决**：`busid` 固定 `"1-1"`（PC 端 `-b 1-1`）；`devid = (busnum << 16) | devnum`，本项目 `0x00010001`，`RET_SUBMIT` 里回同样的值即可（Linux 客户端只用它路由，单设备时不会严格校验）。

### 坑 22：bulk IN 超时给足

**原因**：CMSIS-DAP 某些命令响应较慢。

**解决**：bulk IN 超时给到 60 s（`IN_TIMEOUT_MS`），控制传输 3 s，bulk OUT 5 s。长耗时响应会超时重传，调试场景够用。

---

## 附：一张排查对照表

| 现象 | 最可能的原因 | 先查什么 |
| --- | --- | --- |
| `list` 正常，`attach` 立刻断 | import 回包多了描述符 / 没开 NODELAY | 服务端 import 回包长度是否 320 |
| `EP with X already allocated` | 本地消费者占了接口/端点 | `PROBE_ENABLE` / `CDC_TEST` 是否关 |
| `list` 之后服务端没了 | 客户端跑在 accept 任务里 | 是否独立任务 + 8192 栈 |
| 客户端拿到 config 描述符就断 | NODELAY 缺失 / `select` 太长 | 5 ms 轮询 + TCP_NODELAY |
| `vhci driver is not loaded` | Secure Boot / 驱动没装 | BIOS Secure Boot、`pnputil` 列表 |
| `list` 空列表 | 设备没上电/没枚举 | 串口有没有 `USB: VID:PID` 日志 |
| WiFi 跑流量就重启 | SDIO mempool / C6 固件不匹配 | `USE_MEMPOOL=n`、C6 版本 |
