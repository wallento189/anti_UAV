# autoaim_serial 串口协议 v1

视觉侧（`autoaim_serial`）与下位机（云台控制板）之间的二进制串口协议。控制律完全在下位机，本协议只传"目标角误差"和"云台姿态"。

上位机实现：`src/autoaim_serial/include/autoaim_serial/protocol.hpp`、`src/autoaim_serial/src/protocol.cpp`。
两端修改任何字段都必须同步本文档与 C 参考实现。

## 1. 链路与语义

```
autoaim_tracker/target ──► autoaim_serial ──► autoaim_serial/send
                                  │
                                  └──► UART 命令帧 ──► 下位机（控制律）
UART 反馈帧 ──► autoaim_serial ──► autoaim_serial/recv
```

- 命令帧在每条 `TrackedTarget` 到达时发送（约 90Hz），不做定频重发。
- `TrackedTarget` 的 `status` 为 TRACKING 之外的取值时，所有角度都是 0；下位机必须同时看 `status` 与数值，不能只看数值。
- 视觉链路正常时命令帧连续；若视觉侧（相机/检测/跟踪/串口节点）整体挂掉，链路会静默。**静默与 `status=0` 含义不同**：前者是通信故障，应进入更保守的保护；建议下位机超时阈值 200~300ms。
- 视觉侧不发送任何"心跳"或伪造的 LOST 帧。
- 反馈帧由下位机按自己的节奏发送，视觉侧只做接收、限流告警和转发。

角度符号全链路统一，与 `TrackedTarget` 一致：

| 量 | 正方向 | 单位 |
| --- | --- | --- |
| `yaw`, `yaw_error`, `yaw_rate` | 向左 | rad, rad/s |
| `pitch`, `pitch_error`, `pitch_rate` | 向下 | rad, rad/s |

反馈角的零位由下位机定义（云台零位），视觉侧不做二次变换。

## 2. 通用编码规则

- 所有多字节字段 **小端（little-endian）**。
- `float32` 为 IEEE-754 binary32；发送端必须保证不是 NaN/Inf（视觉侧编码前会清洗，下位机也应做同样检查）。
- **禁止把结构体直接 `memcpy` 到串口**：逐字段、逐字节写，避免对齐和大小端问题。
- 帧头 `AA 55` 之后依次是 `msg_id`、`seq`、`len`、`payload`、`crc16`。
- `len` 只统计 payload 字节数，不含帧头/长度/CRC。
- 没有帧尾，也不做字节转义（payload 里可以出现 `AA 55`，靠 `len` 定界）。

## 3. 命令帧：视觉 → 下位机（`msg_id = 0x01`）

固定 24 字节。

| 偏移 | 长度 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 2 | `SOF` | — | `AA 55` |
| 2 | 1 | `msg_id` | uint8 | `0x01` |
| 3 | 1 | `seq` | uint8 | 0~255 循环，每帧 +1 |
| 4 | 1 | `len` | uint8 | `17` |
| 5 | 1 | `status` | uint8 | 0=LOST, 1=TRACKING, 2=HOLDING |
| 6 | 4 | `yaw_error` | float32 | 目标相对光轴的 yaw 误差 |
| 10 | 4 | `pitch_error` | float32 | 目标相对光轴的 pitch 误差 |
| 14 | 4 | `yaw_rate` | float32 | 目标角速度估计，供下位机前馈 |
| 18 | 4 | `pitch_rate` | float32 | 同上 |
| 22 | 2 | `crc16` | uint16 | 覆盖偏移 2..21，小端 |

示例：`status=1, yaw=+0.05, pitch=-0.02, yaw_rate=+0.3, pitch_rate=-0.1, seq=0`

```
AA 55 01 00 11 01 CD CC 4C 3D 0A D7 A3 BC 9A 99 99 3E CD CC CC BD CF 82
```

`status` 的语义由 tracker 决定：TRACKING 才有真实观测；HOLDING/LOST 时其余字段为 0。下位机不要对 HOLDING 做位置外推——需要保持时保持自身最后位置即可。

## 4. 反馈帧：下位机 → 视觉（`msg_id = 0x02`）

固定 15 字节。

| 偏移 | 长度 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 2 | `SOF` | — | `AA 55` |
| 2 | 1 | `msg_id` | uint8 | `0x02` |
| 3 | 1 | `seq` | uint8 | 0~255 循环，每帧 +1 |
| 4 | 1 | `len` | uint8 | `8` |
| 5 | 4 | `yaw` | float32 | 云台当前 yaw |
| 9 | 4 | `pitch` | float32 | 云台当前 pitch |
| 13 | 2 | `crc16` | uint16 | 覆盖偏移 2..12，小端 |

示例：`yaw=-0.1, pitch=+0.05, seq=7`

```
AA 55 02 07 08 CD CC CC BD CD CC 4C 3D 8E F4
```

反馈不携带时间戳，`autoaim_serial/recv` 的 `header.stamp` 是视觉侧收到帧的时刻，不是下位机的采样时刻。v1 没有时间同步，因此不要用它做跨端延迟测量。

## 5. CRC16-CCITT-FALSE

参数：poly `0x1021`、init `0xFFFF`、不反转输入/输出、xorout `0x0000`。

覆盖范围：从 `msg_id` 到 payload 末尾（不含 `AA 55`）。CRC 本身按小端写入。

测试向量：`"123456789"`（ASCII）→ `0x29B1`。

```c
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t size) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}
```

## 6. 接收端要求

视觉侧解析器（`FrameParser`）的行为，下位机建议照做：

1. 按字节扫描 `AA 55`，不要假设一次 `read` 拿到完整帧。
2. 读到 `len` 后检查范围（本实现上限 32 字节），非法就丢弃 1 字节重新扫描。
3. 收满 `5 + len + 2` 字节后校验 CRC；失败时丢弃 1 字节重新扫描（坏帧 payload 里可能藏着下一帧的帧头）。
4. CRC 正确但 `msg_id` 未知，按 `len` 跳过并计数，不报错（硬件回环测试时视觉侧会收到自己发的 `0x01`）。
5. `msg_id = 0x02` 但 `len != 8`，或角度不是有限值，整帧丢弃并计数。
6. 帧中途长时间（如 50ms）没有新字节，清空半帧缓存，避免卡死在一个坏长度上。

统计项：`crc_errors`、`ignored_frames`、`resyncs`。联调时任何一项持续增长都说明链路或协议有问题。

## 7. 下位机 C 参考实现

只做"解命令帧 + 编反馈帧"，可以直接移植。

```c
#include <stdint.h>
#include <string.h>

#define SOF0 0xAA
#define SOF1 0x55
#define MSG_COMMAND  0x01
#define MSG_FEEDBACK 0x02
#define COMMAND_LEN  17
#define FEEDBACK_LEN 8
#define MAX_PAYLOAD  32

static uint16_t crc16_ccitt(const uint8_t *data, uint32_t size) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static float read_f32_le(const uint8_t *p) {
    uint32_t bits = (uint32_t)p[0]
                  | ((uint32_t)p[1] << 8)
                  | ((uint32_t)p[2] << 16)
                  | ((uint32_t)p[3] << 24);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void write_f32_le(uint8_t *p, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    p[0] = (uint8_t)(bits);
    p[1] = (uint8_t)(bits >> 8);
    p[2] = (uint8_t)(bits >> 16);
    p[3] = (uint8_t)(bits >> 24);
}

typedef struct {
    uint8_t status;
    float yaw_error;
    float pitch_error;
    float yaw_rate;
    float pitch_rate;
} command_t;

typedef enum {
    WAIT_SOF0, WAIT_SOF1, WAIT_ID, WAIT_SEQ, WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC0, WAIT_CRC1
} parse_state_t;

typedef struct {
    parse_state_t state;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MAX_PAYLOAD];
    uint8_t index;
    uint16_t crc_received;
    command_t command;
    uint32_t crc_errors;
    uint32_t ignored;
} parser_t;

/* 返回 1 表示本次字节凑齐了一帧有效命令帧，command 已更新。 */
int parser_feed(parser_t *p, uint8_t byte) {
    switch (p->state) {
    case WAIT_SOF0:
        if (byte == SOF0) p->state = WAIT_SOF1;
        break;
    case WAIT_SOF1:
        p->state = (byte == SOF1) ? WAIT_ID : (byte == SOF0 ? WAIT_SOF1 : WAIT_SOF0);
        break;
    case WAIT_ID:
        p->msg_id = byte;
        p->state = WAIT_SEQ;
        break;
    case WAIT_SEQ:
        p->seq = byte;
        p->state = WAIT_LEN;
        break;
    case WAIT_LEN:
        p->len = byte;
        p->index = 0;
        p->state = (byte == 0 || byte > MAX_PAYLOAD) ? WAIT_SOF0 : WAIT_PAYLOAD;
        break;
    case WAIT_PAYLOAD:
        p->payload[p->index++] = byte;
        if (p->index == p->len) p->state = WAIT_CRC0;
        break;
    case WAIT_CRC0:
        p->crc_received = byte;
        p->state = WAIT_CRC1;
        break;
    case WAIT_CRC1:
        p->crc_received |= (uint16_t)byte << 8;
        p->state = WAIT_SOF0;
        {
            uint8_t body[3 + MAX_PAYLOAD];
            body[0] = p->msg_id;
            body[1] = p->seq;
            body[2] = p->len;
            memcpy(&body[3], p->payload, p->len);
            const uint16_t crc = crc16_ccitt(body, (uint32_t)p->len + 3);
            if (crc != p->crc_received) {
                ++p->crc_errors;
                break;
            }
            if (p->msg_id == MSG_COMMAND && p->len == COMMAND_LEN) {
                p->command.status = p->payload[0];
                p->command.yaw_error = read_f32_le(&p->payload[1]);
                p->command.pitch_error = read_f32_le(&p->payload[5]);
                p->command.yaw_rate = read_f32_le(&p->payload[9]);
                p->command.pitch_rate = read_f32_le(&p->payload[13]);
                return 1;
            }
            ++p->ignored;
        }
        break;
    }
    return 0;
}

/* 编码一帧反馈，返回 15。 */
uint32_t encode_feedback(uint8_t seq, float yaw, float pitch, uint8_t out[15]) {
    uint16_t crc;
    out[0] = SOF0;
    out[1] = SOF1;
    out[2] = MSG_FEEDBACK;
    out[3] = seq;
    out[4] = FEEDBACK_LEN;
    write_f32_le(&out[5], yaw);
    write_f32_le(&out[9], pitch);
    crc = crc16_ccitt(&out[2], 3 + FEEDBACK_LEN);
    out[13] = (uint8_t)(crc);
    out[14] = (uint8_t)(crc >> 8);
    return 15;
}
```

注意上面是最小实现：CRC 失败后直接回到 `WAIT_SOF0`，极端情况下会漏掉坏帧内部嵌套的下一帧。视觉侧实现会逐字节回退重新扫描，下位机若追求同等鲁棒性，可以保留最近 40 字节并用滑动窗口查找。

## 8. 联调检查表

- [ ] 两端 CRC 测试向量都是 `0x29B1`。
- [ ] 用本文两条 hexdump 对比两端编码结果，逐字节一致。
- [ ] 空闲时无命令帧则下位机超时；`status=0` 与"链路断"进入不同的保护状态。
- [ ] 运行 10 分钟：视觉侧 `crc_errors`、`resyncs` 不增长；下位机 `crc_errors` 不增长。
- [ ] 覆盖：TRACKING、HOLDING、LOST 三种状态的数值与 `status` 一致。
- [ ] 覆盖：拔插 USB、重启视觉节点、重启下位机后能自动恢复。
- [ ] 覆盖：把 TX/RX 短接做硬件回环，视觉侧能忽略自己发出的 `0x01` 帧。

## 9. 版本

| 版本 | 日期 | 变化 |
| --- | --- | --- |
| v1 | 2026-10-26 | 命令帧 24 字节、反馈帧 15 字节、float32 原生类型、CRC16-CCITT |

`msg_id` 是扩展点：新增消息类型用新 id，旧接收端按 `len` 跳过。已有字段的类型、偏移、含义不得复用或重编号；只能追加 `msg_id`。
