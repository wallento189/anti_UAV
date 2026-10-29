# autoaim_serial 串口协议 v1.1（下位机集成说明）

面向读者：云台控制板（下位机）开发者。读完本文应能独立完成两件事：

1. 正确解析视觉侧下发的**命令帧**（`msg_id = 0x01`）；
2. 正确编码并回传**反馈帧**（`msg_id = 0x02`），并知道异常、超时、丢帧时该怎么处理。

视觉侧内部如何算出目标角不在本文范围，只在影响下位机行为的地方给出结论。

---

## 0. 先记住这四条

1. **上电先持续发反馈帧**。视觉侧必须先知道云台当前姿态，才可能给出 `status=1` 的目标角；没有反馈，你只会收到 `status=0`。
2. **`status == 1` 才控制**，其余取值（0/2）只保持位置，不执行任何动作。
3. **命令帧里的角度是"转到多少度"的绝对设定值**，不是"还差多少度"的误差。同一条命令重复收到、延迟到达都不影响结果。
4. **200~300ms 收不到任何命令帧 = 通信故障**，进入保护。这与收到 `status=0` 是两件不同的事。

---

## 1. 物理层与链路

| 项目 | 值 |
| --- | --- |
| 电平/接口 | UART（TTL 或经 USB 转串口） |
| 默认波特率 | `115200` |
| 数据格式 | 8 数据位、无校验、1 停止位（8N1） |
| 流控 | 无（RTS/CTS、XON/XOFF 均关闭） |
| 字节序 | 所有多字节字段**小端** |
| 编码 | 纯二进制，无文本、无换行、无转义 |

- 波特率两端必须一致；视觉侧在 `serial.yaml` 里配置。板子支持的话可以一起提到 921600，但要同时改两端。115200 下命令帧（24 字节 × 约 90Hz）只占约 2.2 kB/s，留给反馈的带宽足够跑到 200Hz 以上。
- 两个方向互不阻塞：命令帧由视觉侧的相机/检测节奏驱动，约 **30Hz**；反馈帧的节奏完全由下位机决定（见第 4 节）。

方向只有两种帧：

```
视觉侧 ──0x01 命令帧──► 下位机（控制律、云台） ──0x02 反馈帧──► 视觉侧
```

---

## 2. 帧格式（两种帧共用）

```
AA 55 │ msg_id │ seq │ len │ payload（len 字节） │ crc16（2 字节）
      └──────────────── CRC 覆盖范围 ────────────────┘
```

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| `SOF` | 2 | 固定 `AA 55` |
| `msg_id` | 1 | `0x01` 是你要接收的命令、`0x02` 是你发送给我的反馈 |
| `seq` | 1 | 发送方每帧 +1，0~255 循环；只用于发现丢帧，不要求连续 |
| `len` | 1 | **仅** payload 字节数，不含帧头、`len` 本身和 CRC |
| `payload` | `len` | 见第 3、4 节 |
| `crc16` | 2 | CRC16-CCITT-FALSE，小端，覆盖 `msg_id` 到 payload 末尾（**不含** `AA 55`） |

- 没有帧尾，也不做字节转义。payload 里可以出现 `AA 55`，接收端**必须靠 `len` 定界**，不能靠找下一个 `AA 55`。
- 一帧的总长度 = `5 + len + 2`。
- 未知 `msg_id`（比如回环测试时自己发出的帧被读回来）按 `len` 跳过并计数，不要当错误。

---

## 3. 命令帧：下位机**接收**（`msg_id = 0x01`）

固定 **24 字节**，`len = 17`。

| 偏移 | 长度 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 2 | `SOF` | — | `AA 55` |
| 2 | 1 | `msg_id` | uint8 | `0x01` |
| 3 | 1 | `seq` | uint8 | 0~255 循环 |
| 4 | 1 | `len` | uint8 | `17` |
| 5 | 1 | `status` | uint8 | 0=LOST、1=TRACKING、2=HOLDING |
| 6 | 4 | `yaw_target` | float32 | 目标 yaw 绝对角（rad） |
| 10 | 4 | `pitch_target` | float32 | 目标 pitch 绝对角（rad） |
| 14 | 4 | `yaw_rate` | float32 | 目标 yaw 角速度估计（rad/s） |
| 18 | 4 | `pitch_rate` | float32 | 目标 pitch 角速度估计（rad/s） |
| 22 | 2 | `crc16` | uint16 | 覆盖偏移 2..21，小端 |

### 3.1 `status`：唯一的控制开关

| 值 | 含义 | 下位机应该做什么 |
| --- | --- | --- |
| 0 | LOST：视觉侧没有目标 | **不控制**。保持最后位置，或按你原有的失靶策略处理 |
| 1 | TRACKING：本帧目标有效 | **唯一允许控制的取值**，按 `yaw_target` / `pitch_target` 驱动 |
| 2 | HOLDING：短暂丢失（连续几帧没检测到） | **不控制**，保持最后位置；不要用它的数值做外推 |

- `status != 1` 时，其余四个数值字段一律为 0。**这些 0 没有任何物理含义**，不要拿它们当控制量。
- 不要用 `status != 1` 的帧去"补零"或"归中"——绝对角语义下，0 意味着云台零位，直接执行会造成大幅机动。

### 3.2 `yaw_target` / `pitch_target`：绝对角，不是误差

定义：**观测那一帧图像时，云台姿态 + 目标相对相机光轴的角偏差**。等价于"要让相机指向目标，云台应该处于的角度"。

因此：

- 用位置环直接追这两个角度即可，**不要把它们当误差做累加**（那等于把位置指令积分成速度指令）。
- 重复执行同一帧、延迟到达、突然补发，都不会让云台多走一段；丢帧也不会丢积分量。
- `yaw_rate` / `pitch_rate` 是同一个目标的角度变化率估计，可用于前馈或提前量补偿；没有可信估计时会为 0，此时当作"无前馈"即可。
- 与反馈帧的 `yaw` / `pitch` **同一坐标、同一零位、同一符号**，视觉侧不做任何二次变换。

符号约定（全链路统一）：

| 量 | 正方向 | 单位 |
| --- | --- | --- |
| `yaw_target`、`yaw_rate`、反馈 `yaw` | 向左 | rad、rad/s |
| `pitch_target`、`pitch_rate`、反馈 `pitch` | 向下 | rad、rad/s |

**角度不做 ±π 展开**：`yaw_target` 可能超出 ±π（例如 `4.71` 或 `-3.5`）。下位机必须能接受连续角；如果你的云台角是有界回绕的，请自行取最近等价角（`target + 2πk` 中离当前角最近的那个）。

### 3.3 什么时候会收不到命令帧

- 视觉侧每处理一帧图像就发一帧，约 90Hz。
- 视觉侧暂时拿不到与该帧图像同一时刻的云台姿态时，会**丢掉这一帧 TRACKING 命令**（不会发假数据）。所以命令帧出现几帧的空洞是正常的；`status=0/2` 的帧不受影响，照常发送。
- 视觉侧整体挂掉（相机、检测、或上位机进程）时，链路会完全静默。
- 无论哪种情况，**下位机都只看自己的超时计时器**：建议 200~300ms 没有任何有效命令帧就进入保护。视觉侧不会发心跳，也不会有"假 LOST"帧来续命。

### 3.4 解码示例

`status=1, yaw_target=+0.05, pitch_target=-0.02, yaw_rate=+0.3, pitch_rate=-0.1, seq=0`：

```
AA 55 01 00 11 01 CD CC 4C 3D 0A D7 A3 BC 9A 99 99 3E CD CC CC BD CF 82
```

| 字段 | 字节 | 解析结果 |
| --- | --- | --- |
| `SOF` | `AA 55` | — |
| `msg_id` | `01` | `0x01` 命令帧 |
| `seq` | `00` | 0 |
| `len` | `11` | 17 |
| `status` | `01` | 1 = TRACKING |
| `yaw_target` | `CD CC 4C 3D` | `0x3D4CCCCD` = **+0.05** |
| `pitch_target` | `0A D7 A3 BC` | `0xBCA3D70A` = **-0.02** |
| `yaw_rate` | `9A 99 99 3E` | `0x3E99999A` = **+0.3** |
| `pitch_rate` | `CD CC CC BD` | `0xBDCCCCCD` = **-0.1** |
| `crc16` | `CF 82` | `0x82CF`（小端） |

---

## 4. 反馈帧：下位机**发送**（`msg_id = 0x02`）

固定 **15 字节**，`len = 8`。

| 偏移 | 长度 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 2 | `SOF` | — | `AA 55` |
| 2 | 1 | `msg_id` | uint8 | `0x02` |
| 3 | 1 | `seq` | uint8 | 0~255 循环，每帧 +1 |
| 4 | 1 | `len` | uint8 | `8` |
| 5 | 4 | `yaw` | float32 | 云台当前 yaw（rad） |
| 9 | 4 | `pitch` | float32 | 云台当前 pitch（rad） |
| 13 | 2 | `crc16` | uint16 | 覆盖偏移 2..12，小端 |

### 4.1 什么时候发

- **一直发，不要等命令帧**。上电初始化完成就开始，频率由你决定。
- 视觉侧每帧图像都会去匹配时间上最近的反馈，**反馈越密，目标角越准**：
  - 当前配置要求：最近一帧反馈与图像时刻的间隔 ≤ 50ms（对应 ≥ 20Hz），否则该帧不给目标角。
  - 实际建议：**100~200Hz**。115200 下 15 字节/帧约 1.3ms，100Hz 只占约 1.5 kB/s，不会挤占命令帧。
  - 反馈低于 20Hz 时，大部分观测帧都找不到 50ms 内的姿态，视觉侧基本进不了 TRACKING——表现就是"上位机一直发 status=0，云台不动"。
- 反馈可以比命令帧快，也可以完全不看命令帧的节奏，**但不要停发**，也不要把多帧合并成一帧。

### 4.2 填什么值

- `yaw` / `pitch`：**当前真实姿态**，与命令帧同一零位、同一符号（左正、下正），单位 rad。
- 反馈的是姿态而不是"电机编码器原始值"：如果编码器零位、减速比、方向与云台零位不一致，请在板子上换算好再发。视觉侧不会做二次变换。
- 允许超出 ±π，也允许负值。
- 发送前检查有限性，NaN/Inf 帧会被视觉侧整帧丢弃。

### 4.3 编码示例

`yaw=-0.1, pitch=+0.05, seq=7`：

```
AA 55 02 07 08 CD CC CC BD CD CC 4C 3D 8E F4
```

| 字段 | 字节 | 说明 |
| --- | --- | --- |
| `SOF` | `AA 55` | — |
| `msg_id` | `02` | `0x02` 反馈帧 |
| `seq` | `07` | 7 |
| `len` | `08` | 8 |
| `yaw` | `CD CC CC BD` | `0xBDCCCCCD` = **-0.1** |
| `pitch` | `CD CC 4C 3D` | `0x3D4CCCCD` = **+0.05** |
| `crc16` | `8E F4` | `0xF48E`（小端） |

---

## 5. CRC16-CCITT-FALSE

| 参数 | 值 |
| --- | --- |
| 多项式 | `0x1021` |
| 初值 | `0xFFFF` |
| 输入/输出反转 | 否 |
| 结果异或 | `0x0000` |
| 覆盖范围 | `msg_id`、`seq`、`len`、payload（**不含** `AA 55`） |
| 写入顺序 | 小端 |

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

---

## 6. 收发实现要求

### 6.1 接收（解析命令帧）

1. 把串口当**字节流**处理，不要假设一次 `read` 能拿到完整帧；逐字节喂给状态机最稳。
2. 扫描 `AA 55` 作为帧头。若当前字节是 `AA` 但下一个不是 `55`，要重新判断这个 `AA` 是否可能是新帧头（常见实现：回退一个字节重扫）。
3. 读到 `len` 后立刻检查范围：**只接受 `1 ≤ len ≤ 32`**，非法就丢弃 1 字节重新扫描。
4. 收满 `5 + len + 2` 字节后算 CRC。失败就丢弃 1 字节重新扫描——坏帧的 payload 里可能正好藏着下一帧的帧头，整段丢弃会连带吞掉好帧。
5. CRC 正确但 `msg_id` 未知：按 `len` 跳过，计数，不报错。
6. `msg_id = 0x01` 但 `len != 17`：整帧丢弃并计数。
7. 帧中途超过 50ms 没有新字节：清空半帧缓存，避免卡死在一个错误长度上。

### 6.2 发送（编码反馈帧）

1. **禁止把结构体直接 `memcpy` 上串口**：逐字段、逐字节写，避免对齐、填充字节和小端问题。
2. 每帧 15 字节，`len` 固定 8，CRC 覆盖 `msg_id` 起始的 `3 + 8 = 11` 字节。
3. 发送前检查 `yaw` / `pitch` 是否为有限值。
4. 串口写和你的控制环不在同一上下文时，用快照/双缓冲把最新姿态传给发送侧，避免读到一半被更新。
5. 写操作要能容忍阻塞或部分写入；不要因为一次写失败就停掉整个控制环。

### 6.3 必须实现的保护

| 情况 | 建议动作 |
| --- | --- |
| 连续 200~300ms 收不到任何命令帧 | 判定通信故障，进入保护（保持/回零/断发射，按整机策略） |
| 命令帧持续到达但一直是 `status=0` | 通信正常，是"没有目标"或"视觉侧没拿到你的反馈"。先查上行反馈链路，别当成串口故障 |
| 收到 `status = 0` 或 `2` | 不控制，保持最后位置 |
| 连续出现 CRC 错误或帧头对不上 | 记录计数并告警；持续增长说明波特率、线序或共地有问题 |
| 反馈帧发不出去 | 控制环照常运行；不要因此停止回传尝试，视觉侧恢复后需要立刻拿到姿态 |

---

## 7. C 参考实现

只做两件事：解命令帧、编反馈帧，可直接移植。省略了平台相关的串口读写。

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
#define CMD_TIMEOUT_MS 250

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
    float yaw_target;   /* 绝对角，不是误差 */
    float pitch_target; /* 绝对角，不是误差 */
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
    uint32_t last_frame_ms; /* 由调用方在收到有效帧时更新 */
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
                p->command.yaw_target = read_f32_le(&p->payload[1]);
                p->command.pitch_target = read_f32_le(&p->payload[5]);
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

主循环建议结构（伪代码）：

```c
parser_t parser = {0};
uint32_t last_cmd_ms = now_ms();
uint32_t last_feedback_ms = now_ms();
uint8_t feedback_seq = 0;

while (1) {
    /* 1) 每个控制周期都发反馈，不要等命令帧 */
    if (now_ms() - last_feedback_ms >= 5) {       /* 200Hz */
        uint8_t frame[15];
        encode_feedback(feedback_seq++, gimbal_yaw, gimbal_pitch, frame);
        uart_write(frame, sizeof(frame));         /* 允许失败，下周期重试 */
        last_feedback_ms = now_ms();
    }

    /* 2) 收命令帧，只认 status == 1 */
    uint8_t buf[64];
    int n = uart_read(buf, sizeof(buf), 10);
    for (int i = 0; i < n; ++i) {
        if (parser_feed(&parser, buf[i])) {
            last_cmd_ms = now_ms();
            if (parser.command.status == 1) {
                aim_at(parser.command.yaw_target, parser.command.pitch_target,
                       parser.command.yaw_rate, parser.command.pitch_rate);
            } else {
                hold_position();                  /* 不控制，保持 */
            }
        }
    }

    /* 3) 超时保护独立于 status */
    if (now_ms() - last_cmd_ms > CMD_TIMEOUT_MS) {
        link_lost_protection();
    }
}
```

注意：上面的解析器是**最小实现**，CRC 失败后直接回到 `WAIT_SOF0`，极端情况下会漏掉坏帧内部嵌套的下一帧。追求鲁棒性时按 6.1 的第 4 条改为逐字节回退重扫（保留最近 40 字节滑动窗口）。

---

## 8. 联调检查表

- [ ] 两端 CRC 测试向量都是 `0x29B1`。
- [ ] 用第 3.4、4.3 节的两条 hexdump 对比两端编解码结果，逐字节一致。
- [ ] 收到 `status=1` 才动；0 和 2 都不动。
- [ ] 拔掉串口线 → 250ms 内进入保护；插回后能自动恢复控制。
- [ ] 目标在画面里移动时，`yaw_target` 随云台姿态平稳变化；若它跟着云台一起"追"自己，说明反馈姿态与实际不符。
- [ ] `yaw_target` 跨过 ±π 时云台不跳变。
- [ ] 反馈频率实测 ≥ 100Hz，且视觉侧日志里没有"没有可用姿态"一类告警。
- [ ] 运行 10 分钟：下位机的 CRC 错误计数不增长；上位机的 `crc_errors` / `resyncs` 不增长。
- [ ] 把 TX/RX 短接做硬件回环：读回自己发出的 `0x02` 时，命令解析器应因 `msg_id != 0x01` 跳过并只计入 `ignored`，不能误当命令执行。
- [ ] 重启任意一端后链路能自动恢复。

