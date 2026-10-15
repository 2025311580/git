/*
 * MSPM0G3507 移植 F32C TTL 无刷电机位置闭环例程。
 * 电机 TTL 通信使用 UART1。
 *
 * 本版本流程：上电握手 → 识别接的是哪个 ID 的电机 → 平滑回零到 0°
 *            → 自动在 0°↔180° 之间往返。
 * 运动曲线（梯形加减速）在 Control/f32c_motion.c，本文件只负责协议收发。
 */
#include "board.h"
#include "uart_callback.h"
#include "f32c_motion.h"

/* 位置闭环变量：Speed 为位置模式下的运行速度，T_Position 为目标位置，Current_Position 为反馈位置。
 * 带 volatile 的那几个是 UART1 接收中断里写的，主循环只读，见 uart_callback.h 的说明。 */
int Motor1_Speed = F32C_RUN_SPEED_RPM, Motor2_Speed = F32C_RUN_SPEED_RPM;
volatile int motor1_Current_Speed, motor2_Current_Speed;
int Motor1_T_Position = 0, Motor2_T_Position = 0;
volatile int Motor1_Current_Position, Motor2_Current_Position;

/* 电机 ID 只有 F32C_MotorID 这一个来源（f32c_motion.c 里定义，开机识别后写入；
 * 0 = 没识别出来）。不要另外再定义一份 "motor1_ID = 1" 之类的常量 —— 两套 ID
 * 迟早会对不上：发目标帧用一个、发配置帧用另一个，症状是电机莫名其妙不动。 */

/* 控制模式（F32C 手册 4.2.2 表3）：
 *   0x0000 速度模式              0x0001 多圈位置（带 T 型轨迹规划）
 *   0x0002 单圈绝对（带 T 型轨迹） 0x0003 多圈位置（直通）
 *   0x0004 单圈绝对（直通）
 * 选 0x0004：单圈绝对模式下 0 度是一个固定的机械位置，配合手册 4.2.10
 * "单圈绝对角度设置零点"（可掉电保存），每次上电电机都回到同一个点。
 * 直通 vs T 型：手册 P7 说"需要高频率的发送改变目标位置，建议选择直通模式"，
 * 本工程主循环就是 150Hz 反复下发目标位置，所以用直通；加减速不靠驱动板，
 * 由 f32c_motion.c 在 MCU 这边铺成梯形曲线再喂过去（直通模式下直接把 0 跳
 * 到 1800 发出去，驱动板会按"运行速度"上限猛冲，到位容易过冲来回晃）。
 * 注意：手册 P7 注明电机上电默认为速度模式，所以每次开机都必须重发模式命令。
 *
 * ★ 必须是 u16：模式值是 2 字节，帧里拆成 [3] 高字节、[4] 低字节。以前这里是
 *   u8，`mode >> 8` 恒为 0，恰好 0x0004 低字节就是全部所以没出事；哪天想换成
 *   ≥0x0100 的模式，高字节会被静默吃掉，写出来变成另一个模式（甚至 0x0000
 *   速度模式，电机完全不理会位置指令）。 */
u16 F32C_Mode = 0x0004;

/* 【一次性开关】置 1 烧录一次：把驱动板恢复出厂设置（手册 4.2.11），
 * 也就是把"单圈绝对零点"还原成出厂那个零点，冲掉后来用 0x0A 设的偏移量。
 *
 * ⚠️ 两点注意：
 * 1. 手册只给了帧格式（7A 02 0B 73 7B），没写明恢复范围。按"恢复出厂设置"
 *    的字面意思，电机 ID 很可能也会被还原成出厂默认的 2，加速度等参数同理。
 *    本程序每次上电都会重发使能/模式/速度并自动识别 ID，所以这些被还原了
 *    也一样能跑，不用手动改回。
 * 2. 恢复完把这个开关改回 0 再烧一次，否则每次上电都会恢复一遍出厂。 */
#define F32C_FACTORY_RESET  0

/* ============================================================
 * 【一次性开关】开机干什么。平时保持 0。
 *
 *   0 = 正常运行：回零 → 0°↔180° 往返
 *   1 = 只失能：把电机松开，然后 MCU 彻底安静（不发任何帧）。用来手拧
 *       电机，或者腾出串口总线让 WHEELTEC 上位机设零点。
 *   2 = 把电机"现在这个位置"设成零点：不发使能、不下发任何目标，只做
 *       识别 → 失能 → 0x0A 设零点 → 0x08 保存 → 停住。
 *
 * 为什么 1/2 不能走正常开机流程：正常流程开头要"使能 + 选模式"，而驱动板
 * 里还存着上一轮的目标角度，一使能就可能往那个旧目标冲过去，把刚摆好的
 * 位置弄跑。所以这两条路全程不使能。
 *
 * ⚠️ 用完记得改回 0 再烧一次。2 不改回来的话，以后每次上电都会把"当时电机
 *    所在的位置"重新定义成 0 点，固定回零就失效了。
 * ⚠️ 失能故意不跟 0x08 保存：免得哪天忘了改回来，以后每次上电电机都是松的。
 * ⚠️ 上位机的 USB-TTL 和 MCU 的 TX 要是并在同一条线上，两个都是推挽输出会
 *    顶牛。最稳的是把 MCU 的 TX 那根线拔掉（GND 留着），只让上位机发话。
 * ============================================================ */
#define F32C_BOOT_MODE  0

/* 目标角度下发节拍：每几拍发一次。1 = 每拍都发。
 * 一拍 6.667ms（见 f32c_motion.h 的 F32C_TICK_US），所以 1 → 150Hz —— 正好是
 * 中断节拍本身，帧间隔均匀，不靠"每隔几拍"制造抖动。
 * 驱动板手册 1.2 节：控制频率建议 100Hz、最大 200Hz。直通模式下驱动板是"照着
 * 移动目标追"，目标每拍跳多大电机就跟得多顿：当前曲线峰值 1440°/s，100Hz 每拍
 * 要跳 14.4°，150Hz 只跳 9.6°，波动小 1/3。
 * 调大就是降频（2 → 75Hz、3 → 50Hz），要更低频改这个数。别往 1 以下调，
 * 手册上限 200Hz，而且一拍也就发得出一帧。 */
#define F32C_SEND_EVERY     1

/* 每隔几个下发节拍问一次机械角度反馈（4 → 37.5Hz）。
 * 到位判据和 OLED 显示用几十 Hz 完全够；少问一帧就少占一点时间片，
 * 目标流的节奏更稳。想回到 50Hz 就改成 3（那一拍要发 3 帧共 ~4.7ms，
 * 6.667ms 的节拍还塞得下，但余量只剩不到 2ms）。 */
#define F32C_FB_EVERY       4

/* 反馈帧还没收到过时的哨兵值，用来判断串口那头到底有没有电机在回答。 */
#define F32C_FB_NONE        (-100000)

/* 以下协议数组保持参考例程写法：帧头 0x7A，帧尾 0x7B，中间 BCC 异或校验。 */
u8 motor1_Enable_data[5] = {0x7A, 0x01, 0x06, 0x7D, 0x7B};
u8 motor2_Enable_data[5] = {0x7A, 0x02, 0x06, 0x7E, 0x7B};

u8 motor1_Mode_data[7] = {0x7A, 0x01, 0x00, 0x00, 0x00, 0x7B, 0x7B};
u8 motor2_Mode_data[7] = {0x7A, 0x02, 0x00, 0x00, 0x00, 0x78, 0x7B};

u8 motor1_Speed_data_task[7] = {0x7A, 0x01, 0x01, 0x00, 0x00, 0x00, 0x7B};
u8 motor2_Speed_data_task[7] = {0x7A, 0x02, 0x01, 0x00, 0x00, 0x00, 0x7B};

/* 单圈绝对角度控制（手册 4.2.5 表6）：功能码 0x03，2 字节数据，0~3599 即 0~359.9 度。
 * 注意与"多圈绝对角度"（功能码 0x02，4 字节有符号）不是一套格式，别混。
 *
 * 1 号 / 2 号的模板只差第 [1] 字节的地址，而那一位每次发送前都要按实际 ID 重填、
 * BCC 也要重算，所以只留一份，地址由 F32C_SendTarget() 的参数决定。上面的使能 /
 * 模式 / 速度三组也同理（各留了两份是为了少动已跑通的代码）。 */
u8 f32c_Position_data_task[7] = {0x7A, 0x01, 0x03, 0x00, 0x00, 0x00, 0x7B};

/* 0x0E 0x02 请求"机械角度"反馈（单圈绝对角度，手册 4.2.13）。 */
u8 motor1_Feedback_data[6] = {0x7A, 0x01, 0x0E, 0x02, 0x77, 0x7B};
u8 motor2_Feedback_data[6] = {0x7A, 0x02, 0x0E, 0x02, 0x74, 0x7B};

/* 下发"使能 + 选择控制模式 + 设置位置模式运行速度"这一组命令，
 * 即手册 P17 控制步骤的第 1~3 步（手册 4.2.6 / 4.2.2 / 4.2.3）。
 * 这三条命令都不会让电机转动，重复发送是安全的；BCC 每次重算。
 *
 * ⚠️ 速度这一步（功能码 0x01）绝对不能只发一次：
 * 位置模式下"运行速度"为 0 时，目标角度指令发过去电机也一动不动
 * （实测：只发使能+模式+目标角度，电机稳在原地完全不理；补发速度后
 * 立刻转到目标角度）。驱动板掉电重启后运行速度会丢，所以必须跟着
 * 使能/模式一起周期性重发，否则"电机单独断电再上电就回不到 0 点"。
 *
 * 做成"一次只发一帧"的轮流器（rr 走 0→使能、1→模式、2→速度）：
 * 整组 6 帧一起发要十几毫秒，主循环在这期间一帧目标角度都发不出去，
 * 会把 150Hz 的目标流拱出一个洞 —— 表现就是每秒顿一下。拆成每拍捎一帧
 * 就完全看不出来了。 */
static void F32C_SendSetupOne(void)
{
    static u8 rr = 0;
    u8  addr  = (rr <= 2) ? (F32C_MotorID ? F32C_MotorID : 1) : 2;
    u8  which = rr % 3;                       /* 0=使能 1=模式 2=速度 */
    u8 *en, *md, *sp;

    if (addr == 2)
    {
        en = motor2_Enable_data;  md = motor2_Mode_data;  sp = motor2_Speed_data_task;
    }
    else
    {
        en = motor1_Enable_data;  md = motor1_Mode_data;  sp = motor1_Speed_data_task;
    }

    if (which == 0)
    {
        en[1] = addr;
        en[3] = BCC_Sum1(en, 3);
        USART1_SEND(en, sizeof(motor1_Enable_data));
    }
    else if (which == 1)
    {
        md[1] = addr;
        md[3] = (u8)(F32C_Mode >> 8);
        md[4] = (u8)F32C_Mode;
        md[5] = BCC_Sum1(md, 5);
        USART1_SEND(md, sizeof(motor1_Mode_data));
    }
    else
    {
        sp[1] = addr;                          /* 轮发也要带地址，别串到另一个 ID */
        sp[5] = BCC_Sum1(sp, 5);
        USART1_SEND(sp, sizeof(motor1_Speed_data_task));
    }

    /* 识别到 ID 就只轮它那三帧；没识别出来就把两个地址都轮一遍，
     * 谁在线谁动。 */
    rr++;
    if (rr >= (u8)((F32C_MotorID == 0) ? 6 : 3)) rr = 0;
}

/* 开机握手用：此刻没有实时性要求，一次把整组 6 帧发完。
 * 这时 F32C_MotorID 还是 0，所以两个地址都会发到，把在线的那个叫醒。 */
static void F32C_SendSetup(void)
{
    u8 i;

    for (i = 0; i < 6; i++)
    {
        F32C_SendSetupOne();
        delay_ms(2);
    }
}

/* 开机握手：反复重发"使能+模式"，直到驱动器必然已经上电自检完毕。
 * 手册 P17 特意写了"先给电机通电后再给单片机通电"—— 说明驱动器启动比
 * 单片机慢得多。若两块板一起上电，开机那一次命令会发在驱动器就绪之前而
 * 全部丢失，驱动器就停在【出厂默认的速度模式、转速0】，症状是电机是软的、
 * 完全不理位置指令，看起来就像"位置模式没生效"。这里重复发 3 秒兜住这个
 * 时间差。
 * 关键：这期间不发任何目标位置，所以电机不会动 —— 下一步标定零点时，
 * 电机仍然停在用户摆好的那个位置上。 */
static void F32C_StartupHandshake(void)
{
    u8 i;

    for (i = 0; i < 15; i++)   /* 15 * 200ms = 3s */
    {
        F32C_SendSetup();
        delay_ms(200);
    }
}

/* 下发一帧单圈绝对目标角度（当前曲线点）。addr 是要发给哪个 ID 的电机。 */
static void F32C_SendTarget(u8 addr, int x10)
{
    if (x10 < 0)     x10 = 0;
    if (x10 > 3599)  x10 = 3599;

    f32c_Position_data_task[1] = addr;
    f32c_Position_data_task[3] = (u8)((x10 >> 8) & 0xFF);
    f32c_Position_data_task[4] = (u8)(x10 & 0xFF);
    f32c_Position_data_task[5] = BCC_Sum1(f32c_Position_data_task, 5);
    USART1_SEND(f32c_Position_data_task, sizeof(f32c_Position_data_task));
}

/* 识别接的是哪个 ID 的电机：分别向 1 号 / 2 号要一帧机械角度，
 * 谁回了就是谁（驱动板只应答地址匹配的帧，同一条总线上多机也不会答错）。
 * 顺便把它当前的真实角度当回零起点 —— 否则曲线从 0 起步、而电机实际在
 * 150°，一上电 setpoint 就先跳一大步。
 * 返回识别到的 ID；0 表示两边都没搭理（没接电机 / 没上电 / 线接反）。 */
static u8 F32C_DetectMotor(int *start_x10)
{
    u8 i, id = 0;

    Motor1_Current_Position = F32C_FB_NONE;
    Motor2_Current_Position = F32C_FB_NONE;

    for (i = 0; i < 10; i++)
    {
        USART1_SEND(motor1_Feedback_data, sizeof(motor1_Feedback_data));
        delay_ms(3);
        USART1_SEND(motor2_Feedback_data, sizeof(motor2_Feedback_data));
        delay_ms(3);

        if (Motor1_Current_Position != F32C_FB_NONE ||
            Motor2_Current_Position != F32C_FB_NONE)
        {
            break;
        }
    }

    if (Motor2_Current_Position != F32C_FB_NONE)
    {
        id        = 2;
        *start_x10 = Motor2_Current_Position;
    }
    else if (Motor1_Current_Position != F32C_FB_NONE)
    {
        id        = 1;
        *start_x10 = Motor1_Current_Position;
    }
    else
    {
        id        = 0;
        *start_x10 = 0;
    }

    /* 哨兵值不能留在全局变量里（OLED 显示和到位判据都会用到它）*/
    if (Motor1_Current_Position == F32C_FB_NONE) Motor1_Current_Position = 0;
    if (Motor2_Current_Position == F32C_FB_NONE) Motor2_Current_Position = 0;

    return id;
}

/* 把刚收到的机械角度喂给运动模块（到位判据用）。 */
static void F32C_UpdateFeedback(void)
{
    if (F32C_MotorID == 1)      F32C_Motion_Feedback(Motor1_Current_Position);
    else if (F32C_MotorID == 2) F32C_Motion_Feedback(Motor2_Current_Position);
}

#if (F32C_BOOT_MODE == 2)
/* 【一次性标定】把电机此刻所处位置烧成单圈绝对 0 点（手册 4.2.10，掉电保存）。
 * 手册原话："可控制电机转动到合适的角度后发送此协议即可将目前该位置设置为
 * 单圈绝对角度 0 度，可掉电保存" —— 没要求先使能，所以全程让电机失能着，
 * 位置最保险。地址用识别出来的 ID，接 1 号还是 2 号都能标。 */
static void F32C_CalibrateZero(u8 id)
{
    u8 set_zero[5] = {0x7A, 0x00, 0x0A, 0x00, 0x7B};   /* 手册 4.2.10 */
    u8 save_data[5] = {0x7A, 0x00, 0x08, 0x00, 0x7B};   /* 手册 4.2.8 保存参数 */

    set_zero[1]  = id;
    set_zero[3]  = BCC_Sum1(set_zero, 3);
    save_data[1] = id;
    save_data[3] = BCC_Sum1(save_data, 3);

    /* 先让驱动器把模式切换彻底坐实，再标定，避免命令挤在一起。 */
    delay_ms(200);
    USART1_SEND(set_zero, sizeof(set_zero));
    delay_ms(50);
    /* 4.2.10 已注明"可掉电保存"，这里再补一条 0x08 兜底
     * （上位机说明里也是"设置零点"之后再点"保存设备参数"）。 */
    USART1_SEND(save_data, sizeof(save_data));
    delay_ms(200);
}
#endif

#if F32C_FACTORY_RESET
/* 【一次性】恢复出厂设置（手册 4.2.11）：把驱动板里存的偏移量冲掉，
 * 单圈绝对零点回到出厂时定的那个位置。
 * 手册没写明这条命令恢复哪些参数，ID 可能一起被还原成出厂默认的 2 ——
 * 上面 F32C_DetectMotor 会自动认出新的 ID，不用手动改。 */
static void F32C_RestoreFactory(u8 id)
{
    u8 factory[5]   = {0x7A, 0x00, 0x0B, 0x00, 0x7B};   /* 手册 4.2.11 */
    u8 save_data[5] = {0x7A, 0x00, 0x08, 0x00, 0x7B};   /* 手册 4.2.8 保存参数 */

    factory[1]   = id;
    factory[3]   = BCC_Sum1(factory, 3);
    save_data[1] = id;
    save_data[3] = BCC_Sum1(save_data, 3);

    /* 先让驱动器把模式切换彻底坐实，再动手，避免命令挤在一起。 */
    delay_ms(200);
    USART1_SEND(factory, sizeof(factory));
    delay_ms(200);
    /* 手册没写这条是否掉电保存，补一条 0x08 兜底。 */
    USART1_SEND(save_data, sizeof(save_data));
    delay_ms(200);
}
#endif

#if (F32C_BOOT_MODE != 0)
/* 失能电机（手册 4.2.6）：帧头 7A + 地址 + 功能码 0x05 + BCC + 帧尾 7B。
 * 注意功能码是 0x05 本身，不是"0x06 带一个数据位"——
 * ID=1 → 7A 01 05 7E 7B，ID=2 → 7A 02 05 7D 7B。 */
static void F32C_DisableMotor(u8 id)
{
    u8 off[5] = {0x7A, 0x00, 0x05, 0x00, 0x7B};

    off[1] = id;
    off[3] = BCC_Sum1(off, 3);

    USART1_SEND(off, sizeof(off));
    delay_ms(20);          /* 等这一帧发完。之后本程序再也不碰串口了。 */
}
#endif

int main(void)
{
    u8  motor_id;
    int start_x10 = 0;
    u32 last_send_tick  = 0;
    u32 last_setup_tick = 0;
    u8  setup_left      = 0;   /* 本次"每秒重发"还剩几帧要捎出去 */
    u8  fb_div          = 0;   /* 反馈分频计数 */

    /* SysConfig 已完成系统时钟、UART1、定时器等初始化。 */
    SYSCFG_DL_init();
    DL_GPIO_setDigitalInternalResistor(GPIO_UART_1_IOMUX_RX, DL_GPIO_RESISTOR_PULL_UP);
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);

    /* 开启 UART1 接收中断和 TIMER0 周期中断。 */
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* 这块板子上没接 OLED，所以没有任何显示代码。OLED 驱动（软 SPI 那套）
     * 和按键扫描都已在 2026-10 的清理里删掉 —— 要接回来从 git 历史里取，
     * 并在 SysConfig 的 GPIO 里把 OLED_RST/DC/SCL/SDA 或 KEY/USER_KEY
     * 重新加回来（那两批引脚当时也一并从配置里移除了）。 */

    /* 参考例程先发送 0x00 唤醒串口，避免上电首字节丢失；随后等待电机上电完成。 */
    usart1_send(0x00);
    delay_ms(1500);

#if (F32C_BOOT_MODE != 0)
    /* ============ 一次性模式（见文件上方 F32C_BOOT_MODE）============
     * 故意绕开下面的正常开机流程：正常流程开头会"使能 + 选模式"，而驱动板
     * 里还存着上一轮的目标角度，一使能就可能往那个旧目标冲过去，把刚摆好的
     * 位置弄跑。这条路全程不使能，只读角度、失能、写参数。 */
    {
        u8 id = 0;
        u8 i;

        /* 反复问到驱动板应答为止（它启动比单片机慢）。最多等 5 秒。 */
        for (i = 0; i < 25 && id == 0; i++)
        {
            id = F32C_DetectMotor(&start_x10);
            if (id == 0) delay_ms(200);
        }

        if (id == 0)
        {
            /* 一个都没应答就什么都别写 —— 宁可这次不生效，也不能把零点写到
             * 猜出来的地址上。电机保持原状，重新上电再试即可。 */
            while (1) { delay_ms(500); }
        }

        /* 先失能，让电机彻底松开：设零点那一下没有任何电流冲击 */
        F32C_DisableMotor(id);

#if (F32C_BOOT_MODE == 2)
        /* 把"电机现在这个物理位置"设成单圈绝对 0 点，并保存 */
        F32C_CalibrateZero(id);

        /* 设完回读一次机械角度确认：零点真写进去了的话，同一个物理位置上
         * 这里应该读到 ~0。读回来的值留在 Motor1/Motor2_Current_Position 里，
         * 接着仿真器看这两个变量就能判断到底成没成（4.2.10 设的就是
         * "机械角度"的 0 点，上位机那个按钮就叫"当前机械角度设置为0点"）。 */
        F32C_DetectMotor(&start_x10);
#endif

        /* 干完就停住：此后不发任何帧，定时器里的 LED 心跳照跑 */
        while (1) { delay_ms(500); }
    }
#endif

    /* 先把"位置模式运行速度"帧的内容填好：后面的开机握手和周期性重发
     * 都要带着它一起发，所以必须赶在第一次 F32C_SendSetup 之前准备好。 */
    motor1_Speed_data_task[3] = (int)Motor1_Speed >> 8;
    motor1_Speed_data_task[4] = (int)Motor1_Speed;
    motor2_Speed_data_task[3] = (int)Motor2_Speed >> 8;
    motor2_Speed_data_task[4] = (int)Motor2_Speed;

    /* 1~3. 使能 + 选择控制模式 + 设置运行速度（手册 P17 的控制步骤）。
     * 模式命令格式 7A <ID> 00 <模式高8位> <模式低8位> <BCC> 7B：模式值写在
     * [3][4]；[2] 是功能码必须保持 0x00，写别的会被当成别的命令（例如 0x01
     * 就变成速度命令，电机只会停在转速 0、完全不理会位置指令）。
     * 这三条命令反复重发直到驱动器就绪，原因见 F32C_StartupHandshake。 */
    F32C_StartupHandshake();

    /* 4. 认出接的是哪个电机，并记下它现在在什么角度（回零的起点）*/
    motor_id = F32C_DetectMotor(&start_x10);

#if F32C_FACTORY_RESET
    if (motor_id != 0)
    {
        F32C_RestoreFactory(motor_id);

        /* 零点变了，起点角度必须重读 —— 否则曲线还从出厂前的旧角度起步，
         * 第一帧发出去就是一大跳。ID 也可能被一起还原（出厂默认 2），
         * 这里重新识别一遍正好一并处理。 */
        motor_id = F32C_DetectMotor(&start_x10);
    }
#endif

    /* 5. 开始运动：先回零，回零完成后自动开始 0°↔180° 往返。
     *    曲线和状态机都在 f32c_motion.c，这里只是"点火"。
     *    （原来还有短按暂停 / 长按回零的按键，现在没接按键，已去掉。）*/
    F32C_Motion_Begin(motor_id, start_x10);

    /* 主循环就干一件事：按节拍把 setpoint 发出去。
     * 这里没有任何阻塞的活了（OLED 已去掉），目标帧的节奏是均匀的 ——
     * 这才是"丝滑"的前提：驱动板 150Hz 均匀收到小步移动的目标，就跟着
     * 匀速走；要是哪一拍发晚了，它只能保持旧目标，下一拍就得跳一大步。 */
    while (1)
    {
        /* ---- a. 每秒把"使能+模式+速度"重发一遍 ----
         * 驱动板若因干扰、欠压、或者电机电源被单独关掉再打开而自行复位，
         * 会掉回出厂的速度模式、运行速度归 0，从而完全忽略位置指令 ——
         * 表现就是"电机回不到 0 点"。重发能把状态整个拉回来。
         * 这三条命令本身不会让电机转动，所以不影响位置保持。
         * 帧不是集中发的，而是在下面 b 之后一拍捎一帧（setup_left），
         * 免得一次占十几毫秒把 150Hz 的目标流拱出一个洞。 */
        if ((u32)(F32C_TickCount - last_setup_tick) >= F32C_MS_TO_TICKS(1000))
        {
            last_setup_tick = F32C_TickCount;
            setup_left      = 3;
        }

        /* ---- b. 每一拍下发一次当前曲线点（150Hz）----
         * 手册 1.2 节建议 100Hz、上限 200Hz。这里贴着中断节拍走：直通模式下
         * 驱动板是"照着移动目标追"，目标每拍跳多大电机就跟得多顿 —— 当前曲线
         * 峰值 1440°/s，100Hz 每拍要跳 14.4°，150Hz 只跳 9.6°，波动小 1/3。
         * ⚠️ 上限不是 150 而是 200 在硬件上办不到：115200 下目标帧 7 字节要
         * 608µs，加上手册要求的帧间 ≥1ms 空闲，一帧就占 1.6ms；反馈请求
         * 6 字节再来 1.5ms。200Hz 只有 5ms 一拍，三样挤在一起必然超时。 */
        if ((u32)(F32C_TickCount - last_send_tick) >= F32C_SEND_EVERY)
        {
            last_send_tick = F32C_TickCount;

            /* 旧变量，跟着曲线走，方便另接 DataScope 之类做波形对比 */
            Motor1_T_Position = Motor2_T_Position = F32C_Setpoint_X10;

            if (F32C_MotorID == 1 || F32C_MotorID == 2)
            {
                F32C_SendTarget(F32C_MotorID, F32C_Setpoint_X10);
            }
            else
            {
                /* 没识别出 ID（比如没接反馈线，发过去没人理而已）：
                 * 两个地址都发，谁在谁动。注意这条路每拍多一帧 + 一个
                 * delay_ms(1)，最坏一拍（再加上捎带的重发帧和反馈请求）
                 * 要 6.3ms，6.667ms 的节拍基本被吃光 —— 只是应急路径，
                 * 认不出电机时会看到目标流偶有跳拍，属正常。 */
                F32C_SendTarget(1, F32C_Setpoint_X10);
                delay_ms(1);
                F32C_SendTarget(2, F32C_Setpoint_X10);
            }

            /* 手册 4.2 要求帧间留空闲间隔（一般 ≥1ms）—— 驱动板用的是
             * DMA + 串口空闲中断断帧，两帧挨太紧会被当成一帧乱码丢掉。 */
            delay_ms(1);

            /* ---- c. 周期重发的"使能/模式/速度"：这一拍捎一帧 ---- */
            if (setup_left)
            {
                setup_left--;
                F32C_SendSetupOne();
                delay_ms(1);
            }

            /* ---- d. 反馈 50Hz 问一次就够（到位判据和显示都用它）---- */
            if (++fb_div >= F32C_FB_EVERY)
            {
                fb_div = 0;

                if (F32C_MotorID == 2)
                {
                    USART1_SEND(motor2_Feedback_data, sizeof(motor2_Feedback_data));
                }
                else
                {
                    USART1_SEND(motor1_Feedback_data, sizeof(motor1_Feedback_data));
                }
                delay_ms(1);
            }

            /* ---- e. 把刚收到的机械角度喂给运动模块（到位判据用）---- */
            F32C_UpdateFeedback();
        }
    }
}
