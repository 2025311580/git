#include "f32c_motion.h"

/* ---------- 对外状态 ---------- */
volatile F32C_State_t F32C_State     = F32C_ST_WAIT;
volatile int  F32C_Setpoint_X10      = 0;
volatile int  F32C_Goal_X10          = F32C_HOME_X10;
volatile u8   F32C_MotorID           = 0;
volatile u32  F32C_CycleCount        = 0;
volatile u32  F32C_TickCount         = 0;

/* ---------- 梯形曲线内部状态 ----------
 * 速度单位用 Q8 定点："多少个 1/256 的 0.1°" 每拍。用定点而不是浮点：
 * Cortex-M0+ 没有 FPU，中断里跑软浮点太慢，而且定点完全不丢精度。 */
typedef struct
{
    int     pos_x10;      /* 曲线当前点（就是下发的目标角度）*/
    int     goal_x10;     /* 本段终点 */
    int32_t v_q8;         /* 当前速度，Q8；单位 0.1°/拍 */
    int32_t vmax_q8;      /* 巡航速度上限 */
    int32_t acc_q8;       /* 每拍速度增量（加减速）*/
    int32_t pos_acc;      /* 位移累加器：v 是 Q8 的，得攒够 256 才进 1 个 0.1° */
    int     last_dir;     /* 上一拍的运动方向，用来发现"要反向了" */
    u8      busy;         /* 1 = 曲线还在走 */
    u8      soft_stop;    /* 1 = 只减速不前进（暂停时的软停）*/
} F32C_Profile_t;

static F32C_Profile_t  prof;

static volatile int    fb_x10      = 0;   /* 驱动板反馈的机械角度 */
static u8              use_fb      = 0;   /* 有没有可用的反馈：没接反馈线就
                                           * 不等到位判据，曲线走完即到点 */
static u16             dwell_ticks = 0;
static u16             arrive_ticks = 0;  /* 连续多少拍在到位判据内 */
static u16             wait_ticks  = 0;   /* 曲线走完后等反馈的拍数 */
static u8              auto_run    = F32C_AUTO_RUN;   /* 回零后是否自动开始往返 */

/* ---------- 单位换算 ----------
 * 速度：0.1°/s → Q8 的 0.1°/拍   v_q8 = v * (TICK_US/1e6) * 256
 * 加速度：0.1°/s² → Q8 的 0.1°/拍² a_q8 = a * (TICK_US/1e6)² * 256
 * 例：VMAX 14400（1440°/s）→ 24577，每拍 24577/256 = 96 个 0.1°（9.6°），
 *     6.667ms 一拍即 9.6/0.0066672 = 1440°/s，对得上。
 *
 * 中间量必须走 int64 先乘后除：一拍 6667µs，VMAX 那一档 28800（2880°/s）时
 * v*6667*256 = 4.9e10，早出了 int32 的 2.1e9；加速度那边 a*6667²*256 更是
 * 5.2e15。除完的结果本身不大（VMAX → 49154），装得回 int32。
 * Cortex-M0+ 没有硬件除法，64 位除要走软件例程 —— 但这两个函数只在
 * F32C_Profile_Start 里调，一秒最多几次，不在热路径上。 */
static int32_t F32C_Speed_To_Q8(int v_x10s)
{
    return (int32_t)(((int64_t)v_x10s * F32C_TICK_US * 256) / 1000000);
}

static int32_t F32C_Accel_To_Q8(int a_x10s2)
{
    return (int32_t)(((int64_t)a_x10s2 * F32C_TICK_US * F32C_TICK_US * 256)
                     / 1000000000000LL);
}

/* 启动一段新曲线。注意 pos_x10 不动 —— 曲线永远从"当前点"接着走，
 * 所以中途改目标（比如长按回零）是接上去的，不会跳。 */
static void F32C_Profile_Start(int goal_x10, int vmax_x10s, int acc_x10s2)
{
    prof.goal_x10 = goal_x10;
    prof.vmax_q8  = F32C_Speed_To_Q8(vmax_x10s);
    prof.acc_q8   = F32C_Accel_To_Q8(acc_x10s2);
    if (prof.acc_q8 < 1) prof.acc_q8 = 1;      /* 防 0：下面要拿它做除数 */
    prof.pos_acc  = 0;
    prof.soft_stop = 0;
    prof.busy     = 1;

    F32C_Goal_X10 = goal_x10;
    arrive_ticks  = 0;
    wait_ticks    = 0;
}

/* 走一拍曲线。 */
static void F32C_Profile_Step(void)
{
    int32_t rem_x10, rem_q8, stop_q8;
    int32_t step;
    int     dir;

    if (!prof.busy) return;

    rem_x10 = (int32_t)prof.goal_x10 - (int32_t)prof.pos_x10;

    /* 到了且速度收住了才算走完 */
    if (rem_x10 == 0 && prof.v_q8 == 0)
    {
        prof.busy = 0;
        return;
    }

    dir = (rem_x10 > 0) ? 1 : -1;

    /* 要反向了（例如正往 180° 走时被叫回零）：先把速度清 0，让 setpoint
     * 停一下再往反方向加速。不这么做就是从巡航空速度直接反转，机械冲击很大。 */
    if (dir != prof.last_dir)
    {
        prof.v_q8    = 0;
        prof.pos_acc = 0;
        prof.last_dir = dir;
    }

    rem_q8 = (rem_x10 < 0 ? -rem_x10 : rem_x10) << 8;

    if (prof.soft_stop)
    {
        /* 软停：只减速不前进，速度到 0 就结束（位置停在哪儿就是哪儿）*/
        prof.v_q8 -= prof.acc_q8;
        if (prof.v_q8 <= 0)
        {
            prof.v_q8      = 0;
            prof.soft_stop = 0;
            prof.busy      = 0;
            return;
        }
    }
    else
    {
        /* 还需要多少距离才刹得住：d = v²/(2a)。拿它和剩余距离比，
         * 剩余距离不够了就转成减速，这就是 T 型曲线的"提前刹车点"。 */
        stop_q8 = (int32_t)(((int64_t)prof.v_q8 * prof.v_q8) / (2 * prof.acc_q8));

        if (rem_q8 <= stop_q8)
        {
            prof.v_q8 -= prof.acc_q8;
            if (prof.v_q8 < 0) prof.v_q8 = 0;
        }
        else
        {
            prof.v_q8 += prof.acc_q8;
            if (prof.v_q8 > prof.vmax_q8) prof.v_q8 = prof.vmax_q8;
        }
    }

    /* 把 Q8 速度累加成一个一个 0.1°，余数留到下一拍，不丢精度 */
    prof.pos_acc += prof.v_q8;
    step = prof.pos_acc >> 8;
    prof.pos_acc -= step << 8;

    if (step > (rem_q8 >> 8)) step = rem_q8 >> 8;   /* 不许冲过终点 */

    prof.pos_x10 += dir * (int)step;
    F32C_Setpoint_X10 = prof.pos_x10;
}

/* 反馈角度到位没有（在判据内连续若干拍才算数，避开单帧抖动）。 */
static u8 F32C_Endpoint_Reached(void)
{
    int err = fb_x10 - prof.goal_x10;

    if (err < 0) err = -err;

    /* 本工程行程 0~180°，不会跨 0°/360° 边界，所以不用处理环绕。 */
    if (err <= F32C_ARRIVE_X10)
    {
        if (arrive_ticks < F32C_ARRIVE_TICKS) arrive_ticks++;
    }
    else
    {
        arrive_ticks = 0;
    }

    return (arrive_ticks >= F32C_ARRIVE_TICKS) ? 1 : 0;
}

void F32C_Motion_Begin(u8 motor_id, int start_x10)
{
    /* 归一化到 0~3599（单圈绝对角度的量程）*/
    start_x10 %= 3600;
    if (start_x10 < 0) start_x10 += 3600;

    F32C_MotorID      = motor_id;
    use_fb            = (motor_id != 0) ? 1 : 0;
    prof.pos_x10      = start_x10;
    prof.last_dir     = 0;
    prof.v_q8         = 0;
    prof.pos_acc      = 0;
    prof.busy         = 0;
    prof.soft_stop    = 0;
    F32C_Setpoint_X10 = start_x10;
    F32C_CycleCount   = 0;
    fb_x10            = start_x10;
    arrive_ticks      = 0;
    wait_ticks        = 0;
    dwell_ticks       = 0;
    auto_run          = F32C_AUTO_RUN;

    /* 第 1 段：回零。用较慢的一组参数，上电不至于猛冲。 */
    F32C_Profile_Start(F32C_HOME_X10, F32C_HOME_VMAX_X10S, F32C_HOME_ACC_X10S2);
    F32C_State = F32C_ST_HOMING;
}

void F32C_Motion_Feedback(int fb_in_x10)
{
    fb_in_x10 %= 3600;
    if (fb_in_x10 < 0) fb_in_x10 += 3600;

    fb_x10 = fb_in_x10;
}

void F32C_Motion_Tick(void)
{
    F32C_TickCount++;

    switch (F32C_State)
    {
    case F32C_ST_HOMING:
    case F32C_ST_MOVING:
        if (prof.busy)
        {
            F32C_Profile_Step();
            wait_ticks = 0;
        }
        else
        {
            /* 曲线走完了，等驱动板真正停到终点上再算到位 */
            if (wait_ticks < 0xFFFF) wait_ticks++;

            /* 没接反馈线（识别电机时没收到任何回帧）时别等到位判据 ——
             * 反馈恒为 0，永远等不到，只会白白耗掉兜底超时。曲线本来就
             * 精确落在目标上，走完即算到点。 */
            if (!use_fb ||
                F32C_Endpoint_Reached() ||
                (wait_ticks >= F32C_MS_TO_TICKS(F32C_ARRIVE_TIMEOUT_MS)))
            {
                if (F32C_State == F32C_ST_MOVING && prof.goal_x10 == F32C_HOME_X10)
                {
                    F32C_CycleCount++;   /* 回到 0° 了，算一个完整往返 */
                }

                /* 回零完且不想自动跑（auto_run=0）就停在 0° 等着 */
                dwell_ticks = 0;
                F32C_State  = auto_run ? F32C_ST_DWELL : F32C_ST_HOLD;
            }
        }
        break;

    case F32C_ST_DWELL:
        /* 端点停留结束 → 走另一头 */
        if (++dwell_ticks >= F32C_MS_TO_TICKS(F32C_DWELL_MS))
        {
            int next = (prof.goal_x10 == F32C_HOME_X10) ? F32C_GOAL_X10 : F32C_HOME_X10;

            dwell_ticks = 0;
            F32C_Profile_Start(next, F32C_VMAX_X10S, F32C_ACC_X10S2);
            F32C_State = F32C_ST_MOVING;
        }
        break;

    default:
        /* WAIT / HOLD：什么都不做。setpoint 不再变化，驱动板的位置环
         * 会把电机锁在最后那个角度上。 */
        break;
    }
}

void F32C_Motion_ToggleRun(void)
{
    if (F32C_State == F32C_ST_WAIT) return;      /* 还没回零，先不管按键 */

    if (F32C_State == F32C_ST_HOLD)
    {
        auto_run    = 1;
        dwell_ticks = 0;
        F32C_State  = F32C_ST_DWELL;             /* 从当前端点继续往返 */
    }
    else
    {
        /* 暂停：不是直接冻住 —— 巡航中冻住 setpoint 等于让驱动板急停，
         * 手册明确说高速下突然堵转可能反充烧驱动板。这里让曲线自己
         * 按原加速度减速到 0，电机是"刹住"的，停在哪算哪。 */
        auto_run       = 0;
        prof.soft_stop = 1;
        arrive_ticks   = 0;
    }
}

void F32C_Motion_GoHome(void)
{
    if (F32C_State == F32C_ST_WAIT) return;

    auto_run = 0;
    /* 从当前位置接着走向 0°（不重置 pos_x10），速度也接着来 */
    F32C_Profile_Start(F32C_HOME_X10, F32C_HOME_VMAX_X10S, F32C_HOME_ACC_X10S2);
    F32C_State = F32C_ST_HOMING;
}

const char *F32C_StateText(void)
{
    switch (F32C_State)
    {
    case F32C_ST_HOMING: return "HOME ";
    case F32C_ST_DWELL:  return "DWELL";
    case F32C_ST_MOVING: return "RUN  ";
    case F32C_ST_HOLD:   return "HOLD ";
    default:             return "WAIT ";
    }
}
