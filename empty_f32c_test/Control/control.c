#include "board.h"
#include "f32c_motion.h"

/* TIMG0 周期中断（6.667ms，150Hz）：只推进运动曲线。
 *
 * 曲线放中断里跑而不是主循环里：主循环要按 150Hz 收发串口，帧与帧之间还有
 * 时序要求，曲线挤在里面会被串口操作拖得一顿一顿的。中断每 6.667ms 雷打不动
 * 跑一次，setpoint 的变化就是均匀的。
 *
 * 这里原来还有按键扫描（S1：短按暂停 / 长按回零），已去掉 —— 现在只接了一个
 * 电机，没有按键。运动模块里的 F32C_Motion_ToggleRun / F32C_Motion_GoHome
 * 两个接口都留着，以后要接按键或者遥控直接调就行。 */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        LED_Flash(100);          /* 心跳：500ms 翻一次，没显示时靠它看程序活着 */

        F32C_Motion_Tick();
    }
}
