/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "gy87.h"
#include "Madgwick.h"
#include "i2c_adapter.h"
#include "fc_time.h"
#include "atomic_publish.h"
#include "calib.h"
#include "safety.h"
#include "usart.h"
#include "anonym.h"
#include "motor.h"
#include "pid.h"
#include "rc.h"
#include "altitude.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* StatusTask 采集到的诊断量 */
static volatile uint32_t s_heap_free    = 0u;   /* 剩余堆空间（字节） */
static volatile uint32_t s_wm_ctrl      = 0u;   /* 控制任务栈水位 */
static volatile uint32_t s_wm_att       = 0u;   /* 姿态任务栈水位 */
static volatile uint32_t s_wm_safety    = 0u;   /* 安全任务栈水位 */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ==========================================================================
 * 周期
 * ========================================================================== */
#define ATT_WAIT_TIMEOUT_MS  10u  /* 姿态数据等待超时 */
#define SAFETY_PERIOD_MS     10u  /* 安全监控周期 */
#define BUS_PERIOD_MS        1u   /* 总线收发周期 */
#define DISPLAY_PERIOD_MS    500u /* 显示刷新周期 */
#define STATUS_PERIOD_MS     200u /* 状态上报周期 */
#define RC_PERIOD_MS         5u   /* 遥控解析周期 */

/* RC 失联自动缓降斜率：ControlTask 2 ms/拍，每拍递减 1 µs ≈ 500 µs/s。
 * 太快坠落偏快，太慢空中滞留过久；按实机整定。 */
#define FAILSAFE_SLEW_US     1u

/* CMSIS-RTOS2 的 osDelayUntil 在目标节拍已过时立即返回（不延时），
 * 因此“wake += period”一旦落后就永远追不上，会退化成忙循环。
 * 本函数在落后时重新对齐到“当前 + 周期”，保证每拍真正睡眠。 */
static inline void PeriodWait(uint32_t *wake, uint32_t period)
{
	if ((int32_t)(osKernelGetTickCount() - *wake) > 0) {
		*wake = osKernelGetTickCount() + period;
	}
	(void)osDelayUntil(*wake);
	*wake += period;
}

/* 任务间通知标志 */
#define ATT_FLAG_NEW_SAMPLE   (1u << 0)

/* ==========================================================================
 * 姿态角目标限幅
 * ========================================================================== */
/* 绝对硬限：滚转/俯仰目标一律不超过 ±40°，所有控制来源统一生效。
 * 物理依据：倾角 θ 时维持高度需要 1/cosθ 倍悬停推力；45° 需约 1.41 倍，
 * 本机油门余量不足，45° 已无法拉回（安全层 45° 判 FATAL）。
 * 因此目标硬限压到 40°（约 1.31 倍），给致命阈值留 5° 余量。 */
#define ATT_TILT_HARD_DEG    40.0f
/* 摇杆满舵映射：±30°。常规航拍/调参够用，并给硬限留 10° 余量，
 * 避免杆量打满时直接贴着极限飞行。 */
#define ATT_TILT_STICK_DEG   30.0f

/* ==========================================================================
 * CH5 三档传感器上限（aux_us[0]）：与定高辅助无关，始终参与编译
 * ========================================================================== */
/* 滞回：±200µs 进入 / ±100µs 退出（µs 域） */
#define CH5_HI_TH        1700u
#define CH5_HI_REL_TH    1600u
#define CH5_LO_TH        1300u
#define CH5_LO_REL_TH    1400u

/* ==========================================================================
 * CH7 两档开关（aux_us[2]）：电机锁 / 一键起飞触发 —— 始终参与编译
 * --------------------------------------------------------------------------
 * 两种模式下都是"拨高才允许电机转"的硬开关：
 *  - FEATURE_ALT_ASSIST 开：拨高 = 解锁并一键起飞；拨低 = 缓降/撤销解锁
 *  - FEATURE_ALT_ASSIST 关：拨高 = 仅解锁（手动油门飞行）；拨低 = 立即上锁
 * 上电/链路恢复时必须实际经历"低→高"边沿才有效：开关若一直停在 H，
 * 只锁存为基准电平而不触发解锁（见 ControlTask 2c 消抖逻辑）。
 * 阈值用滞回 + 范围判定，不依赖精确的 1000/2000µs。
 * ========================================================================== */
#define CH7_HI_TH        1650u
#define CH7_LO_TH        1350u
#define CH7_DEBOUNCE     50u

/* 油门斜变 ±16µs/拍，全行程约 125ms（两种模式共用） */
#define BASE_SLEW_US     16u

/* ==========================================================================
 * 定高辅助 / 一键起飞（CH6 目标高度 + 自动起降状态机）—— 编译期可选
 * --------------------------------------------------------------------------
 * 由 FEATURE_ALT_ASSIST（control_types.h）决定是否参与编译。
 * 未定义时：本节 CH6 与高度环/状态机全部不编译，CH3 油门按标准全行程直通。
 * ========================================================================== */
#ifdef FEATURE_ALT_ASSIST
/* CH6 一键起飞高度：aux_us[1] 线性映射 1.8~3.5 m，CH7 边沿瞬间锁存 */
#define TKOF_ALT_MIN_M   1.8f
#define TKOF_ALT_MAX_M   3.5f
#define TKOF_ALT_MID_M   2.65f
#define TKOF_ALT_HALF_M  0.85f

/* 高度环：base_01 ∈ hover_ff ± ALT_OUT_DELTA
 * P/I/D 增益走 PID 参数 ID 2018~2020，上位机 PID 页在线调整，默认值见 Anonym_Init */
#define ALT_I_LIM        0.15f
#define ALT_OUT_DELTA    0.30f
#define HOVER_FF_INIT    0.45f      /* 无桨台架保守常数，FLARE 落地瞬间学习 */
#define HOVER_FF_MIN     0.20f
#define HOVER_FF_MAX     0.80f

/* 垂直速率指令与目标高度包络 */
#define VZ_DEADBAND      0.08f      /* 油门杆归一化死区（±80µs），回中=定高 */
#define VZ_STICK_MAX     1.0f       /* 满杆最大爬升/下降速率 m/s */
#define VZ_TAKEOFF       0.6f
#define VZ_LAND          0.4f
#define VZ_LAND_SLOW     0.2f       /* 0.5m 以下末段下降率 */
#define ALT_TGT_FLOOR_M  0.2f       /* 地面守卫：HOLD 目标 ≤ 此值 = 怠速 */
#define ALT_TGT_CEIL_M   3.8f       /* 低于超声 4.0m 量程 */

/* 状态迁移判据 */
#define TKOF_DONE_ERR_M  0.15f
#define TKOF_DONE_VZ     0.25f
#define TKOF_DONE_MS     1000u
#define TKOF_TIMEOUT_MS  15000u     /* 起飞超时：推力不足无法到位 → 转缓降 */
#define LAND_TOUCH_ALT_M 0.25f
#define LAND_TOUCH_MS    600u
#define LAND_FADE_MS     400u       /* 落地油门淡出 */
#define LAND_TIMEOUT_MS  30000u     /* 缓降超时强制淡出 */
#define ARM_WAIT_MS      1000u      /* CH7 边沿后等 Safety 确认解锁 */

typedef enum {
    ASSIST_MANUAL = 0,   /* 油门杆直通（回中摇杆：仅上杆映射） */
    ASSIST_ARM_WAIT,     /* 已发 RC 解锁请求，等 Safety 确认 */
    ASSIST_TAKEOFF,      /* 爬升到锁存高度 */
    ASSIST_HOLD,         /* 定高，杆 = 高度速率指令 */
    ASSIST_LAND,         /* 缓降 */
    ASSIST_FLARE,        /* 落地油门淡出 + 自动上锁 */
} AssistState;
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for AttitudeTask */
osThreadId_t AttitudeTaskHandle;
const osThreadAttr_t AttitudeTask_attributes = {
  .name = "AttitudeTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for RcTask */
osThreadId_t RcTaskHandle;
const osThreadAttr_t RcTask_attributes = {
  .name = "RcTask",
  .stack_size = 320 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal2,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 320 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StatusTask */
osThreadId_t StatusTaskHandle;
const osThreadAttr_t StatusTask_attributes = {
  .name = "StatusTask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for SafetyTask */
osThreadId_t SafetyTaskHandle;
const osThreadAttr_t SafetyTask_attributes = {
  .name = "SafetyTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal4,
};
/* Definitions for BusTask */
osThreadId_t BusTaskHandle;
const osThreadAttr_t BusTask_attributes = {
  .name = "BusTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Control_Task(void *argument);
void Attitude_Task(void *argument);
void Rc_Task(void *argument);
void Comm_Task(void *argument);
void Status_Task(void *argument);
void Display_Task(void *argument);
void Safety_Task(void *argument);
void Bus_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
   (void)xTask;
   (void)pcTaskName;
   __disable_irq();
   for (;;) { }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
   __disable_irq();
   for (;;) { }
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	Anonym_TxInit();   /* 创建 TX 块池与两个队列，必须在 osKernelInitialize 之后 */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(Control_Task, NULL, &ControlTask_attributes);

  /* creation of AttitudeTask */
  AttitudeTaskHandle = osThreadNew(Attitude_Task, NULL, &AttitudeTask_attributes);

  /* creation of RcTask */
  RcTaskHandle = osThreadNew(Rc_Task, NULL, &RcTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(Comm_Task, NULL, &CommTask_attributes);

  /* creation of StatusTask */
  StatusTaskHandle = osThreadNew(Status_Task, NULL, &StatusTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(Display_Task, NULL, &DisplayTask_attributes);

  /* creation of SafetyTask */
  SafetyTaskHandle = osThreadNew(Safety_Task, NULL, &SafetyTask_attributes);

  /* creation of BusTask */
  BusTaskHandle = osThreadNew(Bus_Task, NULL, &BusTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Control_Task */
/**
  * @brief  Function implementing the ControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Control_Task */
void Control_Task(void *argument)
{
  /* USER CODE BEGIN Control_Task */
	
	(void)argument;

	/* ---- PID 实例 ---- */
	Pid pid_roll_rate, pid_pitch_rate, pid_yaw_rate;
	Pid pid_roll_att,  pid_pitch_att,  pid_yaw_att;

	/* 参数 ID 映射（与 AutoPID 上位机一致：2000 起，6 组 × P/I/D）：
	 * 2000-2002: Roll Att  P I D   2003-2005: Pitch Att  P I D
	 * 2006-2008: Yaw Att   P I D   2009-2011: Roll Rate  P I D
	 * 2012-2014: Pitch Rate P I D   2015-2017: Yaw Rate  P I D
	 *
	 * 单位约定：外环输入/误差全程用 deg，PID 输出 deg/s，送内环前显式
	 * ×0.0174533 转 rad/s；内环输入 rad/s，输出 ±1。
	 * 注意：由旧"输出即 rad/s"约定切换后，上位机 2000~2008 号外环增益需 ×57.3 重标。
	 *
	 * 限幅（先给保守值，台架按需放宽）：
	 *   内环输出 ±1；外环 roll/pitch ±344°/s（=6 rad/s）、yaw ±86°/s（=1.5 rad/s） */
	Pid_Init(&pid_roll_rate,  Anon_GetPidGain(2009u), Anon_GetPidGain(2010u), Anon_GetPidGain(2011u),
					 0.3f, 1.0f, 0.3f);
	Pid_Init(&pid_pitch_rate, Anon_GetPidGain(2012u), Anon_GetPidGain(2013u), Anon_GetPidGain(2014u),
					 0.3f, 1.0f, 0.3f);
	Pid_Init(&pid_yaw_rate,   Anon_GetPidGain(2015u), Anon_GetPidGain(2016u), Anon_GetPidGain(2017u),
					 0.2f, 1.0f, 0.5f);
	Pid_Init(&pid_roll_att,   Anon_GetPidGain(2000u), Anon_GetPidGain(2001u), Anon_GetPidGain(2002u),
					 28.6f, 344.0f, 1.0f);   /* i_lim/out_lim 单位 deg/s */
	Pid_Init(&pid_pitch_att,  Anon_GetPidGain(2003u), Anon_GetPidGain(2004u), Anon_GetPidGain(2005u),
					 28.6f, 344.0f, 1.0f);
	/* yaw 角度环：磁力计抖动风险高，输出限幅保守、D 低通压尖峰 */
	Pid_Init(&pid_yaw_att,    Anon_GetPidGain(2006u), Anon_GetPidGain(2007u), Anon_GetPidGain(2008u),
					 20.0f, 86.0f, 0.2f);

	uint32_t last_us = 0u;
	uint32_t tick = 0u;
	uint32_t wake = osKernelGetTickCount() + 1u;

	/* 内环 500Hz；外环由新姿态事件驱动（实际频率 = 姿态更新率 200Hz），
	 * 用上一帧姿态的 seq/usec 判新帧与差分 dt */
	float tgt_gx = 0.0f, tgt_gy = 0.0f, tgt_gz = 0.0f;
	fc_seq_t  	last_att_seq  = 0u;   /* 上次外环消费的姿态快照序列号 */
	fc_usec_t  	last_att_usec = 0u;   /* 上次外环消费的姿态快照时刻，用于 dt 差分 */
	uint32_t  	att_loss_cnt  = 0u;   /* 姿态连续丢失拍数（2ms/拍），超过才清零外环 */
	bool      	yaw_hold_active = false;  /* yaw 角度保持模式（磁力计可信时置位） */
	float     	yaw_hold_deg    = 0.0f;   /* 保持的目标航向，单位 deg */

	uint8_t   ch5_pos     = 2u;      /* 传感器上限档位：0=GYRO 1=IMU 2=MARG（CH5，始终参与） */
	uint16_t  prev_thr_us    = MOTOR_US_MIN;    /* 上一拍油门输出（斜变状态） */
	/* CH7 两档开关消抖状态（两种模式共用） */
	bool      ch7_lvl      = false;  /* 消抖后的稳定电平 */
	bool      ch7_synced   = false;  /* 首帧/链路恢复后已同步基准电平（不认作边沿） */
	bool      ch7_low_seen = false;  /* 已观测到低位：只有低→高才算有效解锁沿 */
	uint8_t   ch7_cnt      = 0u;     /* 候选电平持续拍数 */
#ifdef FEATURE_ALT_ASSIST
	/* ---- 定高辅助 / 一键起飞状态（CH6） ---- */
	AssistState assist_state = ASSIST_MANUAL;
	bool      ch7_session = false;   /* CH7=H 辅助会话 */
	float     tko_alt_m   = 0.0f;    /* 锁存的一键起飞目标高度 m */
	float     h_tgt_m     = 0.0f;    /* 高度环当前目标高度 m */
	float     vz_cmd      = 0.0f;    /* 垂直速率指令 m/s */
	float     hover_ff    = HOVER_FF_INIT;  /* 悬停前馈（FLARE 学习，仅 RAM） */
	float     alt_i       = 0.0f;    /* 高度环积分 */
	float     alt_kp      = Anon_GetPidGain(2018u);  /* 高度环 P（ID 2018） */
	float     alt_ki      = Anon_GetPidGain(2019u);  /* 高度环 I（ID 2019） */
	float     alt_kd      = Anon_GetPidGain(2020u);  /* 高度环 D（ID 2020） */
	float     base_01     = 0.0f;    /* 辅助层基础油门 [0,1] */
	fc_seq_t  last_alt_seq  = 0u;    /* 上次高度环消费的高度快照 */
	uint32_t  last_alt_usec = 0u;
	uint8_t   abort_reason   = ALT_ABORT_NONE;  /* 粘滞到下一次 CH7 边沿 */
	uint32_t  state_t0       = 0u;   /* 状态进入时刻（拍） */
	uint32_t  touch_cnt      = 0u;   /* 落地检测连续拍数 */
	uint32_t  assist_att_cnt = 0u;   /* 辅助态姿态丢失拍数 */
	char      lbuf[32];              /* LOG 格式化缓冲 */
#endif

  /* Infinite loop */


	for (;;) {
		PeriodWait(&wake, 2u);
		tick++;

		const uint32_t now = Fc_GetMicros();
		float dt = (last_us == 0u) ? 0.001f : (float)(now - last_us) * 1e-6f;
		last_us = now;
		if (dt > 0.03f) { dt = 0.03f; }   /* 抖大时夹紧，防积分爆冲 */

		/* ---- 1. 读输入 ---- */
		SensorData       sen;
		AttitudeSnapshot att;
		const bool sen_ok = FcChan_Read(&g_chan_sensor,   &sen) && sen.valid;
		const bool att_ok = FcChan_Read(&g_chan_attitude, &att) && att.valid;
#ifdef FEATURE_ALT_ASSIST
		AltitudeSnapshot alt;
		const bool alt_ok = FcChan_Read(&g_chan_altitude, &alt) && alt.valid;
#endif

		AnonDbgCtrl dbg;
		Anon_GetDbgCtrl(&dbg);
		const bool dbg_mode = (dbg.mode != 0);   /* 非 0 即接管 */

		RcCommandSnapshot rc_snap;
		RcNorm rcn;
		const bool rc_ok = FcChan_Read(&g_chan_rc, &rc_snap)
                && Rc_Normalize(&rc_snap, &rcn);

		/* ---- 2. 目标量决策（DBG 优先，其次 RC，最后 idle） ---- */
		float    tgt_roll_deg  = 0.0f;
		float    tgt_pitch_deg = 0.0f;
		float    tgt_yaw_rate  = 0.0f;   /* rad/s */
		uint16_t thr_us        = MOTOR_US_MIN;
		bool     control_on    = false;

		if (dbg_mode) {
			tgt_roll_deg  = (float)dbg.tgt_roll_centi_deg  / 100.0f;
			tgt_pitch_deg = (float)dbg.tgt_pitch_centi_deg / 100.0f;
			tgt_yaw_rate  = (float)dbg.tgt_yaw_rate_centi_dps / 100.0f * 0.0174533f;
			thr_us        = (uint16_t)dbg.throttle_us;
			control_on    = true;
		} else if (rc_ok) {
			/* 摇杆 → 目标姿态（满舵 ±ATT_TILT_STICK_DEG）、偏航速率（±90°/s） */
			tgt_roll_deg  = rcn.roll  * ATT_TILT_STICK_DEG;
			tgt_pitch_deg = -rcn.pitch * ATT_TILT_STICK_DEG;
			tgt_yaw_rate  = rcn.yaw * 90.0f * 0.0174533f;
#ifdef FEATURE_ALT_ASSIST
			/* 回中油门杆手动直通：仅用上半程（回中=怠速，满上=2000µs）；
			 * 定高辅助可用时本值被 2f 接管覆盖 */
			float t = rcn.thr - 0.5f;
			if (t < 0.0f) { t = 0.0f; }
			thr_us        = (uint16_t)(1000.0f + t * 2000.0f);
#else
			/* 标准油门直通：CH3 全行程 1000~2000µs → 1000~2000µs */
			thr_us        = (uint16_t)(1000.0f + rcn.thr * 1000.0f);
#endif
			control_on    = true;
		} else {
			/* RC 失联且非调试：
			 *  - Safety 仍允许输出（ARMED 未超时 或 FAILSAFE）→ 水平自稳 + 油门缓降着陆；
			 *  - Safety 已失锁 → 目标全零、油门最低。 */
			if (Safety_IsAllowed()) {
				tgt_roll_deg  = 0.0f;          /* 水平自稳 */
				tgt_pitch_deg = 0.0f;
				tgt_yaw_rate  = 0.0f;
				/* 缓降：每拍递减 FAILSAFE_SLEW_US，钳到最低油门，飞机缓慢下降 */
				thr_us = (prev_thr_us > (MOTOR_US_MIN + FAILSAFE_SLEW_US))
				       ? (uint16_t)(prev_thr_us - FAILSAFE_SLEW_US)
				       : MOTOR_US_MIN;
				control_on    = true;
			} else {
				thr_us        = MOTOR_US_MIN;
				control_on    = false;
			}
		}

		/* ---- 2a. 姿态角硬限幅（所有来源统一：DBG / RC / 未来的 0x41） ---- */
		tgt_roll_deg  = FC_SAT(tgt_roll_deg,  -ATT_TILT_HARD_DEG, ATT_TILT_HARD_DEG);
		tgt_pitch_deg = FC_SAT(tgt_pitch_deg, -ATT_TILT_HARD_DEG, ATT_TILT_HARD_DEG);

		/* ---- 2b. CH5 三档传感器上限（滞回消抖；RC 丢失保持最后档位） ---- */
		if (rc_ok) {
			uint8_t pos = ch5_pos;
			if (pos == 2u)      { if (rcn.aux_us[0] < CH5_HI_REL_TH) { pos = 1u; } }
			else if (pos == 0u) { if (rcn.aux_us[0] > CH5_LO_REL_TH) { pos = 1u; } }
			else {
				if (rcn.aux_us[0] > CH5_HI_TH)       { pos = 2u; }
				else if (rcn.aux_us[0] < CH5_LO_TH)  { pos = 0u; }
			}
			if (pos != ch5_pos) {
				static const char *const mode_name[3] = {"SENS:GYRO", "SENS:IMU6", "SENS:MARG9"};
				ch5_pos = pos;
				Attitude_SetSensorMode((SensorMode)pos);
				AnonLog(0u, mode_name[pos]);
			}
		}

		/* ---- 2c. CH7 两档开关消抖边沿检测（两种模式共用） ---- */
		/* 边沿标志仅本拍有效：防止解锁被拒后 CH7 仍停在 H 时跨拍重复触发 */
		bool ch7_rise = false;
		bool ch7_fall = false;
		if (!rc_ok) {
			ch7_synced   = false;   /* 链路丢失：恢复后以当前档位重新同步基准 */
			ch7_low_seen = false;   /* 恢复后必须重新经历低位才允许解锁 */
			ch7_cnt      = 0u;
		}
		bool cand = ch7_lvl;
		if (rc_ok) {
			if (!ch7_synced) {
				/* 上电/恢复首帧：直接锁存当前档位为基准电平，不产生边沿，
				 * 防止开机时 CH7 已停在 H 被合成成 L→H 边沿误触发解锁 */
				ch7_lvl      = (rcn.aux_us[2] > CH7_HI_TH);
				ch7_low_seen = (rcn.aux_us[2] < CH7_LO_TH);
				ch7_synced   = true;
				ch7_cnt      = 0u;
			} else {
				if (rcn.aux_us[2] > CH7_HI_TH)      { cand = true;  }
				else if (rcn.aux_us[2] < CH7_LO_TH) { cand = false; }
				/* 见过低位才记录：解锁沿必须来自"低→高"，上电即为高不算 */
				if (rcn.aux_us[2] < CH7_LO_TH)      { ch7_low_seen = true; }
			}
		}
		if (ch7_synced && cand != ch7_lvl) {
			if (++ch7_cnt >= CH7_DEBOUNCE) {
				ch7_lvl  = cand;
				ch7_cnt  = 0u;
				ch7_rise = cand && ch7_low_seen;   /* 无低位经历的高电平不产生解锁沿 */
				ch7_fall = !cand;
				if (ch7_rise) { ch7_low_seen = false; }  /* 一次低位只换一次解锁 */
			}
		} else {
			ch7_cnt = 0u;
		}

		bool assist_on = false;   /* 仅辅助模式会置 true；两种配置下都需声明 */
#ifdef FEATURE_ALT_ASSIST
		/* ---- 2d. 定高辅助状态机（DBG 接管时整体旁路） ---- */
		if (dbg_mode) {
			if (assist_state != ASSIST_MANUAL) {
				assist_state = ASSIST_MANUAL;
				ch7_session  = false;
				Safety_RequestArmRc(false);
				abort_reason = ALT_ABORT_BENCH;
			}
		} else {
			const bool armed = Safety_IsArmed();

			/* 会话仍在但 Safety 已复锁（上位机一票否决/链路丢失）：退出辅助 */
			if (ch7_session && !armed && (assist_state != ASSIST_ARM_WAIT)) {
				ch7_session  = false;
				assist_state = ASSIST_MANUAL;
				Safety_RequestArmRc(false);
				if (abort_reason == ALT_ABORT_NONE) { abort_reason = ALT_ABORT_HOST; }
			}

			if (ch7_rise && !ch7_session) {
				/* L→H：锁存 CH6 目标高度；未解锁则发 RC 解锁请求，条件由 Safety 裁决 */
				tko_alt_m = FC_SAT(TKOF_ALT_MID_M
				                   + ((float)rcn.aux_us[1] - 1500.0f) / 500.0f * TKOF_ALT_HALF_M,
				                   TKOF_ALT_MIN_M, TKOF_ALT_MAX_M);
				h_tgt_m      = alt_ok ? alt.height_m : 0.0f;
				base_01      = (float)(prev_thr_us - MOTOR_US_MIN) / 1000.0f;
				alt_i        = 0.0f;
				abort_reason = ALT_ABORT_NONE;
				ch7_session  = true;
				state_t0     = tick;
				assist_state = armed ? ASSIST_TAKEOFF : ASSIST_ARM_WAIT;
				if (!armed) { Safety_RequestArmRc(true); }
				const int cm = (int)(tko_alt_m * 100.0f + 0.5f);
				(void)snprintf(lbuf, sizeof(lbuf), "CH7 TKOF TGT %d.%02dm", cm / 100, cm % 100);
				AnonLog(2u, lbuf);
			} else if (ch7_fall && ch7_session) {
				if (assist_state == ASSIST_TAKEOFF || assist_state == ASSIST_HOLD) {
					assist_state = ASSIST_LAND;      /* 正常缓降流程，非故障中止 */
					state_t0     = tick;
					abort_reason = ALT_ABORT_CH7_LOW;
					AnonLog(0u, "CH7: LAND START");
				} else if (assist_state == ASSIST_ARM_WAIT) {
					Safety_RequestArmRc(false);
					ch7_session  = false;
					assist_state = ASSIST_MANUAL;
				}
			}

			switch (assist_state) {
			case ASSIST_ARM_WAIT:
				if (armed) {
					assist_state = ASSIST_TAKEOFF;
					state_t0     = tick;
				} else if ((tick - state_t0) * 2u >= ARM_WAIT_MS) {
					/* 解锁前置条件不满足，Safety 拒绝 */
					Safety_RequestArmRc(false);
					ch7_session  = false;
					assist_state = ASSIST_MANUAL;
					abort_reason = ALT_ABORT_PRECOND;
					AnonLog(1u, "CH7: ARM REJECTED");
				}
				break;

			case ASSIST_TAKEOFF:
			case ASSIST_HOLD:
			case ASSIST_LAND:
				assist_on = true;
				/* 姿态丢失 >50ms：退出辅助（Safety 也会很快复锁） */
				if (!att_ok) {
					if (++assist_att_cnt > 25u) {
						assist_att_cnt = 0u;
						ch7_session  = false;
						assist_state = ASSIST_MANUAL;
						Safety_RequestArmRc(false);
						abort_reason = ALT_ABORT_ATT_LOSS;
						assist_on    = false;
						AnonLog(1u, "ASSIST: ATT LOST");
						break;
					}
				} else {
					assist_att_cnt = 0u;
				}
				/* 高度数据丢失：退出辅助，退回降级手动油门（经斜变无跳变） */
				if (!alt_ok) {
					ch7_session  = false;
					assist_state = ASSIST_MANUAL;
					Safety_RequestArmRc(false);
					abort_reason = ALT_ABORT_ALT_LOSS;
					assist_on    = false;
					AnonLog(1u, "ASSIST: ALT LOST");
					break;
				}
				if (assist_state == ASSIST_TAKEOFF) {
					vz_cmd = VZ_TAKEOFF;
					if ((alt.height_m >= tko_alt_m - TKOF_DONE_ERR_M)
						&& (fabsf(alt.vz_mps) < TKOF_DONE_VZ)
						&& ((tick - state_t0) * 2u >= TKOF_DONE_MS)) {
						assist_state = ASSIST_HOLD;
						h_tgt_m      = alt.height_m;   /* 到位即就地定高 */
					} else if ((tick - state_t0) * 2u >= TKOF_TIMEOUT_MS) {
						/* 推力不足迟迟不到位：转缓降，不无限顶油门 */
						assist_state = ASSIST_LAND;
						state_t0     = tick;
						AnonLog(1u, "TKOF TIMEOUT -> LAND");
					}
				} else if (assist_state == ASSIST_HOLD) {
					/* 回中油门杆：回中=定高，推/拉=高度速率指令（±80µs 死区）；
					 * RC 丢失时 rcn 无效，必须置 0 保持当前高度 */
					if (!rc_ok) { vz_cmd = 0.0f; break; }
					float t = rcn.thr - 0.5f;
					if ((t > -VZ_DEADBAND) && (t < VZ_DEADBAND)) { t = 0.0f; }
					else { t -= (t > 0.0f) ? VZ_DEADBAND : -VZ_DEADBAND; }
					vz_cmd = FC_SAT(t / (0.5f - VZ_DEADBAND), -1.0f, 1.0f) * VZ_STICK_MAX;
				} else {   /* ASSIST_LAND：缓降，末段减速 */
					vz_cmd = (alt.height_m > 0.5f) ? -VZ_LAND : -VZ_LAND_SLOW;
					if (alt.height_m < LAND_TOUCH_ALT_M) {
						if (++touch_cnt >= (uint32_t)(LAND_TOUCH_MS / 2u)) {
							assist_state = ASSIST_FLARE;
							state_t0     = tick;
							/* 落地瞬间学习悬停点（仅 RAM，不自动落盘） */
							hover_ff = FC_SAT(base_01, HOVER_FF_MIN, HOVER_FF_MAX);
							alt_i    = 0.0f;
							AnonLog(2u, "TOUCHDOWN: FLARE");
						}
					} else {
						touch_cnt = 0u;
					}
					if ((tick - state_t0) * 2u >= LAND_TIMEOUT_MS) {
						assist_state = ASSIST_FLARE;
						state_t0     = tick;
						AnonLog(1u, "LAND TIMEOUT");
					}
				}
				break;

			case ASSIST_FLARE:
				assist_on = true;
				base_01 = FC_SAT(base_01 - (2.0f / (float)LAND_FADE_MS), 0.0f, 1.0f);
				if ((tick - state_t0) * 2u >= LAND_FADE_MS) {
					Safety_RequestArmRc(false);
					ch7_session  = false;
					assist_state = ASSIST_MANUAL;
					assist_on    = false;
					AnonLog(2u, "LANDED: AUTO DISARM");
				}
				break;

			default:   /* ASSIST_MANUAL：油门杆直通 */
				break;
			}
		}

		/* ---- 2e. 高度环（新高度帧驱动 50Hz，事件式同外环） ---- */
		if (assist_on && (assist_state != ASSIST_FLARE) && alt_ok
			&& alt.stamp.seq != last_alt_seq) {
			float dt_a = (last_alt_usec == 0u) ? 0.02f
			                     : (float)(alt.stamp.usec - last_alt_usec) * 1e-6f;
			if (dt_a <= 0.0f) { dt_a = 0.02f; }
			if (dt_a >  0.1f) { dt_a = 0.1f;  }
			last_alt_seq  = alt.stamp.seq;
			last_alt_usec = alt.stamp.usec;

			/* 目标高度斜坡：TAKEOFF 爬向锁存值，HOLD/LAND 按速率指令积分 */
			h_tgt_m += (assist_state == ASSIST_TAKEOFF) ? (VZ_TAKEOFF * dt_a)
			                                            : (vz_cmd * dt_a);
			float ceil_m = ALT_TGT_CEIL_M;
			if (assist_state == ASSIST_TAKEOFF && tko_alt_m < ceil_m) { ceil_m = tko_alt_m; }
			h_tgt_m = FC_SAT(h_tgt_m, ALT_TGT_FLOOR_M, ceil_m);

			const float err_h = h_tgt_m - alt.height_m;

			/* 条件积分（与 pid.c 同风格）：修正饱和且误差同向时冻结 */
			float i_new = alt_i + alt_ki * err_h * dt_a;
			i_new = FC_SAT(i_new, -ALT_I_LIM, ALT_I_LIM);

			float corr = alt_kp * err_h + i_new + alt_kd * (vz_cmd - alt.vz_mps);
			if (corr > ALT_OUT_DELTA) {
				if (err_h <= 0.0f) { alt_i = i_new; }
				corr = ALT_OUT_DELTA;
			} else if (corr < -ALT_OUT_DELTA) {
				if (err_h >= 0.0f) { alt_i = i_new; }
				corr = -ALT_OUT_DELTA;
			} else {
				alt_i = i_new;
			}

			/* 地面守卫：目标与实际高度都贴地才怠速（解锁在地面不自转起飞）；
			 * 若只看目标，空中拉杆下降会把油门直接切断导致自由落体 */
			if (assist_state == ASSIST_HOLD
				&& h_tgt_m <= ALT_TGT_FLOOR_M
				&& alt.height_m <= LAND_TOUCH_ALT_M) {
				base_01 = 0.0f;
			} else {
				base_01 = FC_SAT(hover_ff + corr, 0.05f, 0.95f);
			}
		}

#else
		/* ---- 2d'. 无定高辅助：CH7 就是电机锁 ---- */
		/* 拨高 = 解锁，允许遥控器控制（Safety 另校验油门在最低位）；
		 * 拨低 = 立即上锁。上电时开关若已在 H，2c 不产生解锁沿，故不会误解锁 */
		if (ch7_rise) {
			Safety_RequestArmRc(true);
			AnonLog(2u, "CH7: RC ARM");
		} else if (ch7_fall) {
			Safety_RequestArmRc(false);
			AnonLog(0u, "CH7: RC LOCK");
		}
#endif

		/* ---- 2f. 油门接管 + 斜变混合（仅 RC 路径，DBG 直通不斜变） ---- */
		if (!dbg_mode) {
#ifdef FEATURE_ALT_ASSIST
			if (assist_on) {
				thr_us = (uint16_t)(MOTOR_US_MIN + (uint16_t)(base_01 * 1000.0f));
			}
#endif
			if (rc_ok) {
				const int32_t d = (int32_t)thr_us - (int32_t)prev_thr_us;
				const int32_t step = (d > (int32_t)BASE_SLEW_US) ? (int32_t)BASE_SLEW_US
				                   : ((d < -(int32_t)BASE_SLEW_US) ? -(int32_t)BASE_SLEW_US : d);
				thr_us = (uint16_t)((int32_t)prev_thr_us + step);
			}
		}
		prev_thr_us = thr_us;

		/* ---- 3. 在线同步增益（上位机改 ID 2000~2020 即时生效） ---- */
		if ((tick % 50u) == 0u) {   /* 100ms 拉一次，别每拍调（后续修改，加标志位，上位机改了才拉） */
				Pid_SetGains(&pid_roll_rate,  Anon_GetPidGain(2009u), Anon_GetPidGain(2010u), Anon_GetPidGain(2011u));
				Pid_SetGains(&pid_pitch_rate, Anon_GetPidGain(2012u), Anon_GetPidGain(2013u), Anon_GetPidGain(2014u));
				Pid_SetGains(&pid_yaw_rate,   Anon_GetPidGain(2015u), Anon_GetPidGain(2016u), Anon_GetPidGain(2017u));
				Pid_SetGains(&pid_roll_att,   Anon_GetPidGain(2000u), Anon_GetPidGain(2001u), Anon_GetPidGain(2002u));
				Pid_SetGains(&pid_pitch_att,  Anon_GetPidGain(2003u), Anon_GetPidGain(2004u), Anon_GetPidGain(2005u));
				Pid_SetGains(&pid_yaw_att,    Anon_GetPidGain(2006u), Anon_GetPidGain(2007u), Anon_GetPidGain(2008u));
#ifdef FEATURE_ALT_ASSIST
				/* 高度环 2018~2020：直接取缓存，写入即生效 */
				alt_kp = Anon_GetPidGain(2018u);
				alt_ki = Anon_GetPidGain(2019u);
				alt_kd = Anon_GetPidGain(2020u);
#endif
		}

		/* ---- 4. 外环（ATT_HOLD 模式，新姿态事件驱动，约 200Hz） ---- */
		const bool att_mode = control_on && att_ok;
		if (att_mode) {
			att_loss_cnt = 0u;
			if (att.stamp.seq != last_att_seq) {   /* 只有新姿态帧才执行外环 */
					/* dt 用姿态邮戳差分：BusTask 掉速时自动适配；
					 * 首帧或断流后恢复时 last_att_usec 为 0/过旧，夹紧到 5ms 起步 */
					float dt_att = (last_att_usec == 0u) ? 0.005f
					                     : (float)(att.stamp.usec - last_att_usec) * 1e-6f;
					if (dt_att <= 0.0f)      { dt_att = 0.005f; }   /* 邮戳异常保护 */
					if (dt_att > 0.03f)      { dt_att = 0.03f;  }   /* 断流恢复夹紧 */

					const float act_roll  = att.roll_rad  * 57.2957795f;
					const float act_pitch = att.pitch_rad * 57.2957795f;
					/* 外环全程用度：输入 deg、输出 deg/s，送内环前显式转 rad/s */
					tgt_gx = Pid_Update(&pid_roll_att,  tgt_roll_deg,  act_roll,  dt_att) * 0.0174533f;
					tgt_gy = Pid_Update(&pid_pitch_att, tgt_pitch_deg, act_pitch, dt_att) * 0.0174533f;

					/* Yaw 外环：磁力计可信才做角度保持，否则回退速率直通 */
					if (att.mag_used) {
							const float yaw_meas = att.yaw_rad * 57.2957795f;
							if (!yaw_hold_active) {
									/* 切入角度模式：锁存当前航向为目标，避免目标跳变 */
									yaw_hold_deg    = yaw_meas;
									yaw_hold_active = true;
									Pid_Reset(&pid_yaw_att);
							}
							/* 摇杆叠加到航向目标：回中即保持新航向 */
							yaw_hold_deg += tgt_yaw_rate * 57.2957795f * dt_att;
							/* 环绕处理：目标夹到 [-180,180]，防 ±180° 处 358° 假误差 */
							while (yaw_hold_deg >  180.0f) { yaw_hold_deg -= 360.0f; }
							while (yaw_hold_deg < -180.0f) { yaw_hold_deg += 360.0f; }
							tgt_gz = Pid_Update(&pid_yaw_att, yaw_hold_deg, yaw_meas, dt_att) * 0.0174533f;
					} else {
							yaw_hold_active = false;
							Pid_Reset(&pid_yaw_att);
					}

					last_att_seq  = att.stamp.seq;
					last_att_usec = att.stamp.usec;
			}
    /* else 保持上次外环输出，供下一拍内环继续跟踪 */
		} else if (control_on) {
			/* 姿态短暂丢失：保持上次外环输出，连续超过 50ms 才清零复位 */
			if (++att_loss_cnt > 25u) {
					att_loss_cnt = 0u;
					tgt_gx = 0.0f;
					tgt_gy = 0.0f;
					yaw_hold_active = false;
					Pid_Reset(&pid_roll_att);
					Pid_Reset(&pid_pitch_att);
					Pid_Reset(&pid_yaw_att);
			}
		} else {
				/* 控制关闭：外环立即清零复位 */
				att_loss_cnt = 0u;
				tgt_gx = 0.0f;
				tgt_gy = 0.0f;
				yaw_hold_active = false;
				Pid_Reset(&pid_roll_att);
				Pid_Reset(&pid_pitch_att);
				Pid_Reset(&pid_yaw_att);
		}

		/* Yaw：未进入角度保持时，速率目标每拍直通 */
		if (!yaw_hold_active) { tgt_gz = tgt_yaw_rate; }

		/* ---- 5. 内环（500Hz） ---- */
		float out_roll = 0.0f, out_pitch = 0.0f, out_yaw = 0.0f;
		if (sen_ok && control_on) {
				out_roll  = Pid_Update(&pid_roll_rate,  tgt_gx, sen.gx_radps, dt);
				out_pitch = Pid_Update(&pid_pitch_rate, tgt_gy, sen.gy_radps, dt);
				out_yaw   = Pid_Update(&pid_yaw_rate,   tgt_gz, sen.gz_radps, dt);
		} else {
				Pid_Reset(&pid_roll_rate);
				Pid_Reset(&pid_pitch_rate);
				Pid_Reset(&pid_yaw_rate);
		}

		/* ---- 6. 混控（门控在 motor.c 里，DISARMED 时自动钳到 1000µs） ---- */
		Motor_Mix(thr_us, out_roll, out_pitch, out_yaw);

		/* ---- 7. 发布 ctrl 快照（首字段 FcStamp 由 Publish 回填） ---- */
		ControlOutput out;
		(void)memset(&out, 0, sizeof(out));
		out.valid           = true;
		out.motor_base_01   = (float)(thr_us - MOTOR_US_MIN) / 1000.0f;
		out.roll_torque_nd  = out_roll;
		out.pitch_torque_nd = out_pitch;
		out.yaw_torque_nd   = out_yaw;
		out.assist_active   = assist_on;
#ifdef FEATURE_ALT_ASSIST
		out.alt_target_m    = assist_on ? h_tgt_m : 0.0f;
		out.abort_reason    = abort_reason;
#else
		out.alt_target_m    = 0.0f;
		out.abort_reason    = ALT_ABORT_NONE;
#endif
		FcChan_Publish(&g_chan_ctrl, &out, now);

		/* ---- 8. 波形接线（每拍写，CommTask 10ms 读） ---- */
		uint8_t status = 0u;
		if (Safety_IsArmed()) { status |= 0x01u; }
		if (att_ok)           { status |= 0x02u; }
		if (sen_ok)           { status |= 0x04u; }
		if (rc_ok)            { status |= 0x08u; }   /* bit3 = RC 存活 */
		if (dbg_mode)         { status |= 0x10u; }   /* bit4 = DBG 接管 */
		if (assist_on)        { status |= 0x20u; }   /* bit5 = 定高辅助激活 */
		const int16_t sr = (int16_t)(tgt_roll_deg  * 100.0f);
		const int16_t sp = (int16_t)(tgt_pitch_deg * 100.0f);
		const int16_t sy = (int16_t)(tgt_yaw_rate * 57.2957795f * 100.0f);
		Anon_SetTuneTargets(sr, sp, sy, status, (uint8_t)dbg.mode);
	}
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_Attitude_Task */
/**
* @brief Function implementing the AttitudeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Attitude_Task */
void Attitude_Task(void *argument)
{
  /* USER CODE BEGIN Attitude_Task */
  /* Infinite loop */
	Attitude_Init();
	uint32_t last_tick = 0u;

	for (;;) {
		/* 等 BusTask 每 5 ms 发一次的新样本通知；超时说明采样链断了，
		 * 回到等待即可，由安全层的姿态年龄判据把整机降级 */
		if ((osThreadFlagsWait(ATT_FLAG_NEW_SAMPLE, osFlagsWaitAny,
		                       ATT_WAIT_TIMEOUT_MS) & ATT_FLAG_NEW_SAMPLE) == 0u) {
			continue;
		}

		SensorData sen;
		if (!FcChan_Read(&g_chan_sensor, &sen) || !sen.valid) {
			continue;
		}

		const uint32_t now = Fc_GetMicros();
		float dt = (last_tick == 0u) ? 0.005f : (float)(sen.stamp.usec - last_tick) * 1e-6f;
		last_tick = sen.stamp.usec;

		/* TODO: 调用 Madgwick 更新，具体 API 看 Madgwick.h */
		if (!Attitude_Update(&sen, dt)) {
			continue;                     /* 输入无效，跳过本拍 */
    }
		
		const Quaternion_t  qq = Attitude_GetQuaternion();
		const EulerAngles_t ee = Attitude_GetEulerAngles();

		AttitudeSnapshot att;
		(void)memset(&att, 0, sizeof(att));
		
		att.valid     = true;
		att.mag_used  = Attitude_IsMagUsed();
		att.q0        = qq.q0;
		att.q1        = qq.q1;
		att.q2        = qq.q2;
		att.q3        = qq.q3;
		att.roll_rad  = ee.roll;
		att.pitch_rad = ee.pitch;
		att.yaw_rad   = ee.yaw;

		FcChan_Publish(&g_chan_attitude, &att, now);
	}
  /* USER CODE END Attitude_Task */
}

/* USER CODE BEGIN Header_Rc_Task */
/**
* @brief Function implementing the RcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Rc_Task */
void Rc_Task(void *argument)
{
  /* USER CODE BEGIN Rc_Task */
	(void)argument;
	uint32_t wake = osKernelGetTickCount() + RC_PERIOD_MS;
	
  /* Infinite loop */
	for (;;) {
		PeriodWait(&wake, RC_PERIOD_MS);

		/* PPM 帧就绪或状态翻转时内部 publish，无新帧/无状态变化则跳过 */
		Rc_TaskStep();
	}
  /* USER CODE END Rc_Task */
}

/* USER CODE BEGIN Header_Comm_Task */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Comm_Task */
void Comm_Task(void *argument)
{
  /* USER CODE BEGIN Comm_Task */
  /* Infinite loop */
	(void)argument;

	uint8_t rx_buf[64];
	uint32_t wake = osKernelGetTickCount() + 10u;    /* 10 ms 周期 */

	for (;;) {
		PeriodWait(&wake, 10u);
		
		/* 0. 兜底重启：回调重启失败或 UART 错误后，恢复接收 */
		if (Uart1_Rx_TakeRestartReq()) {
			Uart1_Rx_Start();
		}

		/* 1. 排空 UART DMA 环形缓冲，喂给接收状态机 */
		uint16_t got;
		while ((got = Uart_RxRing_Pop(rx_buf, (uint16_t)sizeof(rx_buf))) > 0u) {
			Anonym_RxPush(rx_buf, got);
		}

		/* 2. 10 ms 上报节拍 */
		Anonym_Step();
		
		/* 3 发送 SafetyTask 缓存的日志（若有） */
		uint8_t log_color = 0u;
		char    log_msg[40];
		if (Safety_TakeLog(&log_color, log_msg, sizeof(log_msg))) {
				AnonLog(log_color, log_msg);
		}

		/* 4. 推一帧待发数据到 UART DMA（之前漏了这行） */
    Anonym_TxPump();
	}
  /* USER CODE END Comm_Task */
}

/* USER CODE BEGIN Header_Status_Task */
/**
* @brief Function implementing the StatusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Status_Task */
void Status_Task(void *argument)
{
  /* USER CODE BEGIN Status_Task */
  /* Infinite loop */
	(void)argument;

	uint32_t wake = osKernelGetTickCount() + STATUS_PERIOD_MS;

	for (;;) {
		PeriodWait(&wake, STATUS_PERIOD_MS);

		s_heap_free = (uint32_t)xPortGetFreeHeapSize();
		s_wm_att    = (uint32_t)uxTaskGetStackHighWaterMark(AttitudeTaskHandle);
		s_wm_ctrl = (uint32_t)uxTaskGetStackHighWaterMark(ControlTaskHandle);
		s_wm_safety = (uint32_t)uxTaskGetStackHighWaterMark(SafetyTaskHandle);
	}
  /* USER CODE END Status_Task */
}

/* USER CODE BEGIN Header_Display_Task */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Display_Task */
void Display_Task(void *argument)
{
  /* USER CODE BEGIN Display_Task */
  /* 心跳灯：LD2 每 100 ms 翻转一次，5 Hz 闪烁说明调度器仍在运行 */
  (void)argument;
  uint32_t wake = osKernelGetTickCount() + DISPLAY_PERIOD_MS;

  for (;;)
  {
    PeriodWait(&wake, DISPLAY_PERIOD_MS);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
  /* USER CODE END Display_Task */
}

/* USER CODE BEGIN Header_Safety_Task */
/**
* @brief Function implementing the SafetyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Safety_Task */
void Safety_Task(void *argument)
{
  /* USER CODE BEGIN Safety_Task */
  /* Infinite loop */
	(void)argument;
	uint32_t wake = osKernelGetTickCount() + SAFETY_PERIOD_MS;

	for (;;) {
		PeriodWait(&wake, SAFETY_PERIOD_MS);

		AttitudeSnapshot att;
		ControlOutput    ctrl;
		RcCommandSnapshot rc;

		const bool att_ok  = FcChan_Read(&g_chan_attitude, &att);
		const bool ctrl_ok = FcChan_Read(&g_chan_ctrl, &ctrl) && ctrl.valid;
		const bool rc_ok   = FcChan_Read(&g_chan_rc, &rc) && rc.valid;

		Safety_Step(att_ok  ? &att  : NULL,
								ctrl_ok ? &ctrl : NULL,
								rc_ok   ? &rc   : NULL);

		if (!Safety_IsAllowed()) {
			Motor_DisarmAll();
		}
	}
  /* USER CODE END Safety_Task */
}

/* USER CODE BEGIN Header_Bus_Task */
/**
* @brief Function implementing the BusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Bus_Task */
void Bus_Task(void *argument)
{
  /* USER CODE BEGIN Bus_Task */
  /* Infinite loop */
	(void)argument;

	/* IMU 初始化失败不阻塞其他任务：每 500 ms 重试一次，
	 * 并把失败码经 0xA0 LOG 帧打到上位机日志窗口。
	 * 码值：1=BUSY 2=TIMEOUT 3=NACK 4=BUS_FAULT（3 最常见=地址/接线错）。
	 * 此时链路数据不新鲜，安全层停留在 DISARMED，允许调试。 */
	I2cResult imu_r;
	uint32_t imu_try = 0u;
	char msg[24];
	while ((imu_r = GY87_Init()) != I2C_OK) {
		if ((imu_try++ % 3u) == 0u) {
			(void)snprintf(msg, sizeof(msg), "GY87 INIT FAIL r=%d", (int)imu_r);
			AnonLog(1u, msg);
		}
		osDelay(500u);
	}
	AnonLog(2u, "GY87 READY");

	SensorData s;
	uint32_t div = 0u;                      /* 每 5 个样本（5 ms = 200 Hz）通知 AT 一次 */
	uint32_t alt_div = 0u;                  /* Altitude_Tick 降频到 5ms：BMP180 慢速 I2C 不污染 1ms 采样节拍 */
	uint32_t wake = osKernelGetTickCount() + BUS_PERIOD_MS;

	for (;;) {
		PeriodWait(&wake, BUS_PERIOD_MS);

		I2C_QueueDrain();                   /* 执行延迟的恢复动作（空队列时开销为 0） */

		const fc_usec_t now = Fc_GetMicros();
		if (++alt_div >= 5u) {              /* BMP180 状态机基于 deadline，5ms 轮询粒度足够 */
			alt_div = 0u;
			Altitude_Tick(now);
		}
		
		if (GY87_Sample(&s, now) == I2C_OK) {
			/* 校准已在 GY87_Sample 出口施加，此处不得重复 */ 
			FcChan_Publish(&g_chan_sensor, &s, now);

			div = (div + 1u) % 5u;
			if (div == 0u) {
					(void)osThreadFlagsSet(AttitudeTaskHandle, ATT_FLAG_NEW_SAMPLE);
			}
		}

	}
  /* USER CODE END Bus_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

