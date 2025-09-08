#include "main.h"

#define DBG_TAG "MOTOR"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "board.h"
#include "state.h"
#include "pid.h"
#include "imu.h"

// PWM device handles (for motor control)
struct rt_device_pwm *pwm_dev1;
struct rt_device_pwm *pwm_dev9;
struct rt_device_pwm *pwm_dev10;
struct rt_device_pwm *pwm_dev11;

//#define PWM_DEV_NAME1 "pwm1"
//#define PWM_DEV_NAME9 "pwm9"
//#define PWM_DEV_NAME10 "pwm10"
//#define PWM_DEV_NAME11 "pwm11"

//#define MOTOR_LF_IN 4	// 左后电机的PWM引脚位TIM1的CH4
//                      // Left-rear motor PWM pin is TIM1 CH4
//#define MOTOR_LB_IN 2	// 左后电机的PWM引脚位TIM1的CH2
//                      // Left-back motor PWM pin is TIM1 CH2
//#define MOTOR_RF_IN 1	// 左后电机的PWM引脚位TIM9的CH1
//                      // Right-front motor PWM pin is TIM9 CH1
//#define MOTOR_RB_IN 1	// 左后电机的PWM引脚位TIM11的CH1
//                      // Right-back motor PWM pin is TIM11 CH1
//
//#define MOTOR_LF_BREAK 3 // 左前刹车 TIM1_CH3
//                        // Left-front brake signal on TIM1 CH3
//#define MOTOR_LB_BREAK 1 // 左前刹车 TIM1_CH1
//                        // Left-back brake signal on TIM1 CH1
//#define MOTOR_RF_BREAK 2 // 左前刹车 TIM9_CH3
//                        // Right-front brake signal on TIM9 CH3
//#define MOTOR_RB_BREAK 1 // 左前刹车 TIM10_CH1
//                        // Right-back brake signal on TIM10 CH1
//
//#define MOTOR_DIR_LF GET_PIN(A, 0)	 // 左前电机方向引脚
//                                      // Left-front motor direction pin
//#define MOTOR_DIR_LB GET_PIN(D, 12)	 // 左后电机方向引脚
//                                      // Left-back motor direction pin
//#define MOTOR_DIR_RF GET_PIN(B, 4)	 // 右前电机方向引脚
//                                      // Right-front motor direction pin
//#define MOTOR_DIR_RB GET_PIN(A, 15)	 // 右后电机方向引脚
//                                      // Right-back motor direction pin
//
////#define MOTOR_BREAK_LF GET_PIN(E, 13)
////#define MOTOR_BREAK_LB GET_PIN(E, 9)
////#define MOTOR_BREAK_RF GET_PIN(E, 6)
////#define MOTOR_BREAK_RB GET_PIN(B, 8)
//
//#define MOTOR_FG_LF GET_PIN(A, 1)     // 左前 FG 信号 (speed feedback pin)
//#define MOTOR_FG_LB GET_PIN(D, 13)    // 左后 FG 信号
//#define MOTOR_FG_RF GET_PIN(B, 5)     // 右前 FG 信号
//#define MOTOR_FG_RB GET_PIN(B, 3)     // 右后 FG 信号
//
//#define MOTOR_VIN_ENABLE_PIN GET_PIN(E, 4) // 电机电源使能引脚
//                                          // Motor VIN enable pin

//#define PERIOD 50000 // 周期，单位ns，频率20KHz。很关键的参数，调试好后尽量不动
//                    // Period in ns, frequency 20 kHz. Critical parameter, avoid changing after tuning
//#define PERIOD 10500 // 周期，单位ns，频率100KHz。很关键的参数，调试好后尽量不动
//#define PERIOD 16250 // 周期，单位ns，频率10KHz。很关键的参数，调试好后尽量不动
//#define PERIOD 40000 // 周期，单位ns，频率25KHz。很关键的参数，调试好后尽量不动

// Encoder device names for each wheel
#define ENCODER_LF_DEV "pulse5"
#define ENCODER_LB_DEV "pulse4"
#define ENCODER_RF_DEV "pulse3"
#define ENCODER_RB_DEV "pulse2"

// 定时器设备句柄 - timer device handles for encoder inputs
rt_device_t hw_dev_2,hw_dev_3,hw_dev_4,hw_dev_5 = RT_NULL;

pid_motor_control pid_motor;  // Motor PID control state
speed_pid speed_pid_s;        // Speed PID controller state

// Structure to represent one motor (PWM + encoder)
typedef struct robot_motor_t
{
	  struct rt_device_pwm *pwm_dev;                 // PWM device handle
	  struct rt_pulse_encoder_device *encoder_dev;   // Encoder device handle
} robot_motor_t;

// Motor instances for left-front, left-back, right-front, right-back
struct robot_motor_t motor_lf , motor_lb , motor_rf , motor_rb;

// Motor direction enumeration
typedef enum motor_direction_t
{
	  STOP, FORWARD, BACKWARD, LEFT, RIGHT,
} motor_direction_t;

// Motor state: PWM duty cycle + direction
typedef struct motor_state_t
{
	  uint8_t duty_l;             // Left motor duty cycle
	  uint8_t duty_r;             // Right motor duty cycle
	  motor_direction_t dir;      // Motion direction
} motor_state_t;

motor_state_t motor_state;       // Current motor state

// Exponential smoothing filter variables
float smoothedValue_speed = 0.0;
float smoothedValue_angular = 0.0;
float prevSmoothedValue_speed = 0.0;
float prevSmoothedValue_angular = 0.0;
float weight_speed = 0.25;   // Smoothing weight (0.25 for 24V, 0.3 for 12V)
float weight_angular = 0.55; // Smoothing weight for angular velocity

// Simple exponential smoothing filter function
float filter ( float rawValue , float w , float prevValue )
{
	  float result = w * rawValue + (1.0 - w) * prevValue;
	  return result;
}

extern u_int8_t count_state;
float coeff_angular = 0.7;  // Angular coefficient for control scaling

// Initialize motor control pins (direction, enable, etc.)
void motor_init_pin ( void )
{
	  // 初始化电机方向控制相关引脚
	  // Initialize motor direction control pins
//	  rt_pin_mode ( MOTOR_DIR_LF , PIN_MODE_OUTPUT );
//	  rt_pin_mode ( MOTOR_DIR_LB , PIN_MODE_OUTPUT );
//	  rt_pin_mode ( MOTOR_DIR_RF , PIN_MODE_OUTPUT );
//	  rt_pin_mode ( MOTOR_DIR_RB , PIN_MODE_OUTPUT );

      rt_pin_mode( MOTOR_DIR_L , PIN_MODE_OUTPUT ); // Left wheel direction
      rt_pin_mode( MOTOR_DIR_R , PIN_MODE_OUTPUT ); // Right wheel direction

//	  rt_pin_mode ( MOTOR_VIN_ENABLE_PIN , PIN_MODE_OUTPUT ); // 电机供电使能引脚 - motor VIN enable

	  // 初始化默认的电机方向状态
	  // Initialize default motor direction state
	  // 方向控制——默认前进方向
	  // Direction control: default set to "forward"
//	  rt_pin_write ( MOTOR_DIR_LF , PIN_LOW );   // 左前
//	  rt_pin_write ( MOTOR_DIR_LB , PIN_LOW );   // 左后
//	  rt_pin_write ( MOTOR_DIR_RF , PIN_HIGH );  // 右前
//	  rt_pin_write ( MOTOR_DIR_RB , PIN_HIGH );  // 右后
	  rt_pin_write ( MOTOR_DIR_L , PIN_LOW );   // 左轮 (forward)
	  rt_pin_write ( MOTOR_DIR_R , PIN_HIGH );  // 右轮 (forward)

	  // 初始化默认的电机供电使能状态
	  // Initialize default motor power enable state
	  // 供电使能控制——默认使能
	  // Power enable control: default ON
//	  rt_pin_write ( MOTOR_VIN_ENABLE_PIN , PIN_LOW );
//	  rt_thread_delay ( 1000 );
//	  rt_pin_write ( MOTOR_VIN_ENABLE_PIN , PIN_HIGH );
}

/**
 * @brief Initialize motor encoders and hardware timers.
 *
 * This function sets up the hardware timers (timer2, timer3, timer4, timer5) 
 * in periodic mode, which are used for handling motor encoder signals or timing tasks. 
 * Depending on whether `USE_PULPLSE_ENCODER` is defined, it either:
 *   - Initializes pulse encoder devices for each motor (LF, LB, RF, RB), or
 *   - Configures GPIO pins as inputs for fallback pulse detection.
 *
 * @note Uses RT-Thread device framework APIs (rt_device_find, rt_device_open, etc.).
 */
void motor_init_encoder(void)
{
#if 1   // Always enabled block for hardware timers
    rt_err_t ret = RT_EOK;
    rt_hwtimer_mode_t mode; /* Timer mode */

    // =============================
    // Setup Timer2
    // =============================
    hw_dev_2 = rt_device_find("timer2");
    if (hw_dev_2 == RT_NULL)
    {
        LOG_D("hwtimer sample run failed! can't find %s device!\n", "timer2");
        return RT_ERROR;
    }
    ret = rt_device_open(hw_dev_2, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        LOG_D("open %s device failed!\n", "timer2");
        return ret;
    }
    // Set timer2 to periodic mode (instead of default one-shot)
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev_2, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        LOG_D("set mode failed! ret is :%d\n", ret);
        return ret;
    }

    // =============================
    // Setup Timer3
    // =============================
    hw_dev_3 = rt_device_find("timer3");
    if (hw_dev_3 == RT_NULL)
    {
        LOG_D("hwtimer sample run failed! can't find %s device!\n", "timer3");
        return RT_ERROR;
    }
    ret = rt_device_open(hw_dev_3, RT_DEVICE_OFLAG_RDWR);
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev_3, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        LOG_D("set mode failed! ret is :%d\n", ret);
        return ret;
    }

    // =============================
    // Setup Timer4
    // =============================
    hw_dev_4 = rt_device_find("timer4");
    if (hw_dev_4 == RT_NULL)
    {
        LOG_D("hwtimer sample run failed! can't find %s device!\n", "timer4");
        return RT_ERROR;
    }
    ret = rt_device_open(hw_dev_4, RT_DEVICE_OFLAG_RDWR);
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev_4, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        LOG_D("set mode failed! ret is :%d\n", ret);
        return ret;
    }

    // =============================
    // Setup Timer5
    // =============================
    hw_dev_5 = rt_device_find("timer5");
    if (hw_dev_5 == RT_NULL)
    {
        LOG_D("hwtimer sample run failed! can't find %s device!\n", "timer5");
        return RT_ERROR;
    }
    ret = rt_device_open(hw_dev_5, RT_DEVICE_OFLAG_RDWR);
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev_5, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        LOG_D("set mode failed! ret is :%d\n", ret);
        return ret;
    }
#endif

#ifdef USE_PULPLSE_ENCODER
    // ======================================================
    // Encoder devices initialization (if pulse encoder driver is used)
    // ======================================================
    motor_lf.encoder_dev = (struct rt_pulse_encoder_device*) rt_device_find(ENCODER_LF_DEV);
    if (motor_lf.encoder_dev == RT_NULL)
    {
        LOG_E("cannot find %s", ENCODER_LF_DEV);
    }
    else
    {
        rt_device_open(motor_lf.encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    }

    motor_lb.encoder_dev = (struct rt_pulse_encoder_device*) rt_device_find(ENCODER_LB_DEV);
    if (motor_lb.encoder_dev == RT_NULL)
    {
        LOG_E("cannot find %s", ENCODER_LB_DEV);
    }
    else
    {
        rt_device_open(motor_lb.encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    }

    motor_rf.encoder_dev = (struct rt_pulse_encoder_device*) rt_device_find(ENCODER_RF_DEV);
    if (motor_rf.encoder_dev == RT_NULL)
    {
        LOG_E("cannot find %s", ENCODER_RF_DEV);
    }
    else
    {
        rt_device_open(motor_rf.encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    }

    motor_rb.encoder_dev = (struct rt_pulse_encoder_device*) rt_device_find(ENCODER_RB_DEV);
    if (motor_rb.encoder_dev == RT_NULL)
    {
        LOG_E("cannot find %s", ENCODER_RB_DEV);
    }
    else
    {
        rt_device_open(motor_rb.encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    }

#else
    // ======================================================
    // Fallback: Use GPIO pins as pulse input (no encoder driver)
    // ======================================================
    rt_pin_mode(MOTOR_FG_LF, PIN_MODE_INPUT);
    rt_pin_mode(MOTOR_FG_LB, PIN_MODE_INPUT);
    rt_pin_mode(MOTOR_FG_RF, PIN_MODE_INPUT);
    rt_pin_mode(MOTOR_FG_RB, PIN_MODE_INPUT);
#endif
}


/**
 * @brief Configure PWM channels for left and right wheels.
 *
 * This function initializes the PWM devices and sets up PWM output channels 
 * for the left motors (LF = left front, LB = left back). 
 * It configures their period and initial duty cycle (0%). 
 * It also enables the PWM channels to be ready for motor control.
 *
 * @note Currently only pwm1 (for left motors) is active. 
 *       Other devices (pwm9, pwm10, pwm11 for right motors and brake control) 
 *       are present but commented out.
 */
void motor_init_pwm(void)
{
    rt_err_t ret = RT_EOK;

    // =========================================================
    // Initialize PWM1 (used for left motors: LF and LB)
    // =========================================================
    pwm_dev1 = (struct rt_device_pwm*) rt_device_find(PWM_DEV_NAME1);
    if (pwm_dev1 != RT_NULL)
    {
        LOG_D("pwm1 device found");

        // Configure Left Front motor PWM channel
        rt_pwm_set(pwm_dev1, MOTOR_LF_IN, PERIOD, 0);  // PERIOD = base cycle, 0 = initial duty
        ret = rt_pwm_enable(pwm_dev1, MOTOR_LF_IN);
        if (ret != RT_EOK)
        {
            LOG_E("ret=%d => rt_pwm_enable(pwm_dev1, MOTOR_LF_IN)", ret);
        }

        // Configure Left Back motor PWM channel
        rt_pwm_set(pwm_dev1, MOTOR_LB_IN, PERIOD, 0);
        ret = rt_pwm_enable(pwm_dev1, MOTOR_LB_IN);
        if (ret != RT_EOK)
        {
            LOG_E("ret=%d => rt_pwm_enable(pwm_dev1, MOTOR_LB_IN)", ret);
        }

        // -----------------------------------------------------
        // Example for enabling brake channels (currently disabled)
        // -----------------------------------------------------
        // rt_pwm_set(pwm_dev1, MOTOR_LF_BREAK, PERIOD, PERIOD + 1);
        // ret = rt_pwm_enable(pwm_dev1, MOTOR_LF_BREAK);
        // ...
    }
    else
    {
        LOG_D("No pwm1 device found");
    }

    // =========================================================
    // (Commented out sections for other PWM devices)
    // pwm9  -> Right Front motor
    // pwm10 -> Right Front brake
    // pwm11 -> Right Back motor
    // =========================================================
    // These sections follow the same pattern: 
    // find -> configure period + duty -> enable channel
}

/**
 * @brief Apply limits to PWM duty cycle values.
 *
 * This function ensures that the duty cycle is constrained 
 * to a valid range, avoiding very low or very high values 
 * that may cause issues in motor control.
 *
 * @param data Raw duty cycle value (0–100).
 * @return rt_uint32_t Limited duty cycle:
 *         - Values ≤ 5% are clamped to 0%.
 *         - Values ≥ 95% are clamped to 100%.
 *         - Otherwise returns the input unchanged.
 */
rt_uint32_t motor_pwm_limit(rt_uint32_t data)
{
    if (data <= 5)
    {
        return 0;
    }
    else if (data >= 95)
    {
        return 100;
    }
    else
    {
        return data;
    }
}

// Sigmoid函数
float_t sigmoid ( float_t x )
{
	  return 1.0f / (1.0f + expf ( -x ));
}

rt_uint16_t get_variable_acceleration ( float s_start , float s_end , float s_step )
{
	  for ( float x = s_start ; x <= s_end ; x += s_step )
	  {
			float y = sigmoid ( x );
			LOG_W( "%d %d" , x , (rt_uint16_t )y );
	  }
}

//void motor_break ( rt_uint8_t break_mode )
//{
//	  switch ( break_mode )
//	  {
//			case 0 :  //停速——不刹车
//				  robot_state.break_status = 0;
//				  rt_pwm_set ( pwm_dev1 , MOTOR_LF_BREAK , PERIOD , 0 );
//				  rt_pwm_set ( pwm_dev1 , MOTOR_LB_BREAK , PERIOD , 0 );
//				  rt_pwm_set ( pwm_dev9 , MOTOR_RF_BREAK , PERIOD , 0 );
//				  rt_pwm_set ( pwm_dev10 , MOTOR_RB_BREAK , PERIOD , 0 );
//
//				  break;
//			case 1 :  //慢刹车
//				  if ( robot_state.break_status == 0 )
//				  {
//						robot_state.speed = 0;
//						robot_state.steer = 0;
//						robot_state.break_status = 1;
//						for ( rt_uint16_t var = 0 ; var <= 100 ; ++var )
//						{
//							  rt_pwm_set ( pwm_dev1 , MOTOR_LF_BREAK , PERIOD , PERIOD * var / 100 );
//							  rt_pwm_set ( pwm_dev1 , MOTOR_LB_BREAK , PERIOD , PERIOD * var / 100 );
//							  rt_pwm_set ( pwm_dev9 , MOTOR_RF_BREAK , PERIOD , PERIOD * var / 100 );
//							  rt_pwm_set ( pwm_dev10 , MOTOR_RB_BREAK , PERIOD , PERIOD * var / 100 );
//							  rt_thread_delay ( 3 );
//						}
//				  }
//				  break;
//			case 2 :  //急刹车
//				  robot_state.speed = 0;
//				  robot_state.steer = 0;
//				  robot_state.break_status = 1;
//				  rt_pwm_set ( pwm_dev1 , MOTOR_LF_BREAK , PERIOD , PERIOD );
//				  rt_pwm_set ( pwm_dev1 , MOTOR_LB_BREAK , PERIOD , PERIOD );
//				  rt_pwm_set ( pwm_dev9 , MOTOR_RF_BREAK , PERIOD , PERIOD );
//				  rt_pwm_set ( pwm_dev10 , MOTOR_RB_BREAK , PERIOD , PERIOD );
//
//				  break;
//			default :  //默认不刹车
//				  robot_state.break_status = 0;
//				  rt_pwm_set ( pwm_dev1 , MOTOR_LF_BREAK , PERIOD , 0 );
//				  rt_pwm_set ( pwm_dev1 , MOTOR_LB_BREAK , PERIOD , 0 );
//				  rt_pwm_set ( pwm_dev9 , MOTOR_RF_BREAK , PERIOD , 0 );
//				  rt_pwm_set ( pwm_dev10 , MOTOR_RB_BREAK , PERIOD , 0 );
//
////				  rt_pin_write ( MOTOR_BREAK_LF , PIN_LOW );  //默认低电平刹车
////				  rt_pin_write ( MOTOR_BREAK_LB , PIN_LOW );
////				  rt_pin_write ( MOTOR_BREAK_RF , PIN_LOW );
////				  rt_pin_write ( MOTOR_BREAK_RB , PIN_LOW );
//				  break;
//	  }
//}
/**
 * @brief Move robot backward.
 *
 * Depending on whether PID control is enabled:
 * - PID_OFF: Directly sets PWM duty cycles and motor direction pins.
 * - PID enabled: Updates PID targets and direction flags.
 *
 * @param duty_lf Duty cycle for Left Front motor (0–100).
 * @param duty_lb Duty cycle for Left Back motor (0–100).
 * @param duty_rf Duty cycle for Right Front motor (0–100).
 * @param duty_rb Duty cycle for Right Back motor (0–100).
 */
void motor_move_backward(rt_uint32_t duty_lf, rt_uint32_t duty_lb,
	rt_uint32_t duty_rf, rt_uint32_t duty_rb)
{
#ifdef PID_OFF
// Convert duty cycles to PWM pulse widths (apply limit function)
rt_uint32_t pulse_lf = PERIOD * motor_pwm_limit(duty_lf) / 100;
rt_uint32_t pulse_lb = PERIOD * motor_pwm_limit(duty_lb) / 100;
rt_uint32_t pulse_rf = PERIOD * motor_pwm_limit(duty_rf) / 100;
rt_uint32_t pulse_rb = PERIOD * motor_pwm_limit(duty_rb) / 100;

// Motor direction (backward: Right=LOW, Left=HIGH)
rt_pin_write(MOTOR_DIR_R, PIN_LOW);
rt_pin_write(MOTOR_DIR_L, PIN_HIGH);

// Apply PWM to motors
rt_pwm_set(pwm_dev1, MOTOR_R_IN, PERIOD, pulse_rf);
rt_pwm_set(pwm_dev1, MOTOR_L_IN, PERIOD, pulse_lf);
#else
// Update PID control direction (1 = backward)
pid_motor.PID_target_sign_br = 1;
pid_motor.PID_target_sign_fr = 1;
pid_motor.PID_target_sign_bl = 1;
pid_motor.PID_target_sign_fl = 1;

// Update PID target values (scaled by MIX_TARGET)
pid_motor.PID_target_br = MIX_TARGET * motor_pwm_limit(duty_rb) / 100;
pid_motor.PID_target_fr = MIX_TARGET * motor_pwm_limit(duty_rf) / 100;
pid_motor.PID_target_bl = MIX_TARGET * motor_pwm_limit(duty_lb) / 100;
pid_motor.PID_target_fl = MIX_TARGET * motor_pwm_limit(duty_lf) / 100;
#endif
}

/**
* @brief Move robot forward.
*/
void motor_move_forward(rt_uint32_t duty_lf, rt_uint32_t duty_lb,
   rt_uint32_t duty_rf, rt_uint32_t duty_rb)
{
#ifdef PID_OFF
rt_uint32_t pulse_lf = PERIOD * motor_pwm_limit(duty_lf) / 100;
rt_uint32_t pulse_lb = PERIOD * motor_pwm_limit(duty_lb) / 100;
rt_uint32_t pulse_rf = PERIOD * motor_pwm_limit(duty_rf) / 100;
rt_uint32_t pulse_rb = PERIOD * motor_pwm_limit(duty_rb) / 100;

// Motor direction (forward: Right=HIGH, Left=LOW)
rt_pin_write(MOTOR_DIR_R, PIN_HIGH);
rt_pin_write(MOTOR_DIR_L, PIN_LOW);

rt_pwm_set(pwm_dev1, MOTOR_R_IN, PERIOD, pulse_rf);
rt_pwm_set(pwm_dev1, MOTOR_L_IN, PERIOD, pulse_lf);
#else
// Update PID control direction (0 = forward)
pid_motor.PID_target_sign_br = 0;
pid_motor.PID_target_sign_fr = 0;
pid_motor.PID_target_sign_bl = 0;
pid_motor.PID_target_sign_fl = 0;

pid_motor.PID_target_br = MIX_TARGET * motor_pwm_limit(duty_rb) / 100;
pid_motor.PID_target_fr = MIX_TARGET * motor_pwm_limit(duty_rf) / 100;
pid_motor.PID_target_bl = MIX_TARGET * motor_pwm_limit(duty_lb) / 100;
pid_motor.PID_target_fl = MIX_TARGET * motor_pwm_limit(duty_lf) / 100;
#endif
}

/**
* @brief Move robot to the right (sideways movement).
*
* Uses IMU tilt compensation: reduces power if the robot tilts 
* beyond ±18° in roll or pitch.
*/
void motor_move_right(rt_uint32_t duty_lf, rt_uint32_t duty_lb,
 rt_uint32_t duty_rf, rt_uint32_t duty_rb)
{
#ifdef PID_OFF
rt_uint32_t pulse_lf = PERIOD * motor_pwm_limit(duty_lf) / 100;
rt_uint32_t pulse_lb = PERIOD * motor_pwm_limit(duty_lb) / 100;
rt_uint32_t pulse_rf = PERIOD * motor_pwm_limit(duty_rf) / 100;
rt_uint32_t pulse_rb = PERIOD * motor_pwm_limit(duty_rb) / 100;

// Motor direction: Right=LOW, Left=LOW
rt_pin_write(MOTOR_DIR_R, PIN_LOW);
rt_pin_write(MOTOR_DIR_L, PIN_LOW);

rt_pwm_set(pwm_dev1, MOTOR_R_IN, PERIOD, pulse_rf);
rt_pwm_set(pwm_dev1, MOTOR_L_IN, PERIOD, pulse_lf);
#else
// Mixed directions: right wheels backward, left wheels forward
pid_motor.PID_target_sign_br = 1;
pid_motor.PID_target_sign_fr = 1;
pid_motor.PID_target_sign_bl = 0;
pid_motor.PID_target_sign_fl = 0;

// IMU tilt check for power adjustment
if (abs(IMU_updata.imu_data[0]) >= 18.0f || abs(IMU_updata.imu_data[1]) >= 18.0f)
{
// Reduce power to 70% if tilted
pid_motor.PID_target_br = 0.7 * MIX_TARGET * motor_pwm_limit(duty_rb) / 100;
pid_motor.PID_target_fr = 0.7 * MIX_TARGET * motor_pwm_limit(duty_rf) / 100;
pid_motor.PID_target_bl = 0.7 * MIX_TARGET * motor_pwm_limit(duty_lb) / 100;
pid_motor.PID_target_fl = 0.7 * MIX_TARGET * motor_pwm_limit(duty_lf) / 100;
}
else
{
// Default lower speed (30%)
pid_motor.PID_target_br = 0.3 * MIX_TARGET * motor_pwm_limit(duty_rb) / 100;
pid_motor.PID_target_fr = 0.3 * MIX_TARGET * motor_pwm_limit(duty_rf) / 100;
pid_motor.PID_target_bl = 0.3 * MIX_TARGET * motor_pwm_limit(duty_lb) / 100;
pid_motor.PID_target_fl = 0.3 * MIX_TARGET * motor_pwm_limit(duty_lf) / 100;
}
#endif
}

/**
* @brief Move robot to the left (sideways movement).
*
* Uses IMU tilt compensation: reduces power if the robot tilts 
* beyond ±18° in roll or pitch.
*/
void motor_move_left(rt_uint32_t duty_lf, rt_uint32_t duty_lb,
rt_uint32_t duty_rf, rt_uint32_t duty_rb)
{
#ifdef PID_OFF
rt_uint32_t pulse_lf = PERIOD * motor_pwm_limit(duty_lf) / 100;
rt_uint32_t pulse_lb = PERIOD * motor_pwm_limit(duty_lb) / 100;
rt_uint32_t pulse_rf = PERIOD * motor_pwm_limit(duty_rf) / 100;
rt_uint32_t pulse_rb = PERIOD * motor_pwm_limit(duty_rb) / 100;

// Motor direction: Right=HIGH, Left=HIGH
rt_pin_write(MOTOR_DIR_R, PIN_HIGH);
rt_pin_write(MOTOR_DIR_L, PIN_HIGH);

rt_pwm_set(pwm_dev1, MOTOR_R_IN, PERIOD, pulse_rf);
rt_pwm_set(pwm_dev1, MOTOR_L_IN, PERIOD, pulse_lf);
#else
// Mixed directions: right wheels forward, left wheels backward
pid_motor.PID_target_sign_br = 0;
pid_motor.PID_target_sign_fr = 0;
pid_motor.PID_target_sign_bl = 1;
pid_motor.PID_target_sign_fl = 1;

if (abs(IMU_updata.imu_data[0]) >= 18.0f || abs(IMU_updata.imu_data[1]) >= 18.0f)
{
pid_motor.PID_target_br = 0.7 * MIX_TARGET * motor_pwm_limit(duty_rb) / 100;
pid_motor.PID_target_fr = 0.7 * MIX_TARGET * motor_pwm_limit(duty_rf) / 100;
pid_motor.PID_target_bl = 0.7 * MIX_TARGET * motor_pwm_limit(duty_lb) / 100;
pid_motor.PID_target_fl = 0.7 * MIX_TARGET * motor_pwm_limit(duty_lf) / 100;
}
else
{
pid_motor.PID_target_br = 0.3 * MIX_TARGET * motor_pwm_limit(duty_rb) / 100;
pid_motor.PID_target_fr = 0.3 * MIX_TARGET * motor_pwm_limit(duty_rf) / 100;
pid_motor.PID_target_bl = 0.3 * MIX_TARGET * motor_pwm_limit(duty_lb) / 100;
pid_motor.PID_target_fl = 0.3 * MIX_TARGET * motor_pwm_limit(duty_lf) / 100;
}
#endif
}

/**
* @brief Stop the robot by setting all motors to 0 duty cycle.
*/
void motor_move_stop(void)
{
motor_move_forward(0, 0, 0, 0);
}

/**
* @brief Read encoder RPM values for all motors.
*
* @param rpm Array of size 4 where encoder readings will be stored:
*            rpm[0] = Left Front, rpm[1] = Right Front,
*            rpm[2] = Left Back, rpm[3] = Right Back.
*
* @note Currently commented out. Requires encoder driver.
*/
void motor_encoder_read(rt_int32_t *rpm)
{
// Example:
// rt_device_read(motor_lf.encoder_dev, 0, &rpm[0], 1);
// rt_device_read(motor_rf.encoder_dev, 0, &rpm[1], 1);
// rt_device_read(motor_lb.encoder_dev, 0, &rpm[2], 1);
// rt_device_read(motor_rb.encoder_dev, 0, &rpm[3], 1);
}

/**
* @brief Reset encoder counts for all motors.
*
* @note Currently commented out. Requires encoder driver.
*/
void motor_encoder_reset()
{
// Example:
// rt_device_control(motor_lf.encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
// rt_device_control(motor_rf.encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
// rt_device_control(motor_lb.encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
// rt_device_control(motor_rb.encoder_dev, PULSE_ENCODE_
}
extern volatile rt_bool_t protect_flag;  // 被底部监管保护的值 (value monitored/protected by lower-level system watchdog)

/**
 * @brief Calculate motor PWM duty cycles (left and right wheels) based on speed and steering inputs.
 *
 * This function takes desired speed and steering values, applies constraints and smoothing,
 * then computes appropriate left/right duty cycle values. It also determines the robot's
 * movement direction (FORWARD, BACKWARD, LEFT, RIGHT).
 *
 * @param speed Desired forward/backward speed input (range -100 to +100 approx).
 * @param steer Desired steering input (range -100 to +100 approx).
 */
void motor_get_duty (int16_t speed, int16_t steer)
{
    uint16_t speed_abs = abs(speed);
    uint16_t steer_abs = abs(steer);

    // // 后加往下 (added later, placed below)
    // If system enters protection mode, zero out motor duty and return
    // if (protect_flag == RT_TRUE)
    // {
    //     motor_state.duty_l = 0;
    //     motor_state.duty_r = 0;
    //     // LOG_W("Protected state: system override, no motor output");
    //     return;
    // }

    // 后加往上 (added later, placed above)

    // Clamp speed and steer inputs to max 100
    if (speed_abs > 100) speed_abs = 100;
    if (steer_abs > 100) steer_abs = 100;

    // --- Optional filter stage (disabled in current code) ---
    // smoothedValue_speed = filter(speed_abs, weight_speed, prevSmoothedValue_speed);
    // smoothedValue_angular = filter(steer_abs, weight_angular, prevSmoothedValue_angular);
    // prevSmoothedValue_speed = smoothedValue_speed;
    // prevSmoothedValue_angular = smoothedValue_angular;

    smoothedValue_speed = speed_abs;     // 当前代码取消了手柄滤波 (controller filtering disabled)
    smoothedValue_angular = steer_abs;

    // If no input at all, force both to zero
    if (speed_abs == 0 && steer_abs == 0)
    {
        smoothedValue_speed = 0;
        smoothedValue_angular = 0;
    }

    // ==========================================================
    // CASE 1: Steering > 5 → turning right
    // ==========================================================
    if (steer > 5)
    {
        // Apply differential steering: left motor stronger, right motor weaker
        motor_state.duty_l = smoothedValue_speed * (100 + coeff_angular * smoothedValue_angular) / 100;
        motor_state.duty_r = smoothedValue_speed * (100 - coeff_angular * smoothedValue_angular) / 100;

        // Clamp left duty close to 100 if too high
        motor_state.duty_l = motor_state.duty_l > 95 ? 100 : motor_state.duty_l;

        // Set movement direction depending on speed sign
        if (speed > 5) motor_state.dir = FORWARD;
        else if (speed < -5) motor_state.dir = BACKWARD;
        else
        {
            // If no forward/backward but strong steering → rotate in place to the RIGHT
            if (steer_abs > 10)
            {
                motor_state.dir = RIGHT;
                motor_state.duty_l = coeff_angular * smoothedValue_angular;
                motor_state.duty_r = coeff_angular * smoothedValue_angular;
            }
        }
    }
    // ==========================================================
    // CASE 2: Steering < -5 → turning left
    // ==========================================================
    else if (steer < -5)
    {
        // Apply differential steering: right motor stronger, left motor weaker
        motor_state.duty_l = smoothedValue_speed * (100 - coeff_angular * smoothedValue_angular) / 100;
        motor_state.duty_r = smoothedValue_speed * (100 + coeff_angular * smoothedValue_angular) / 100;

        // Clamp right duty close to 100 if too high
        motor_state.duty_r = motor_state.duty_r > 95 ? 100 : motor_state.duty_r;

        if (speed > 5) motor_state.dir = FORWARD;
        else if (speed < -5) motor_state.dir = BACKWARD;
        else
        {
            // If no forward/backward but strong steering → rotate in place to the LEFT
            if (steer_abs > 10)
            {
                motor_state.dir = LEFT;
                motor_state.duty_l = coeff_angular * smoothedValue_angular;
                motor_state.duty_r = coeff_angular * smoothedValue_angular;
            }
        }
    }
    // ==========================================================
    // CASE 3: No significant steering → straight
    // ==========================================================
    else
    {
        if (speed > 5) motor_state.dir = FORWARD;
        else if (speed < -5) motor_state.dir = BACKWARD;

        // Both wheels equal duty (plus steering correction factor)
        motor_state.duty_l = smoothedValue_speed + coeff_angular * smoothedValue_angular;
        motor_state.duty_r = smoothedValue_speed + coeff_angular * smoothedValue_angular;

        // Clamp near 100%
        motor_state.duty_l = motor_state.duty_l > 95 ? 100 : motor_state.duty_l;
        motor_state.duty_r = motor_state.duty_r > 95 ? 100 : motor_state.duty_r;
    }

    // LOG_D("duty_l %d, duty_r %d", motor_state.duty_l, motor_state.duty_r);
}

/**
 * @brief Control motor behavior based on system state, IMU calibration events, and direction.
 *
 * This function:
 *  - Listens for IMU calibration events (start/stop).
 *  - Disables motor output if the system is in protect mode.
 *  - Runs a calibration movement sequence if magnetometer calibration is active.
 *  - Otherwise computes duty cycles based on robot speed/steer,
 *    and drives the motors according to the current direction (STOP, FORWARD, BACKWARD, LEFT, RIGHT).
 *
 * @param mode Brake mode selector (0: glide, 1: soft brake, 2: hard brake)
 */
void motor_contorl(rt_uint8_t mode)
{
    rt_uint32_t received_flags = 0;
    static uint8_t mag_calib = 0;   // Tracks if magnetometer calibration is active

    // ---------------------------
    // Handle IMU calibration events
    // ---------------------------
    if (imu_event != RT_NULL)
    {
        if (rt_event_recv(imu_event,
                          IMU_CALIB_MOTOR_CONTROL_START | IMU_CALIB_MOTOR_CONTROL_DONE,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_NO,
                          &received_flags) == RT_EOK)
        {
            if (received_flags & IMU_CALIB_MOTOR_CONTROL_START)
                mag_calib = 1; // Enter calibration mode
            if (received_flags & IMU_CALIB_MOTOR_CONTROL_DONE)
                mag_calib = 0; // Exit calibration mode
        }
    }

    // ---------------------------
    // Safety: Protect mode → Stop all motion
    // ---------------------------
    if (protect_flag == RT_TRUE)
    {
        motor_state.duty_l = 0;
        motor_state.duty_r = 0;
        // LOG_W("In protect mode, motors disabled.");
        return;
    }

    // ---------------------------
    // Magnetometer calibration movement
    // ---------------------------
    if (mag_calib)
    {
        // Spin robot to help calibration
        motor_move_right(70, 70, 70, 70);
        coeff_fl = 1;   coeff_fr = -1;
        coeff_bl = 1;   coeff_br = -1;
        // LOG_W("MOTOR_CONTROL_MAG_calib_mode");
        return;
    }

    // ---------------------------
    // Compute motor duty from robot state
    // ---------------------------
    motor_get_duty(robot_state.speed, robot_state.steer);

    // ---------------------------
    // Direction handling
    // ---------------------------
    switch (motor_state.dir)
    {
        case STOP:
            motor_move_stop(); // Fully stop motors
            break;

        case FORWARD:
            coeff_fl = 1; coeff_fr = 1;
            coeff_bl = 1; coeff_br = 1;
            motor_move_forward(motor_state.duty_l, motor_state.duty_l,
                               motor_state.duty_r, motor_state.duty_r);
            break;

        case BACKWARD:
            coeff_fl = -1; coeff_fr = -1;
            coeff_bl = -1; coeff_br = -1;
            motor_move_backward(motor_state.duty_l, motor_state.duty_l,
                                motor_state.duty_r, motor_state.duty_r);
            break;

        case LEFT:
            coeff_fl = -1; coeff_fr = 1;
            coeff_bl = -1; coeff_br = 1;
            motor_move_left(motor_state.duty_l, motor_state.duty_l,
                            motor_state.duty_r, motor_state.duty_r);
            break;

        case RIGHT:
            coeff_fl = 1; coeff_fr = -1;
            coeff_bl = 1; coeff_br = -1;
            motor_move_right(motor_state.duty_l, motor_state.duty_l,
                             motor_state.duty_r, motor_state.duty_r);
            break;
    }
}

/**
 * @brief Simple motor PWM test function.
 *
 * Sets PWM pins for left and right motor outputs as digital outputs
 * and drives them LOW to verify wiring and hardware function.
 */
void motor_pwm_test(void)
{
    rt_pin_mode(MOTOR_PWM_R, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR_PWM_L, PIN_MODE_OUTPUT);
    rt_pin_write(MOTOR_PWM_R, PIN_LOW);
    rt_pin_write(MOTOR_PWM_L, PIN_LOW);
}

/**
 * @brief Motor control thread entry point.
 *
 * Initializes motor hardware (pins, PWM, timers).
 * Continuously runs in a loop:
 *   - If power is enabled (robot_state.pwr == 1), call motor_contorl() with brake mode.
 *   - Otherwise, stop motors safely.
 *
 * Runs with a periodic delay (10 ms).
 *
 * @param parameter Unused thread parameter (RT-Thread convention).
 */
void motor_thread_entry(void *parameter)
{
    // Initialize motor hardware
    motor_init_pin();
    motor_init_pwm();
    // motor_init_encoder(); // Optional if encoders are not needed
    MX_TIM4_Init();
    MX_TIM5_Init();

    while (1)
    {
        if (robot_state.pwr == 1)
        {
            motor_contorl(1); // Brake mode = soft brake
        }
        else
        {
            motor_move_stop(); // Stop motors if power is off
        }

        // Loop period
        rt_thread_mdelay(10);
    }
}
