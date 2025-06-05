#include "main.h"

#define DBG_TAG "MOTOR"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "board.h"
#include "state.h"
#include "pid.h"
#include "imu.h"

struct rt_device_pwm *pwm_dev1;
struct rt_device_pwm *pwm_dev9;
struct rt_device_pwm *pwm_dev10;
struct rt_device_pwm *pwm_dev11;

//#define PWM_DEV_NAME1 "pwm1"
//#define PWM_DEV_NAME9 "pwm9"
//#define PWM_DEV_NAME10 "pwm10"
//#define PWM_DEV_NAME11 "pwm11"
//#define MOTOR_LF_IN 4	//左后电机的PWM引脚位TIM1的CH4
//#define MOTOR_LB_IN 2	//左后电机的PWM引脚位TIM1的CH2
//#define MOTOR_RF_IN 1	//左后电机的PWM引脚位TIM9的CH1
//#define MOTOR_RB_IN 1	//左后电机的PWM引脚位TIM11的CH1
//
//#define MOTOR_LF_BREAK 3//左前刹车TIM1_CH3
//#define MOTOR_LB_BREAK 1//左前刹车TIM1_CH1
//#define MOTOR_RF_BREAK 2//左前刹车TIM9_CH3
//#define MOTOR_RB_BREAK 1//左前刹车TIM10_CH1
//
//#define MOTOR_DIR_LF GET_PIN(A, 0)	//左前D
//#define MOTOR_DIR_LB GET_PIN(D, 12)	//左后C
//#define MOTOR_DIR_RF GET_PIN(B, 4)	//右前B
//#define MOTOR_DIR_RB GET_PIN(A, 15)	//右后A
//
////#define MOTOR_BREAK_LF GET_PIN(E, 13)
////#define MOTOR_BREAK_LB GET_PIN(E, 9)
////#define MOTOR_BREAK_RF GET_PIN(E, 6)
////#define MOTOR_BREAK_RB GET_PIN(B, 8)
//
//#define MOTOR_FG_LF GET_PIN(A, 1)
//#define MOTOR_FG_LB GET_PIN(D, 13)
//#define MOTOR_FG_RF GET_PIN(B, 5)
//#define MOTOR_FG_RB GET_PIN(B, 3)
//
//#define MOTOR_VIN_ENABLE_PIN GET_PIN(E, 4)

//#define PERIOD 50000//周期，单位ns，频率20KHz。很关键的参数，调试好后尽量不动
//#define PERIOD 10500//周期，单位ns，频率100KHz。很关键的参数，调试好后尽量不动

//#define PERIOD 16250//周期，单位ns，频率10KHz。很关键的参数，调试好后尽量不动

//#define PERIOD 40000//周期，单位ns，频率25KHz。很关键的参数，调试好后尽量不动

#define ENCODER_LF_DEV "pulse5"
#define ENCODER_LB_DEV "pulse4"
#define ENCODER_RF_DEV "pulse3"
#define ENCODER_RB_DEV "pulse2"
rt_device_t hw_dev_2,hw_dev_3,hw_dev_4,hw_dev_5 = RT_NULL;   /* 定时器设备句柄 */

pid_motor_control pid_motor;
speed_pid speed_pid_s;

typedef struct robot_motor_t
{
	  struct rt_device_pwm *pwm_dev;
	  struct rt_pulse_encoder_device *encoder_dev;
} robot_motor_t;

struct robot_motor_t motor_lf , motor_lb , motor_rf , motor_rb;

typedef enum motor_direction_t
{
	  STOP, FORWARD, BACKWARD, LEFT, RIGHT,
} motor_direction_t;

typedef struct motor_state_t
{
	  uint8_t duty_l;
	  uint8_t duty_r;
	  motor_direction_t dir;

} motor_state_t;

motor_state_t motor_state;

float smoothedValue_speed = 0.0;
float smoothedValue_angular = 0.0;
float prevSmoothedValue_speed = 0.0;
float prevSmoothedValue_angular = 0.0;
float weight_speed = 0.25;  //24V 0.25, 12V 0.3
float weight_angular = 0.55;

float filter ( float rawValue , float w , float prevValue )
{
	  float result = w * rawValue + (1.0 - w) * prevValue;
	  return result;
}

extern u_int8_t count_state;
float coeff_angular = 0.7;

void motor_init_pin ( void )
{
	  //初始化电机方向控制相关引脚
//	  rt_pin_mode ( MOTOR_DIR_LF , PIN_MODE_OUTPUT );
//	  rt_pin_mode ( MOTOR_DIR_LB , PIN_MODE_OUTPUT );
//	  rt_pin_mode ( MOTOR_DIR_RF , PIN_MODE_OUTPUT );
//	  rt_pin_mode ( MOTOR_DIR_RB , PIN_MODE_OUTPUT );

      rt_pin_mode( MOTOR_DIR_L , PIN_MODE_OUTPUT );
      rt_pin_mode( MOTOR_DIR_R , PIN_MODE_OUTPUT );

//	  rt_pin_mode ( MOTOR_VIN_ENABLE_PIN , PIN_MODE_OUTPUT );//FIXME

	  //初始化默认的电机方向状态
	  //方向控制——默认前进方向
//	  rt_pin_write ( MOTOR_DIR_LF , PIN_LOW );  //左前
//	  rt_pin_write ( MOTOR_DIR_LB , PIN_LOW );  //左后
//	  rt_pin_write ( MOTOR_DIR_RF , PIN_HIGH );  //右前
//	  rt_pin_write ( MOTOR_DIR_RB , PIN_HIGH );  //右后
	  rt_pin_write ( MOTOR_DIR_L , PIN_LOW );  //左轮
	  rt_pin_write ( MOTOR_DIR_R , PIN_HIGH );  //右轮

	  //初始化默认的电机供电使能状态
	  //供电使能控制——默认使能
//	  rt_pin_write ( MOTOR_VIN_ENABLE_PIN , PIN_LOW );  //右后
//	  rt_thread_delay ( 1000 );
//	  rt_pin_write ( MOTOR_VIN_ENABLE_PIN , PIN_HIGH );  //右后

}

void motor_init_encoder ( void )
{

#if 1
      rt_err_t ret = RT_EOK;
      rt_hwtimer_mode_t mode; /* 定时器模式 */
      hw_dev_2 = rt_device_find ( "timer2" );
      if ( hw_dev_2 == RT_NULL )
           {
                 LOG_D( "hwtimer sample run failed! can't find %s device!\n" , "timer2" );
                 return RT_ERROR;
           }
      ret = rt_device_open ( hw_dev_2 , RT_DEVICE_OFLAG_RDWR );
      if ( ret != RT_EOK )
            {
                  LOG_D( "open %s device failed!\n" , "timer2" );
                  return ret;
            }
      /* 设置模式为周期性定时器（若未设置，默认是HWTIMER_MODE_ONESHOT）*/
            mode = HWTIMER_MODE_PERIOD;
            ret = rt_device_control ( hw_dev_2 , HWTIMER_CTRL_MODE_SET , &mode );
            if ( ret != RT_EOK )
            {
                  LOG_D( "set mode failed! ret is :%d\n" , ret );
                  return ret;
            }

      hw_dev_3 = rt_device_find ( "timer3" );
      if ( hw_dev_3 == RT_NULL )
                {
                      LOG_D( "hwtimer sample run failed! can't find %s device!\n" , "timer3" );
                      return RT_ERROR;
                }
      ret = rt_device_open ( hw_dev_3 , RT_DEVICE_OFLAG_RDWR );
      /* 设置模式为周期性定时器（若未设置，默认是HWTIMER_MODE_ONESHOT）*/
                 mode = HWTIMER_MODE_PERIOD;
                 ret = rt_device_control ( hw_dev_3 , HWTIMER_CTRL_MODE_SET , &mode );
                 if ( ret != RT_EOK )
                 {
                       LOG_D( "set mode failed! ret is :%d\n" , ret );
                       return ret;
                 }

      hw_dev_4 = rt_device_find ( "timer4" );
      if ( hw_dev_4 == RT_NULL )
                {
                      LOG_D( "hwtimer sample run failed! can't find %s device!\n" , "timer4" );
                      return RT_ERROR;
                }
      ret = rt_device_open ( hw_dev_4 , RT_DEVICE_OFLAG_RDWR );
      /* 设置模式为周期性定时器（若未设置，默认是HWTIMER_MODE_ONESHOT）*/
                 mode = HWTIMER_MODE_PERIOD;
                 ret = rt_device_control ( hw_dev_4 , HWTIMER_CTRL_MODE_SET , &mode );
                 if ( ret != RT_EOK )
                 {
                       LOG_D( "set mode failed! ret is :%d\n" , ret );
                       return ret;
                 }

      hw_dev_5 = rt_device_find ( "timer5" );
      if ( hw_dev_5 == RT_NULL )
                {
                      LOG_D( "hwtimer sample run failed! can't find %s device!\n" , "timer5" );
                      return RT_ERROR;
                }
      ret = rt_device_open ( hw_dev_5 , RT_DEVICE_OFLAG_RDWR );
      /* 设置模式为周期性定时器（若未设置，默认是HWTIMER_MODE_ONESHOT）*/
                 mode = HWTIMER_MODE_PERIOD;
                 ret = rt_device_control ( hw_dev_5 , HWTIMER_CTRL_MODE_SET , &mode );
                 if ( ret != RT_EOK )
                 {
                       LOG_D( "set mode failed! ret is :%d\n" , ret );
                       return ret;
                 }

#endif
#ifdef USE_PULPLSE_ENCODER
	  motor_lf.encoder_dev = (struct rt_pulse_encoder_device*) rt_device_find ( ENCODER_LF_DEV );
	  if ( motor_lf.encoder_dev == RT_NULL )
	  {
			LOG_E( "cannot find %s" , ENCODER_LF_DEV );
	  }
	  else
	  {
			rt_device_open ( motor_lf.encoder_dev , RT_DEVICE_OFLAG_RDONLY );
	  }

	  motor_lb.encoder_dev = (struct rt_pulse_encoder_device*) rt_device_find ( ENCODER_LB_DEV );
	  if ( motor_lb.encoder_dev == RT_NULL )
	  {
			LOG_E( "cannot find %s" , ENCODER_LB_DEV );
	  }
	  else
	  {
			rt_device_open ( motor_lb.encoder_dev , RT_DEVICE_OFLAG_RDONLY );
	  }

	  motor_rf.encoder_dev = (struct rt_pulse_encoder_device*) rt_device_find ( ENCODER_RF_DEV );
	  if ( motor_rf.encoder_dev == RT_NULL )
	  {
			LOG_E( "cannot find %s" , ENCODER_RF_DEV );
	  }
	  else
	  {
			rt_device_open ( motor_rf.encoder_dev , RT_DEVICE_OFLAG_RDONLY );
	  }

	  motor_rb.encoder_dev = (struct rt_pulse_encoder_device*) rt_device_find ( ENCODER_RB_DEV );
	  if ( motor_rb.encoder_dev == RT_NULL )
	  {
			LOG_E( "cannot find %s" , ENCODER_RB_DEV );
	  }
	  else
	  {
			rt_device_open ( motor_rb.encoder_dev , RT_DEVICE_OFLAG_RDONLY );
	  }
#else
	  //初始化电机脉冲相关引脚
	  rt_pin_mode ( MOTOR_FG_LF , PIN_MODE_INPUT );
	  rt_pin_mode ( MOTOR_FG_LB , PIN_MODE_INPUT );
	  rt_pin_mode ( MOTOR_FG_RF , PIN_MODE_INPUT );
	  rt_pin_mode ( MOTOR_FG_RB , PIN_MODE_INPUT );

#endif

}

/**
 * brief:配置左右轮的PWM通道
 */
void motor_init_pwm ( void )
{
	  rt_err_t ret = RT_EOK;
	  pwm_dev1 = (struct rt_device_pwm*) rt_device_find ( PWM_DEV_NAME1 );
	  if ( pwm_dev1 != RT_NULL )
	  {
			LOG_D( "pwm1 device found" );
			// 配置PWM通道
			rt_pwm_set ( pwm_dev1 , MOTOR_LF_IN , PERIOD , 0 );
			ret = rt_pwm_enable ( pwm_dev1 , MOTOR_LF_IN );
			if ( ret != RT_EOK )
			{
				  LOG_E( "ret=%d => rt_pwm_enable ( pwm_dev1 , MOTOR_L_IN )" , ret );//
			}
			rt_pwm_set ( pwm_dev1 , MOTOR_LB_IN , PERIOD , 0 );
			ret = rt_pwm_enable ( pwm_dev1 , MOTOR_LB_IN );
			if ( ret != RT_EOK )
			{
				  LOG_E( "ret=%d => rt_pwm_enable ( pwm_dev1 , MOTOR_R_IN )" , ret );
			}

//			// 配置刹车通道
//			rt_pwm_set ( pwm_dev1 , MOTOR_LF_BREAK , PERIOD , PERIOD + 1 );
//			ret = rt_pwm_enable ( pwm_dev1 , MOTOR_LF_BREAK );
//			if ( ret != RT_EOK )
//			{
//				  LOG_E( "ret=%d => rt_pwm_enable ( pwm_dev1 , MOTOR_LF_BREAK )" , ret );
//			}
//			rt_pwm_set ( pwm_dev1 , MOTOR_LB_BREAK , PERIOD , PERIOD + 1 );
//			ret = rt_pwm_enable ( pwm_dev1 , MOTOR_LB_BREAK );
//			if ( ret != RT_EOK )
//			{
//				  LOG_E( "ret=%d => rt_pwm_enable ( pwm_dev1 , MOTOR_LB_BREAK )" , ret );
//			}
	  }
	  else
	  {
			LOG_D( "No pwm1 device found" );
	  }

//	  pwm_dev9 = (struct rt_device_pwm*) rt_device_find ( PWM_DEV_NAME9 );
//	  if ( pwm_dev9 != RT_NULL )
//	  {
//			LOG_D( "pwm9 device found" );
//			// 配置PWM通道
//			rt_pwm_set ( pwm_dev9 , MOTOR_RF_IN , PERIOD , 0 );
//			ret = rt_pwm_enable ( pwm_dev9 , MOTOR_RF_IN );
//			if ( ret != RT_EOK )
//			{
//				  LOG_E( "ret=%d => rt_pwm_enable ( pwm_dev9 , MOTOR_RF_IN )" , ret );
//			}
//			// 配置刹车通道
//			rt_pwm_set ( pwm_dev9 , MOTOR_RF_BREAK , PERIOD , PERIOD + 1 );
//			ret = rt_pwm_enable ( pwm_dev9 , MOTOR_RF_BREAK );
//			if ( ret != RT_EOK )
//			{
//				  LOG_E( "ret=%d => rt_pwm_enable ( pwm_dev9 , MOTOR_RF_BREAK )" , ret );
//			}
//	  }
//	  else
//	  {
//			LOG_D( "No pwm device found" );
//	  }
//
//	  pwm_dev10 = (struct rt_device_pwm*) rt_device_find ( PWM_DEV_NAME10 );
//	  if ( pwm_dev10 != RT_NULL )
//	  {
//			LOG_D( "pwm10 device found" );
//			// 配置刹车通道
//			rt_pwm_set ( pwm_dev10 , MOTOR_RF_BREAK , PERIOD , PERIOD + 1 );
//			ret = rt_pwm_enable ( pwm_dev10 , MOTOR_RB_BREAK );
//			if ( ret != RT_EOK )
//			{
//				  LOG_E( "ret=%d => rt_pwm_enable ( pwm_dev10 , MOTOR_RB_BREAK )" , ret );
//			}
//	  }
//	  else
//	  {
//			LOG_D( "No pwm device found" );
//	  }
//
//	  pwm_dev11 = (struct rt_device_pwm*) rt_device_find ( PWM_DEV_NAME11 );
//	  if ( pwm_dev11 != RT_NULL )
//	  {
//			LOG_D( "pwm11 device found" );
//			// 配置PWM通道
//			rt_pwm_set ( pwm_dev11 , MOTOR_RB_IN , PERIOD , 0 );
//			ret = rt_pwm_enable ( pwm_dev11 , MOTOR_RB_IN );
//			if ( ret != RT_EOK )
//			{
//				  LOG_E( "ret=%d => rt_pwm_enable ( pwm_dev11 , MOTOR_RB_IN )" , ret );
//			}
//	  }
//	  else
//	  {
//			LOG_D( "No pwm11 device found" );
//	  }
}

rt_uint32_t motor_pwm_limit ( rt_uint32_t data )
{
	  //做限制
	  if ( data <= 5 )
	  {
			return 0;
	  }
	  else if ( data >= 95 )
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

void motor_move_backward ( rt_uint32_t duty_lf , rt_uint32_t duty_lb , rt_uint32_t duty_rf , rt_uint32_t duty_rb )
{
#ifdef PID_OFF
	  rt_uint32_t pulse_lf = PERIOD * motor_pwm_limit ( duty_lf ) / 100;
	  rt_uint32_t pulse_lb = PERIOD * motor_pwm_limit ( duty_lb ) / 100;
	  rt_uint32_t pulse_rf = PERIOD * motor_pwm_limit ( duty_rf ) / 100;
	  rt_uint32_t pulse_rb = PERIOD * motor_pwm_limit ( duty_rb ) / 100;

//	  motor_break ( 0 );  //不刹车

      //方向控制
      rt_pin_write ( MOTOR_DIR_R , PIN_LOW );  //右后
      rt_pin_write ( MOTOR_DIR_L , PIN_HIGH );  //左后

      // RIGHT
      rt_pwm_set ( pwm_dev1 , MOTOR_R_IN , PERIOD , pulse_rf );

      // LEFT
      rt_pwm_set ( pwm_dev1 , MOTOR_L_IN , PERIOD , pulse_lf );
#else
//	  motor_break ( 0 );  //不刹车
	  /**************正反转信号*******************/
	  pid_motor.PID_target_sign_br = 1;
	  pid_motor.PID_target_sign_fr = 1;
	  pid_motor.PID_target_sign_bl = 1;
	  pid_motor.PID_target_sign_fl = 1;

	  /**************目标值信号*******************/
	  pid_motor.PID_target_br = MIX_TARGET * motor_pwm_limit( duty_rb )/100;
	  pid_motor.PID_target_fr = MIX_TARGET * motor_pwm_limit( duty_rf )/100;
	  pid_motor.PID_target_bl = MIX_TARGET * motor_pwm_limit( duty_lb )/100;
	  pid_motor.PID_target_fl = MIX_TARGET * motor_pwm_limit( duty_lf )/100;

#endif

}

void motor_move_forward ( rt_uint32_t duty_lf , rt_uint32_t duty_lb , rt_uint32_t duty_rf , rt_uint32_t duty_rb )
{
#ifdef PID_OFF
	  rt_uint32_t pulse_lf = PERIOD * motor_pwm_limit ( duty_lf ) / 100;
	  rt_uint32_t pulse_lb = PERIOD * motor_pwm_limit ( duty_lb ) / 100;
	  rt_uint32_t pulse_rf = PERIOD * motor_pwm_limit ( duty_rf ) / 100;
	  rt_uint32_t pulse_rb = PERIOD * motor_pwm_limit ( duty_rb ) / 100;

//	  motor_break ( 0 );  //不刹车

	  //方向控制
	  rt_pin_write ( MOTOR_DIR_R , PIN_HIGH );  //右前
	  rt_pin_write ( MOTOR_DIR_L , PIN_LOW );  //左前


	  // RIGHT
	  rt_pwm_set ( pwm_dev1 , MOTOR_R_IN , PERIOD , pulse_rf );

	  // LEFT
	  rt_pwm_set ( pwm_dev1 , MOTOR_L_IN , PERIOD , pulse_lf );


#else
//	  motor_break ( 0 );  //不刹车
	  pid_motor.PID_target_sign_br = 0;
	  pid_motor.PID_target_sign_fr = 0;
	  pid_motor.PID_target_sign_bl = 0;
	  pid_motor.PID_target_sign_fl = 0;

	  pid_motor.PID_target_br = MIX_TARGET * motor_pwm_limit( duty_rb )/100;
	  pid_motor.PID_target_fr = MIX_TARGET * motor_pwm_limit( duty_rf )/100;
	  pid_motor.PID_target_bl = MIX_TARGET * motor_pwm_limit( duty_lb )/100;
	  pid_motor.PID_target_fl = MIX_TARGET * motor_pwm_limit( duty_lf )/100;

#endif
}

void motor_move_right ( rt_uint32_t duty_lf , rt_uint32_t duty_lb , rt_uint32_t duty_rf , rt_uint32_t duty_rb )
{
#ifdef PID_OFF
	  rt_uint32_t pulse_lf = PERIOD * motor_pwm_limit ( duty_lf ) / 100;
	  rt_uint32_t pulse_lb = PERIOD * motor_pwm_limit ( duty_lb ) / 100;
	  rt_uint32_t pulse_rf = PERIOD * motor_pwm_limit ( duty_rf ) / 100;
	  rt_uint32_t pulse_rb = PERIOD * motor_pwm_limit ( duty_rb ) / 100;

//	  motor_break ( 0 );  //不刹车

      //方向控制
      rt_pin_write ( MOTOR_DIR_R , PIN_LOW );  //右后
      rt_pin_write ( MOTOR_DIR_L , PIN_LOW );  //左前

      // RIGHT
      rt_pwm_set ( pwm_dev1 , MOTOR_R_IN , PERIOD , pulse_rf );

      // LEFT
      rt_pwm_set ( pwm_dev1 , MOTOR_L_IN , PERIOD , pulse_lf );
#else
//	  motor_break ( 0 );  //不刹车
	  pid_motor.PID_target_sign_br = 1;
	  pid_motor.PID_target_sign_fr = 1;
	  pid_motor.PID_target_sign_bl = 0;
	  pid_motor.PID_target_sign_fl = 0;

	  if(abs(IMU_updata.imu_data[0]) >= 18.0f || abs(IMU_updata.imu_data[1]) >= 18.0f)
	  {
	      pid_motor.PID_target_br = 0.7 * MIX_TARGET * motor_pwm_limit( duty_rb )/100;
	      pid_motor.PID_target_fr = 0.7 * MIX_TARGET * motor_pwm_limit( duty_rf )/100;
	      pid_motor.PID_target_bl = 0.7 * MIX_TARGET * motor_pwm_limit( duty_lb )/100;
	      pid_motor.PID_target_fl = 0.7 * MIX_TARGET * motor_pwm_limit( duty_lf )/100;
	  }
	  else {
	      pid_motor.PID_target_br = 0.3 * MIX_TARGET * motor_pwm_limit( duty_rb )/100;
	      pid_motor.PID_target_fr = 0.3 * MIX_TARGET * motor_pwm_limit( duty_rf )/100;
	      pid_motor.PID_target_bl = 0.3 * MIX_TARGET * motor_pwm_limit( duty_lb )/100;
	      pid_motor.PID_target_fl = 0.3 * MIX_TARGET * motor_pwm_limit( duty_lf )/100;
	  }

#endif
}

void motor_move_left ( rt_uint32_t duty_lf , rt_uint32_t duty_lb , rt_uint32_t duty_rf , rt_uint32_t duty_rb )
{
#ifdef PID_OFF
	  rt_uint32_t pulse_lf = PERIOD * motor_pwm_limit ( duty_lf ) / 100;
	  rt_uint32_t pulse_lb = PERIOD * motor_pwm_limit ( duty_lb ) / 100;
	  rt_uint32_t pulse_rf = PERIOD * motor_pwm_limit ( duty_rf ) / 100;
	  rt_uint32_t pulse_rb = PERIOD * motor_pwm_limit ( duty_rb ) / 100;

//	  motor_break ( 0 );  //不刹车

      //方向控制
      rt_pin_write ( MOTOR_DIR_R , PIN_HIGH );  //右前
      rt_pin_write ( MOTOR_DIR_L , PIN_HIGH );  //左后

      // RIGHT
      rt_pwm_set ( pwm_dev1 , MOTOR_R_IN , PERIOD , pulse_rf );

      // LEFT
      rt_pwm_set ( pwm_dev1 , MOTOR_L_IN , PERIOD , pulse_lf );
#else
//	  motor_break ( 0 );  //不刹车
	  pid_motor.PID_target_sign_br = 0;
	  pid_motor.PID_target_sign_fr = 0;
	  pid_motor.PID_target_sign_bl = 1;
	  pid_motor.PID_target_sign_fl = 1;

	  if(abs(IMU_updata.imu_data[0]) >= 18.0f || abs(IMU_updata.imu_data[1]) >= 18.0f)
	  {
          pid_motor.PID_target_br = 0.7 * MIX_TARGET * motor_pwm_limit( duty_rb )/100;
          pid_motor.PID_target_fr = 0.7 * MIX_TARGET * motor_pwm_limit( duty_rf )/100;
          pid_motor.PID_target_bl = 0.7 * MIX_TARGET * motor_pwm_limit( duty_lb )/100;
          pid_motor.PID_target_fl = 0.7 * MIX_TARGET * motor_pwm_limit( duty_lf )/100;
	  }
	  else {
	      pid_motor.PID_target_br = 0.3 * MIX_TARGET * motor_pwm_limit( duty_rb )/100;
	      pid_motor.PID_target_fr = 0.3 * MIX_TARGET * motor_pwm_limit( duty_rf )/100;
	      pid_motor.PID_target_bl = 0.3 * MIX_TARGET * motor_pwm_limit( duty_lb )/100;
	      pid_motor.PID_target_fl = 0.3 * MIX_TARGET * motor_pwm_limit( duty_lf )/100;
	  }

#endif
}
void motor_move_stop(void)
{
    motor_move_forward ( 0 , 0 , 0 , 0 );
}
void motor_encoder_read ( rt_int32_t *rpm )
{
//	  rt_device_read ( motor_lf.encoder_dev , 0 , &rpm [ 0 ] , 1 );
//	  rt_device_read ( motor_rf.encoder_dev , 0 , &rpm [ 1 ] , 1 );
//	  rt_device_read ( motor_lb.encoder_dev , 0 , &rpm [ 2 ] , 1 );
//	  rt_device_read ( motor_rb.encoder_dev , 0 , &rpm [ 3 ] , 1 );
}

void motor_encoder_reset ( )
{
//	  rt_device_control ( motor_lf.encoder_dev , PULSE_ENCODER_CMD_CLEAR_COUNT , RT_NULL );
//	  rt_device_control ( motor_rf.encoder_dev , PULSE_ENCODER_CMD_CLEAR_COUNT , RT_NULL );
//	  rt_device_control ( motor_lb.encoder_dev , PULSE_ENCODER_CMD_CLEAR_COUNT , RT_NULL );
//	  rt_device_control ( motor_rb.encoder_dev , PULSE_ENCODER_CMD_CLEAR_COUNT , RT_NULL );

}

extern volatile rt_bool_t protect_flag;  //被底部监管保护的值
void motor_get_duty ( int16_t speed , int16_t steer )
{
	  uint16_t speed_abs = abs ( speed );
	  uint16_t steer_abs = abs ( steer );

//	  //后加往下
//	  if ( protect_flag == RT_TRUE )
//	  {
//			motor_state.duty_l = 0;
//			motor_state.duty_r = 0;
////			LOG_W( "处于保护状态，被系统接管，禁止速度输出：protect_flag == RT_TRUE" );
//			return;
//	  }

	  //后加往上

	  if ( speed_abs > 100 )
	  {
			speed_abs = 100;
	  }

	  if ( steer_abs > 100 )
	  {
			steer_abs = 100;
	  }

	  // start insert filter
//	  smoothedValue_speed = filter ( speed_abs , weight_speed , prevSmoothedValue_speed );
//	  smoothedValue_angular = filter ( steer_abs , weight_angular , prevSmoothedValue_angular );
//	  prevSmoothedValue_speed = smoothedValue_speed;
//	  prevSmoothedValue_angular = smoothedValue_angular;

	  smoothedValue_speed = speed_abs;//取消手柄滤波
	  smoothedValue_angular = steer_abs;

	  // end insert filter

      if(speed_abs == 0 && steer_abs == 0)
      {
          smoothedValue_speed = 0;
          smoothedValue_angular = 0;
      }

	  if ( steer > 5 )
	  {
			motor_state.duty_l = smoothedValue_speed * (100 + coeff_angular * smoothedValue_angular) / 100;
			motor_state.duty_r = smoothedValue_speed * (100 - coeff_angular * smoothedValue_angular) / 100;
//			motor_state.duty_l = motor_state.duty_l > 100 ? 100 : motor_state.duty_l;
			motor_state.duty_l = motor_state.duty_l > 95 ? 100 : motor_state.duty_l;

			if ( speed > 5 )
			{
				  motor_state.dir = FORWARD;
			}
			else if ( speed < -5 )
			{
				  motor_state.dir = BACKWARD;
			}
			else
			{
				  if ( steer_abs > 10 )
				  {
						motor_state.dir = RIGHT;
						motor_state.duty_l = coeff_angular * smoothedValue_angular;
						motor_state.duty_r = coeff_angular * smoothedValue_angular;
				  }
			}
	  }
	  else if ( steer < -5 )
	  {

			motor_state.duty_l = smoothedValue_speed * (100 - coeff_angular * smoothedValue_angular) / 100;
			motor_state.duty_r = smoothedValue_speed * (100 + coeff_angular * smoothedValue_angular) / 100;
//			motor_state.duty_r = motor_state.duty_r > 100 ? 100 : motor_state.duty_r;
			motor_state.duty_r = motor_state.duty_r > 95 ? 100 : motor_state.duty_r;

			if ( speed > 5 )
			{
				  motor_state.dir = FORWARD;
			}
			else if ( speed < -5 )
			{
				  motor_state.dir = BACKWARD;
			}
			else
			{
				  if ( steer_abs > 10 )
				  {
						motor_state.dir = LEFT;
						motor_state.duty_l = coeff_angular * smoothedValue_angular;
						motor_state.duty_r = coeff_angular * smoothedValue_angular;
				  }
			}
	  }
	  else
	  {
			if ( speed > 5 ) motor_state.dir = FORWARD;
			else if ( speed < -5 ) motor_state.dir = BACKWARD;

			motor_state.duty_l = smoothedValue_speed + coeff_angular * smoothedValue_angular;
			motor_state.duty_r = smoothedValue_speed + coeff_angular * smoothedValue_angular;

			//后加
			motor_state.duty_l = motor_state.duty_l > 95 ? 100 : motor_state.duty_l;
			motor_state.duty_r = motor_state.duty_r > 95 ? 100 : motor_state.duty_r;
	  }
	  //LOG_D("duty_l %d, duty_r: %d", motor_state.duty_l, motor_state.duty_r);
}

void motor_contorl ( rt_uint8_t mode )
{
    rt_uint32_t received_flags = 0;
    static uint8_t mag_calib = 0;
    if(imu_event != RT_NULL)
    {
        if (rt_event_recv(imu_event, IMU_CALIB_MOTOR_CONTROL_START | IMU_CALIB_MOTOR_CONTROL_DONE,
                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_NO, &received_flags) == RT_EOK)
        {
            if(received_flags & IMU_CALIB_MOTOR_CONTROL_START)
                mag_calib = 1;
            if(received_flags & IMU_CALIB_MOTOR_CONTROL_DONE)
                mag_calib = 0;
        }
    }

	  //后加往下
	  if ( protect_flag == RT_TRUE )
	  {
			motor_state.duty_l = 0;
			motor_state.duty_r = 0;
//			LOG_W( "处于保护状态，被系统接管，禁止速度输出：protect_flag == RT_TRUE" );
			return;
	  }
	  if(mag_calib)//磁力计校准模式启动
	  {
	      motor_move_right(70,70,70,70);
	      coeff_fl =1;
	      coeff_fr =-1;
	      coeff_bl =1;
	      coeff_br =-1;
//	      LOG_W("MOTOR_CONTROL_MAG_calib_mode");
	      return;
	  }

	  //后加往上
	  motor_get_duty ( robot_state.speed , robot_state.steer );

//	  robot_state.break_mode = mode;			                          //0：滑行模式  1：慢刹车模式 2：急刹车模式
//	  LOG_I("speed : %d ,steer : %d ",robot_state.speed,robot_state.steer);
//	  if ( robot_state.status == 3 )
//	  {
//			motor_state.dir = STOP;
//	  }

	  switch ( motor_state.dir )
	  {
			case STOP :
//			    motor_move_forward ( 0 , 0 , 0 , 0 );
//				  motor_break ( robot_state.break_mode );
//				  bzero(&path,sizeof(&path));
//				  path.path_bl=0;
//				  path.path_br=0;
//				  path.path_fl=0;
//				  path.path_fr=0;
//				  path.path_differ_b=0;
//				  path.path_differ_f=0;
			    motor_move_stop();
//			    LOG_I("STOP");
				  break;
			case FORWARD :
			    coeff_fl =1;
			    coeff_fr =1;
			    coeff_bl =1;
			    coeff_br =1;
				  motor_move_forward ( motor_state.duty_l , motor_state.duty_l , motor_state.duty_r ,
					          motor_state.duty_r );
//				  LOG_I("FORWARD");

				  if(robot_state.speed==0&&robot_state.steer==0)
				  {
//				      path.path_bl=0;
//				      path.path_br=0;
//				      path.path_fl=0;
//				      path.path_fr=0;
//				      path.path_differ_b=0;
//				      path.path_differ_f=0;
				  }
				  break;
			case BACKWARD :
				  motor_move_backward ( motor_state.duty_l , motor_state.duty_l , motor_state.duty_r ,
					          motor_state.duty_r );
				  coeff_fl =-1;
				  coeff_fr =-1;
				  coeff_bl =-1;
				  coeff_br =-1;
//				  LOG_I("BACKWARD");
				  break;
			case LEFT :
				  motor_move_left ( motor_state.duty_l , motor_state.duty_l , motor_state.duty_r , motor_state.duty_r );
				  coeff_fl =-1;
				  coeff_fr =1;
				  coeff_bl =-1;
				  coeff_br =1;
//				  LOG_I("LEFT");
				  break;
			case RIGHT :
				  motor_move_right ( motor_state.duty_l , motor_state.duty_l , motor_state.duty_r ,
					          motor_state.duty_r );
				  coeff_fl =1;
				  coeff_fr =-1;
				  coeff_bl =1;
				  coeff_br =-1;
//				  LOG_I("RIGHT");
				  break;
	  }

	  //			LOG_D( "RPM: % 8X  % 8X  % 8X  % 8X = % 7d  % 7d  % 7d  % 7d," , robot_state.rpm [ 0 ] ,
	  //			            robot_state.rpm [ 1 ] , robot_state.rpm [ 2 ] , robot_state.rpm [ 3 ] , robot_state.rpm [ 0 ] ,
	  //			            robot_state.rpm [ 1 ] , robot_state.rpm [ 2 ] , robot_state.rpm [ 3 ] );
}

void motor_pwm_test(void)
{
    rt_pin_mode( MOTOR_PWM_R , PIN_MODE_OUTPUT );
    rt_pin_mode( MOTOR_PWM_L , PIN_MODE_OUTPUT );
    rt_pin_write ( MOTOR_PWM_R , PIN_LOW );
    rt_pin_write ( MOTOR_PWM_L , PIN_LOW );
}

void motor_thread_entry ( void *parameter )
{
//      motor_pwm_test();
	  motor_init_pin ( );
	  motor_init_pwm ( );
//	  motor_init_encoder ( );//可省略
	  MX_TIM4_Init();
	  MX_TIM5_Init();
//	  rt_uint8_t speed = 100;
//	  rt_uint32_t add = 0;
//	  get_variable_acceleration(0,100,1);

	  while ( 1 )
	  {

	        if ( robot_state.pwr == 1 )
			{
				  motor_contorl ( 1 );	  //刹车模式
			}
			else
			{
			    motor_move_stop();
//				  motor_break ( 1 );	  //主要是接着Jlink电源等调试时，系统未断电的处理
			}


//	      motor_move_right( 50 , 50 , 50 , 50 );

//	      //测试用
//	      if ( robot_state.pwr == 1 )
//	      {
//	          motor_move_forward ( speed , speed , speed , speed );
//	          rt_thread_mdelay ( 3000 );
//	          motor_move_backward ( speed , speed , speed , speed );
//	          rt_thread_mdelay ( 2990 );
//	          add++;
//	      }
//
//	      rt_kprintf("add=%d\n",add);


			// PWM
//			rt_pwm_set ( pwm_dev9 , MOTOR_RF_IN , PERIOD , PERIOD * 0.8 );
//			rt_pwm_set ( pwm_dev11 , MOTOR_RB_IN , PERIOD , PERIOD * 0.8 );
//			rt_pwm_set ( pwm_dev1 , MOTOR_LF_IN , PERIOD , PERIOD * 0.8 );
//			rt_pwm_set ( pwm_dev1 , MOTOR_LB_IN , PERIOD , PERIOD * 0.8 );

			//BREAK
//			rt_pwm_set ( pwm_dev1 , MOTOR_LF_BREAK , PERIOD , PERIOD * 0.8 );
//			rt_pwm_set ( pwm_dev1 , MOTOR_LB_BREAK , PERIOD , PERIOD * 0.8 );
//			rt_pwm_set ( pwm_dev9 , MOTOR_RF_BREAK , PERIOD , PERIOD * 0.8 );
//			rt_pwm_set ( pwm_dev10 , MOTOR_RF_BREAK , PERIOD , PERIOD * 0.8 );

			rt_thread_mdelay ( 10 );
	  }
}
