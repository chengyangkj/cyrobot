//在这里存放机器人的一些配置信息
#define MAX_RPM 180             // 电机的最大转速 转/分
#define WHEEL_DIAMETER 0.08      //车轮直径（米）
//#define PWM_BITS 8                // 单片机的PWM分辨率
#define LR_WHEELS_DISTANCE 0.37  // 左右轮间距
#define FR_WHEELS_DISTANCE 0.24  // 前后轮间距
//#define MAX_STEERING_ANGLE 0.415  //最大转向角。这仅适用于阿克曼转向系统

//控制电机时 电机接收的PWM的最小值 注意这里需要为负数
#define PWM_MIN -150 
//控制电机时 电机接收的PWM的最大值
#define PWM_MAX 150
#define K_P 0.6 // P constant
#define K_I 0.02 // I constant
#define K_D 0.03 // D constant
#define DEBUG 0
//电压测量引脚
#define POWER_PIN A8
//定义是否使用imu 这里为mpu6050
//#define USE_MPU6050_IMU
//定义蜂鸣器的引脚
#define SOUND_PIN 49
//定义温度传感器数据引脚
#define TEMP_PIN A7
//定义散热器引脚
#define FAN_PIN 11
