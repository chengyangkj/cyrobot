#include "ros.h"
#include "ros/time.h"
//编码器控制头文件
#include "encoder.h"
//电机控制头文件
#include "motor_driver.h"
//header file for publishing velocities for odom
#include "cyrobot_msgs/Velocities.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header file for pid server
#include "cyrobot_msgs/PID.h"
//header file for imu
#include "cyrobot_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "PID.h"
#include "Imu.h"

//导入运动学库
#include "Kinematics.h"
//导入机器基本配置文件
#include "robot_config.h"

//电源电压发布频率
//IMU发布的频率
#define SENSOR_PUBLISH_RATE 10 //hz
//IMU发布的频率
#define IMU_PUBLISH_RATE 20 //hz
//命令发布的频率
#define COMMAND_RATE 20 //hz
//调试的频率
#define DEBUG_RATE 5
PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

//初始化运动学分析
Kinematics kinematics( MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);


//初始化x y z 速度和上一个命令的时间变量
float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;
int FanState=0;
unsigned long g_prev_command_time = 0;
//速度控制命令的回调函数
void commandCallback(const geometry_msgs::Twist& cmd_msg);
//PID的回调函数
void PIDCallback(const cyrobot_msgs::PID& pid);
//Sensor的回调函数
void sensorCallback(const std_msgs::String& msg);
//ros的控制句柄
ros::NodeHandle nh;
//订阅速度控制话题
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
//订阅PID话题
ros::Subscriber<cyrobot_msgs::PID> pid_sub("pid", PIDCallback);
//订阅传感器控制话题
ros::Subscriber<std_msgs::String> sensor_sub("sensor_control", sensorCallback);
//发布电源
std_msgs::String sensor_msg;
ros::Publisher sensor_pub("sensor",&sensor_msg);
//创建IMu发布结点
cyrobot_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
//速度发布结点
cyrobot_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
void setup() {
        //初始化蜂鸣器
    pinMode(SOUND_PIN,OUTPUT);
    //初始化温度传感器输入引脚
     pinMode(TEMP_PIN,INPUT);
     //初始化风扇PWM
     pinMode(FAN_PIN,OUTPUT);
    //发出通电声音
    Power_On_Sound();
    //电压检测模块引脚
    pinMode(POWER_PIN,INPUT);
    //初始化编码器
    initEncoder();
    //初始化电机
    initMotor();
    stopBase();
    //初始化结点
    nh.initNode();
    //绑定波特率
    nh.getHardware()->setBaud(57600);
    //订阅话题
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
     nh.subscribe(sensor_sub);
    //注册发布的话题
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
       nh.advertise(sensor_pub);
    //循环等待连接ros上位机
      while (!nh.connected())
    {
        //发出上位机未连接的声音
       DisConnect_Sound();
        delay(1000);
        nh.spinOnce();
    }
    //发出上位机连接的声音
    Connect_Sound();
    nh.loginfo("CHENGYANGKJ BASE CONNECTED");
    //    Serial.begin(9600);
    
    delay(1);
//    
}
void loop() {

 static unsigned long prev_control_time = 0;
 static unsigned long prev_imu_time = 0;
  static unsigned long prev_sensor_time = 0;
 static unsigned long prev_debug_time = 0;
 static bool imu_is_initialized;

 //当达到设定的频率进行运动
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  {
      moveBase();
      prev_control_time = millis();
  }
 //当未收到命令时停止运动
  if ((millis() - g_prev_command_time) >= 400)
  {
      stopBase();
  }
  //达到发布传感器数据的频率发布
   if ((millis() - prev_sensor_time) >= (1000 / SENSOR_PUBLISH_RATE))
  {
      publish_sensor();
      prev_sensor_time = millis();
  }
  #ifdef USE_MPU6050_IMU
  //当达到imu的发布频率发布imu信息
  if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    //  sanity check if the IMU is connected
      if (!imu_is_initialized)
      {
          imu_is_initialized = initIMU();

          if(imu_is_initialized)
              nh.loginfo("IMU Initialized");
          else
              nh.logfatal("IMU failed to initialize. Check your IMU connection.");
      }
      else
      {
          publishIMU();
      }
      prev_imu_time = millis();
  }
  #endif

  //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            printDebug();
            prev_debug_time = millis();
        }
    }
    //call all the callbacks waiting to be called
     nh.spinOnce();

    //检查是否与上位机断开连接
        while (!nh.connected())
        {
            //发出上位机未连接的声音
            DisConnect_Sound();
            delay(1000);
            nh.spinOnce();
        }
}
//发布传感器
void publish_sensor()
{
    //读取温度值
    double val;
    double  temperature;
    val = analogRead(TEMP_PIN);  //读取模拟量
    temperature = (val-32)/1.8; //转化为温度
    temperature=25+37-temperature;
    //读取电压值
    String battery_val=String(analogRead(POWER_PIN)/40.92);
    String sensor_data="Temp:"+String(temperature)+":Battery:"+battery_val+":FanState:"+String(FanState);
    sensor_msg.data=sensor_data.c_str();
    sensor_pub.publish(&sensor_msg);
}
void PIDCallback(const cyrobot_msgs::PID& pid)
{
    //callback function every time PID constants are received from cyrobot_pid for tuning
    //this callback receives pid object where P,I, and D constants are stored
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
    motor4_pid.updateConstants(pid.p, pid.i, pid.d);
}
void sensorCallback(const std_msgs::String& msg)
{
  switch (msg.data[0])
  {
      case 'F':
          digitalWrite(FAN_PIN,msg.data[1]-'0');
          FanState=msg.data[1]-'0';
          break;
    case 'T':
         tone( SOUND_PIN, 500-10*15, 1000);
         break;
      default:
         break;
  }
  
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    
    
    g_prev_command_time = millis();
}

void moveBase()
{
     
    //根据当前控制消息的速度计算得到每个轮子需要设定的速度
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    //get the current speed of each motor
    long int current_rpm1 = getRPM(1);
    long int current_rpm2 = getRPM(2);
    long int current_rpm3 = getRPM(3);
    long int current_rpm4 = getRPM(4);

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM

    
    runMotor(1,motor1_pid.compute(req_rpm.motor1, current_rpm1));
    runMotor(2,motor2_pid.compute(req_rpm.motor2, current_rpm2));
    runMotor(3,motor3_pid.compute(req_rpm.motor3, current_rpm3));  
    runMotor(4,motor4_pid.compute(req_rpm.motor4, current_rpm4));    
  char buffer[50];
   if(DEBUG)
    {
        if(current_rpm1!=0)
        {
        
    
        sprintf (buffer, "curr_rpm1  : %ld",current_rpm1);
        nh.loginfo(buffer);
        sprintf (buffer, "curr_rpm3 : %ld",current_rpm2);
        nh.loginfo(buffer);
        sprintf (buffer, "curr_rpm3   : %ld", current_rpm3);
        nh.loginfo(buffer);
        sprintf (buffer, "curr_rpm4  : %ld",current_rpm4);
        nh.loginfo(buffer);
    
        }
        if(req_rpm.motor1!=0)
        {
        sprintf (buffer, "set_rpm1  : %ld",long(req_rpm.motor1));
        nh.loginfo(buffer);
        sprintf (buffer, "set_rpm2 : %ld", long(req_rpm.motor2));
        nh.loginfo(buffer);
        sprintf (buffer, "set_rpm3   : %ld", long(req_rpm.motor3));
        nh.loginfo(buffer);
        sprintf (buffer, "set_rpm4  : %ld", long(req_rpm.motor4));
        nh.loginfo(buffer);  
        }
        if(motor1_pid.compute(req_rpm.motor1, current_rpm1)!=0)
        {
          sprintf (buffer, "pid_pwm1  : %ld",long(motor1_pid.compute(req_rpm.motor1, current_rpm1)));
        nh.loginfo(buffer);
        sprintf (buffer, "pid_pwm2 : %ld",long(motor2_pid.compute(req_rpm.motor2, current_rpm2)));
        nh.loginfo(buffer);
        sprintf (buffer, "pid_pwm3   : %ld", long(motor3_pid.compute(req_rpm.motor3, current_rpm3)));
        nh.loginfo(buffer);
        sprintf (buffer, "pid_pwm4  : %ld",long(motor4_pid.compute(req_rpm.motor4, current_rpm4)));
        nh.loginfo(buffer);
        }

    }
    Kinematics::velocities current_vel;

   
   current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
 
    
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;
      
    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
    //电机刹车
   runMotor(1,999);
    runMotor(2,999);
    runMotor(3,999);
    runMotor(4,999);
}

void publishIMU()
{
    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readGyroscope();

    //pass accelerometer data to imu object
    raw_imu_msg.magnetic_field = readMagnetometer();

    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
}



float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printDebug()
{
//    char buffer[50];
//    
//    sprintf (buffer, "Encoder FrontLeft  : %ld", readEncoder(1));
//    nh.loginfo(buffer);
//    sprintf (buffer, "Encoder FrontRight : %ld", readEncoder(2));
//    nh.loginfo(buffer);
//    sprintf (buffer, "Encoder RearLeft   : %ld", readEncoder(3));
//    nh.loginfo(buffer);
//    sprintf (buffer, "Encoder RearRight  : %ld", readEncoder(4));
//    nh.loginfo(buffer);
    
}
//蜂鸣器相关函数
//上位机连接上时声音
void Connect_Sound()
{
   for(int i=0;i<5;i++)
 {
  tone( SOUND_PIN, 500, 100);
  delay(250);
  noTone( SOUND_PIN);
 }
}
//上位机断开时声音
void DisConnect_Sound()
{
  tone( SOUND_PIN, 500, 100);
}
//通电时声音
void Power_On_Sound()
{
 for(int i=0;i<15;i++)
 {
  tone( SOUND_PIN, 500+i*15, 1000);
   delay(100);
    noTone( SOUND_PIN);
 }
}
//断电时声音
void Power_Off_Sound()
{
 for(int i=0;i<15;i++)
 {
  tone( SOUND_PIN, 500-i*15, 1000);
   delay(100);
    noTone( SOUND_PIN);
 }
}
