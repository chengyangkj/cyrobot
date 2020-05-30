//这里为电机驱动，可以根据自己的情况在函数中进行更改
//A电机
#define STOPA 22 //电机急停引脚
#define DIRA 36  //电机方向引脚
#define PWMA 4   //电机调速引脚
//B电机
#define STOPB 23
#define DIRB 37
#define PWMB 5
//C电机
#define STOPC 24
#define DIRC 38
#define PWMC 6
//D电机
#define STOPD 25
#define DIRD 39
#define PWMD 7
//将自己的驱动程序写到这几个函数里
//初始化电机
void initMotor();
//运转电机 传入电机id 和pwm
void runMotor(int id,int pwm);
//设置单个电机状态
void setMotorStaue(int statue,int st,int dir,int pwm, int spd);

//由于我使用的无刷电机是47-240的pwm速度递减 需要进行转换 将pwm脉冲值转换为电机标准脉冲
int trans_pwm(int pwm);
