/*在这个文件配置编码器信息
需将自己4个电机的编码器A B相引脚号替换为自己的
其中A相必须为外部中断引脚
这里使用的是arduino Mega2560 */
//A电机编码器信息 左一
#define PINAA 2  //A编码器引脚A 中断0
#define PINAB 28 //编码器引脚B
//B电机 右一
#define PINBA 3 //B编码器引脚A 中断1
#define PINBB 29
//C电机 左二
#define PINCA 18 //C编码器引脚A 中断2
#define PINCB 30
//D电机 右二
#define PINDA 19 //D编码器引脚A 中断3
#define PINDB 31

#define COUNTS_PER_REV 120       // 车轮每转一圈编码器的计数
//初始化编码器的函数
void initEncoder();
//ABCD编码器的中断事件
void AEncoderEvent();
void BEncoderEvent();
void CEncoderEvent();
void DEncoderEvent();
//返回ABCD编码器的计数
long readEncoder(int id);
//初始化编码器
void resetEncoder();
