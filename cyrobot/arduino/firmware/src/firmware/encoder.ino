volatile long countA = 0L; //A编码器计数变量
volatile long countB = 0L; //B编码器计数变量
volatile long countC = 0L; //C编码器计数变量
volatile long countD = 0L; //D编码器计数变量

unsigned long timeA = 0; // 时间标记
unsigned long timeB = 0; // 时间标记
unsigned long timeC = 0; // 时间标记
unsigned long timeD = 0; // 时间标记
//A编码器本次记录时间
unsigned long Aprev_update_time_=0;
//A编码器本次记录个数
long Aprev_encoder_ticks_=0;

//B编码器本次记录时间
unsigned long Bprev_update_time_=0;
//B编码器本次记录个数
long Bprev_encoder_ticks_=0;

//C编码器本次记录时间
unsigned long Cprev_update_time_=0;
//C编码器本次记录个数
long Cprev_encoder_ticks_=0;

//D编码器本次记录时间
unsigned long Dprev_update_time_=0;
//D编码器本次记录个数
long Dprev_encoder_ticks_=0;

//读取编码器读数
long readEncoder(int id)
{
  switch(id)
  {
    case 1:
    return countA;
    break;
    case 2:
    return countB;
    break;
    case 3:
    return countC;
    break;
    case 4:
    return countD;
    break;
  }
}
void resetEncoder()
{
countA = 0L; //A编码器计数变量
countB = 0L; //B编码器计数变量
countC = 0L; //C编码器计数变量
countD = 0L; //D编码器计数变量
}
void initEncoder()
{
pinMode(PINAA,INPUT);
pinMode(PINAB,INPUT);
pinMode(PINBA,INPUT);
pinMode(PINBB,INPUT);
pinMode(PINCA,INPUT);
pinMode(PINCB,INPUT);
pinMode(PINDA,INPUT);
pinMode(PINDB,INPUT);
resetEncoder();
//A B C D 轮编码器A相线的中断
attachInterrupt(digitalPinToInterrupt(PINAA), AEncoderEvent, FALLING);
attachInterrupt(digitalPinToInterrupt(PINBA), BEncoderEvent, FALLING);
attachInterrupt(digitalPinToInterrupt(PINCA), CEncoderEvent, FALLING);
attachInterrupt(digitalPinToInterrupt(PINDA), DEncoderEvent, FALLING);
}
void AEncoderEvent()
{
  //为了不计入噪音干扰脉冲，
//当2次中断之间的时间大于5ms时，计一次有效计数
if ((millis() - timeA) > 5)
{
//当编码器码盘的OUTA脉冲信号下跳沿每中断一次，(判断正反转)
if ((digitalRead(PINAA) == LOW) && (digitalRead(PINAB) == HIGH))
{
countA++;
}
else
  {
  countA--;
  }
}
timeA == millis();
//Serial.print("countA:");
//Serial.println(countA);

}
void BEncoderEvent()
{
  //为了不计入噪音干扰脉冲，
//当2次中断之间的时间大于5ms时，计一次有效计数
if ((millis() - timeB) > 5)
{
  //当编码器码盘的OUTA脉冲信号下跳沿每中断一次，(判断正反转)
  if ((digitalRead(PINBA) == LOW) && (digitalRead(PINBB) == HIGH))
  {
  countB--;
  }
  else
    {
    countB++;
    }
}
timeB == millis();
//Serial.print("countB:");
//Serial.println(countB);
}

void CEncoderEvent()
{
  //为了不计入噪音干扰脉冲，
//当2次中断之间的时间大于5ms时，计一次有效计数
if ((millis() - timeC) > 5)
{
  //当编码器码盘的OUTA脉冲信号下跳沿每中断一次，(判断正反转)
  if ((digitalRead(PINCA) == LOW) && (digitalRead(PINCB) == HIGH))
  {
  countC++;
  }
  else
    {
    countC--;
    }
}
timeC == millis();
//Serial.print("countC:");
//Serial.println(countC);
}

void DEncoderEvent()
{
  //为了不计入噪音干扰脉冲，

//当2次中断之间的时间大于5ms时，计一次有效计数

if ((millis() - timeD) > 5)
{
  //当编码器码盘的OUTA脉冲信号下跳沿每中断一次，(判断正反转)
  if ((digitalRead(PINDA) == LOW) && (digitalRead(PINDB) == HIGH))
  {
  countD--;
  }
  else
    {
    countD++;
    }
}
timeD == millis();
//Serial.print("countD:");
//Serial.println(countD);
}
int getRPM(int id){
    //获取当前每个轮子的转速的转速 转/分
    long encoder_ticks;
    unsigned long current_time;
    unsigned long dt;
    double dtm;
    double delta_ticks;
    switch(id)
    {
      case 1:
          encoder_ticks = countA;
          current_time = millis();
          dt = current_time - Aprev_update_time_;
    
          //convert the time from milliseconds to minutes
          dtm = (double)dt / 60000;
          delta_ticks = encoder_ticks - Aprev_encoder_ticks_;
      
          //calculate wheel's speed (RPM)
      
          Aprev_update_time_ = current_time;
          Aprev_encoder_ticks_ = encoder_ticks;
          
          return (delta_ticks / COUNTS_PER_REV) / dtm;
      break;
      case 2:
          encoder_ticks = countB;
          current_time = millis();
          dt = current_time - Bprev_update_time_;
    
          //convert the time from milliseconds to minutes
          dtm = (double)dt / 60000;
          delta_ticks = encoder_ticks - Bprev_encoder_ticks_;
      
          //calculate wheel's speed (RPM)
      
          Bprev_update_time_ = current_time;
          Bprev_encoder_ticks_ = encoder_ticks;
          
          return (delta_ticks / COUNTS_PER_REV) / dtm;
      break;
      case 3:
          encoder_ticks = countC;
          current_time = millis();
          dt = current_time - Cprev_update_time_;
    
          //convert the time from milliseconds to minutes
          dtm = (double)dt / 60000;
          delta_ticks = encoder_ticks - Cprev_encoder_ticks_;
      
          //calculate wheel's speed (RPM)
      
          Cprev_update_time_ = current_time;
          Cprev_encoder_ticks_ = encoder_ticks;
          
          return (delta_ticks / COUNTS_PER_REV) / dtm;
      break;
      case 4:
          encoder_ticks = countD;
          current_time = millis();
          dt = current_time - Dprev_update_time_;
    
          //convert the time from milliseconds to minutes
          dtm = (double)dt / 60000;
          delta_ticks = encoder_ticks - Dprev_encoder_ticks_;
      
          //calculate wheel's speed (RPM)
      
          Dprev_update_time_ = current_time;
          Dprev_encoder_ticks_ = encoder_ticks;
          
          return (delta_ticks / COUNTS_PER_REV) / dtm;
      break;
    }
    
  }
