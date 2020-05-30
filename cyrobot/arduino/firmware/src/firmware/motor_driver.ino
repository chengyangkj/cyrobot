void initMotor()
{
pinMode(STOPA,OUTPUT);
pinMode(DIRA,OUTPUT);
pinMode(PWMA,OUTPUT);
pinMode(STOPB,OUTPUT);
pinMode(DIRB,OUTPUT);
pinMode(PWMB,OUTPUT);
pinMode(STOPC,OUTPUT);
pinMode(DIRC,OUTPUT);
pinMode(PWMC,OUTPUT);
pinMode(STOPD,OUTPUT);
pinMode(DIRD,OUTPUT);
pinMode(PWMD,OUTPUT);
runMotor(1,999);
runMotor(2,999);
runMotor(3,999);
runMotor(4,999);

}
void runMotor(int id,int pwm)
{
  switch(id)
  {
    case 1:
           if(pwm >=0)
            {
              setMotorStaue(1,STOPA,DIRA,PWMA,pwm);
            }
            else if(pwm < 0)
            {
              setMotorStaue(-1,STOPA,DIRA,PWMA,abs(pwm));  
            }
             if(pwm==999)
            {
                setMotorStaue(0,STOPA,DIRA,PWMA,pwm);
            }
            
    break;
    case 2:
           if(pwm >= 0)
            {
              setMotorStaue(-1,STOPB,DIRB,PWMB,pwm);
            }
            else if(pwm < 0)
            {
              setMotorStaue(1,STOPB,DIRB,PWMB,abs(pwm));  
            }
             if(pwm==999)
            {
                setMotorStaue(0,STOPB,DIRB,PWMB,abs(pwm));  
            }
            
    break;
    case 3:
           if(pwm >= 0)
            {
              setMotorStaue(1,STOPC,DIRC,PWMC,pwm);
            }
            else if(pwm < 0)
            {
              
              setMotorStaue(-1,STOPC,DIRC,PWMC,abs(pwm));  
            }
             if(pwm==999)
            {
                setMotorStaue(0,STOPC,DIRC,PWMC,abs(pwm));  
            }
            
            
    break;
    case 4:
           if(pwm >=0)
            {
              setMotorStaue(-1,STOPD,DIRD,PWMD,pwm);
            }
            else if(pwm < 0)
            {
              setMotorStaue(1,STOPD,DIRD,PWMD,abs(pwm));  
            }
             if(pwm==999)
            {
                setMotorStaue(0,STOPD,DIRD,PWMD,abs(pwm));  
            }
            

            
    break;
  }
}
void setMotorStaue(int statue,int st,int dir,int pwm, int spd)
{
 
  //控制单个轮子运动
  switch (statue)
  {
     case 0:
      digitalWrite(st,0);
      break;
    case 1:
      digitalWrite(st,1);
      digitalWrite(dir,0);
      analogWrite(pwm,trans_pwm(spd));
      break;
    case -1:
     digitalWrite(st,1);
    digitalWrite(dir,1);
     analogWrite(pwm,trans_pwm(spd));
     break;
  }
 
}


int trans_pwm(int pwm)
{ 
  int repwm=pwm;
  if(repwm>=0)
  {
    repwm+=50;
  }
  else if(repwm<0)
  {
    repwm-=50;
  }
  return 250-repwm;
}
