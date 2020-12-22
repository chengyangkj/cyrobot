#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include  <sensor_msgs/BatteryState.h>
#include <Servo.h>
#include <SimpleDHT.h>
#include "BattProtocol.h"
//
// WiFi Definitions //
//
 
const char* ssid     = "chengyangkj-master-wifi";
const char* password = "chengyangkj";
// Set the rosserial socket server IP address
IPAddress server(10,42,0,79);
IPAddress ip_address;
int status = WL_IDLE_STATUS;
WiFiClient client;
int pinDHT11 = D4;
SimpleDHT11 dht11(pinDHT11);

//重写ROS Nodehandle函数
class WiFiHardware {

  public:
  WiFiHardware() {};

  void init() {
    // 重写init 在init事连接tcp 上位机需要开启rosserial_python
    client.connect(server, 11411);
  }


  int read() {
    //重写读数据 从tcp读
    // implement this method so that it reads a byte from the TCP connection and returns it
    //  you may return -1 is there is an error; for example if the TCP connection is not open
    return client.read();         //will return -1 when it will works
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    //重写 写数据 从tcp写
    // implement this so that it takes the arguments and writes or prints them to the TCP connection
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }

  // returns milliseconds since start of program
  unsigned long time() {
     return millis(); // easy; did this one for you
  }
};

Servo s;
int i;

void chatterCallback(const std_msgs::String& msg) {
  i = atoi(msg.data);
  s.write(i);
}
//声明变量
unsigned char* Tx_Buffer;
unsigned char  Rx_Buffer[37] ;
std_msgs::String str_msg;
sensor_msgs::Temperature temperature_msg;
sensor_msgs::RelativeHumidity humidity_msg;
sensor_msgs::BatteryState batt_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher temperaturePub("temperature",&temperature_msg);
ros::Publisher BatteryStatePub("BatteryState",&batt_msg);
ros::Publisher humidityPub("relativehumidity",&humidity_msg);
ros::Subscriber<std_msgs::String> sub("message", &chatterCallback);
ros::NodeHandle_<WiFiHardware> nh;
BattProtocol battProtocol;
int num=0;

void setupWiFi()
{
  WiFi.begin(ssid, password);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    while(1) delay(500);
  }
 
}

int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<2; i++) v+= analogRead(pin);
  return v/2;
}

void setup() {
  Serial.begin(4800);
  pinMode(D1,OUTPUT);
  // digitalWrite(D1,1);
  setupWiFi();
  delay(2000);
  s.attach(13);  // PWM pin
  nh.initNode();
  //if tcp not connect
if(!client.connected()){
    nh.initNode();
    delay(500);
}
  nh.spinOnce();
  nh.advertise(chatter);
  nh.advertise(temperaturePub);
  nh.advertise(humidityPub);
  nh.advertise(BatteryStatePub);
  nh.subscribe(sub);


}
void pubTemper(){
  byte temperature = 0;
  byte humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht11.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    return;
  }
 temperature_msg.temperature=(int)temperature;
 humidity_msg.relative_humidity=(int)humidity;
  temperaturePub.publish(&temperature_msg);
  humidityPub.publish(&humidity_msg);
  
}
void pubBattStatus(){
        Tx_Buffer= battProtocol.packData();
        for(int i=0;i<8;i++){
          Serial.write(Tx_Buffer[i]);
        }
          int num=Serial.available();
       
                              char string[25];  
        itoa(num, string, 10);  
        str_msg.data =string ;
        chatter.publish( &str_msg );
//        if(Serial.available()){
          
       
//           if(Rx_Buffer[0]==0x01){
//             for(int i=1;i<37;i++){
//                 Rx_Buffer[i]=Serial.read();
//                 str_msg.data =(char*)Rx_Buffer[i] ;
//                chatter.publish( &str_msg );
//             }
//           }
//        }
//        BattStatus status=battProtocol.unpackData(Rx_Buffer);
//        batt_msg.voltage=status.Voltage_data;
//        batt_msg.current=status.Current_data;
//        //batt_msg.  
//       BatteryStatePub.publish(&batt_msg);                                                                              
//        char string[25];  
//        itoa(num, string, 10);  
//        str_msg.data =string ;
//        chatter.publish( &str_msg );
    
}
void loop() {

  //如果tcp没有链接上则循环等待链接
if(!client.connected()){
    nh.initNode();
    delay(500);
}

  pubTemper();
  delay(500);
  pubBattStatus();
  nh.spinOnce();
  delay( 10 );
}
