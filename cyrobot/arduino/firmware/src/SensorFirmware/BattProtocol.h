#ifndef KINEMATICS_H
#define KINEMATICS_H
struct BattStatus
{
    unsigned long  Voltage_data;
    unsigned long  Current_data;
    unsigned long Power_data;
    unsigned long Energy_data;
     unsigned long Pf_data;
      unsigned long CO2_data; 
};

class BattProtocol
{
private:
    /* data */
    unsigned char Tx_Buffer[8];
    unsigned char read_enable,receive_finished,reveive_number=37;
    unsigned long Voltage_data,Current_data,Power_data,Energy_data,Pf_data,CO2_data; 
public:
    BattProtocol(/* args */);
    ~BattProtocol();
    unsigned int calccrc(unsigned char crcbuf,unsigned int crc);
    unsigned int chkcrc(unsigned char *buf,unsigned char len);
    unsigned char*  packData();
    BattStatus unpackData(unsigned char*);
};

#endif
