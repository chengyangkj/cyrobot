#include "BattProtocol.h"

BattProtocol::BattProtocol(/* args */)
{
}

BattProtocol::~BattProtocol()
{
}
unsigned int BattProtocol::calccrc(unsigned char crcbuf,unsigned int crc)
{
        unsigned char i;
        unsigned char chk;
        crc=crc ^ crcbuf;
        for(i=0;i<8;i++)
        {
        chk=( unsigned char)(crc&1);
        crc=crc>>1;
        crc=crc&0x7fff;
        if (chk==1)
        crc=crc^0xa001;
        crc=crc&0xffff;
        }
        return crc;
} 
unsigned int BattProtocol::chkcrc(unsigned char *buf,unsigned char len)
{
        unsigned char hi,lo;
        unsigned int i;
        unsigned int crc;
        crc=0xFFFF;
        for(i=0;i<len;i++)
        {
        crc=calccrc(*buf,crc);
        buf++;
        } 
        hi=( unsigned char)(crc%256);
        lo=( unsigned char)(crc/256);
        crc=(((unsigned int)(hi))<<8)|lo;
return crc;
} 

 unsigned char* BattProtocol::packData(){
         union crcdata
        {
        unsigned int word16;
        unsigned char byte[2];
        }crcnow;
        Tx_Buffer[0]= 0x01; //模块的 ID 号，默认 ID 为 0x01
        Tx_Buffer[1]=0x03;
        Tx_Buffer[2]=0x00;
        Tx_Buffer[3]=0x48;
        Tx_Buffer[4]=0x00;
        //08 06
        Tx_Buffer[5]=0x08;
        crcnow.word16=chkcrc(Tx_Buffer,6);
        Tx_Buffer[6]=crcnow.byte[1]; //CRC 效验低字节在前
        Tx_Buffer[7]=crcnow.byte[0];
        return Tx_Buffer;
 }
BattStatus BattProtocol::unpackData(unsigned char* RX_Buffer){
            BattStatus status;
            unsigned char i;
            union crcdata
            {
                unsigned int word16;
                unsigned char byte[2];
            }crcnow;
            if(RX_Buffer[0]==0x01) //确认 ID 正确
            {
                        crcnow.word16=chkcrc(RX_Buffer,reveive_number-2); //reveive_numbe 是接收数据总长度
                        if((crcnow.byte[0]==RX_Buffer[reveive_number-1])&&(crcnow.byte[1]==RX_Buffer[reveive_number-2])) 
                        //确认 CRC 校验正确
                        {
                                        status.Voltage_data=(((unsigned long)(RX_Buffer[3]))<<24)|(((unsigned
                                        long)(RX_Buffer[4]))<<16)|(((unsigned long)(RX_Buffer[5]))<<8)|RX_Buffer[6];
                                        status.Current_data=(((unsigned long)(RX_Buffer[7]))<<24)|(((unsigned
                                        long)(RX_Buffer[8]))<<16)|(((unsigned long)(RX_Buffer[9]))<<8)|RX_Buffer[10];
                                        status.Power_data=(((unsigned long)(RX_Buffer[11]))<<24)|(((unsigned
                                        long)(RX_Buffer[12]))<<16)|(((unsigned long)(RX_Buffer[13]))<<8)|RX_Buffer[14];
                                        status.Energy_data=(((unsigned long)(RX_Buffer[15]))<<24)|(((unsigned
                                        long)(RX_Buffer[16]))<<16)|(((unsigned long)(RX_Buffer[17]))<<8)|RX_Buffer[18];
                                        status.Pf_data=(((unsigned long)(RX_Buffer[19]))<<24)|(((unsigned
                                        long)(RX_Buffer[20]))<<16)|(((unsigned long)(RX_Buffer[21]))<<8)|RX_Buffer[22];
                                        status.CO2_data=(((unsigned long)(RX_Buffer[23]))<<24)|(((unsigned
                                        long)(RX_Buffer[24]))<<16)|(((unsigned long)(RX_Buffer[25]))<<8)|RX_Buffer[26];
                        }
            }
            return status;
}
