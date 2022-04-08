#include "GYSerialTempSensor.h"


GYSerialTempSensor::GYSerialTempSensor(SharedPointer<AbstractSerial> devcie)
{
    setDev(devcie);
}

ST_Tempture GYSerialTempSensor::getData()
{
    ST_Tempture ret = {0};

    unsigned char i=0,rebuf[30]={0};

    bool Receive_ok = false;
    unsigned char sum = 0;

    while(!Receive_ok)
    {
        int count = this->m_dev->serialDataAvail();

        if(count != -1)//接收完成标志
        {
            rebuf[i++]=this->m_dev->serialGetchar();

            if (rebuf[0]!=add)//帧头不对
                i=0;
            else if ((i==2)&&(rebuf[1]!=0x03))//帧头不对
                i=0;
            else if ((i==3)&&(rebuf[2]>16))//帧头不对
                i=0;
            else if ((i==4)&&(rebuf[3]>16))//帧头不对
                i=0;
            if(i>3)//i等于4时，已经接收到数据量字节rebuf[3]
            {
                if(i<21)
                {
                    if(i==(rebuf[3]+5))//判断是否接收一帧数据完毕
                    {
                        if(!Receive_ok)//当数据处理完成后才接收新的数据
                        {
                            Receive_ok=1;//接收完成标志
                        }

                        i=0;//缓存清0
                    }
                }
                else
                {
                    i=0;
                }
            }
        }
    }

    int z = 0;

    if(Receive_ok)//串口接收完毕
    {
        for(sum=0,z=0; z<(rebuf[3]+4); z++) //rgb_data[3]=3
        sum += rebuf[z];

        if(sum==rebuf[z])//校验和判断
        {
            float to = (rebuf[5]<<8) | rebuf[6];
            float ta = (rebuf[7]<<8) | rebuf[8];
            float bo = (rebuf[9]<<8) | rebuf[10];
            float e  = rebuf[4];

            ret.to = to/100;
            ret.ta = ta/100;
            ret.bo = bo/100;
            ret.e  = e/100;
        }

        for(int k=0; k<30; k++)
        {
          rebuf[k] = 0;
        }

        Receive_ok = false;
    }

    return ret;
}

GYSerialTempSensor::~GYSerialTempSensor()
{

}
