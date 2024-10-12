# 履带无人车实物实验支持包

## 功能
* 控制板指令发送与状态接收
* 手柄遥控与键盘遥控
* 动捕位姿反馈接收

## 控制板指令发送与状态接收
### 底板发送:
1. 底盘状态反馈:
   
    在usb端点0x82

    * drift8 voltage;   //V
    * drift32 imuQuat[3];   //r,p,y
    * drift16 wheelSpeeds[2];   //rad/s
    * uint16_t armAngles[2];    //49~2000
    
    字节对齐，长度21字节

2. 调试信息:

    在usb端点0x84,按字符流发送。

### 底板接收:
1. 速度角速度指令

    在usb端点0x02

    * drift16 linearVel;    // m/s
    * drift16 angularVel;   // rad/s

    字节对齐，长度4字节
2. 机械臂角度指令

    在usb端点0x04

    * uint8_t flag == 0;
    * uint16_t armAngles[2];    //49~2000

    字节对齐,长度5字节

3. 电磁铁开关指令

    在usb端点0x04
    * uint8_t flag == 1;
    * uint8_t emag; //1:open;0:close

    字节对齐，长度2字节

4. 舵机角度指令

    在usb端点0x04
    * uint8_t flag == 2;
    * uint16_t tripodAngle; //deg

    字节对齐，长度3字节

## 手柄遥控与键盘遥控
## 动捕位姿反馈接收