#include <reg52.h>
#include "bsp.h"
//#define WSK_DBG     //调试所使用的代码与实际运行有所区别

uint16_t rawValue = 0;      //AD读数原始值，16位无符号整型
char recvBuffer;                //串口接受区单字节缓冲
uint8_t toSend[6] = {'W', 'S', 'K', '4', '\0', '\0'};             //数据帧
const uint16_t rangeDown[5] = {0, 12000, 12000, 12000, 25000};      //量程升高阈值     
const uint16_t rangeUp[5]   = {54000, 54000, 54000, 54000, 65535};  //量程降低阈值
sbit SPARK = P3^5;  //蜂鸣器位
uint8_t i;    //循环标志
uint8_t regRead;
uint8_t counterUp = 0, counterDown = 0;
const uint8_t BAR = 3;

void UART_Init() // UART串口用的T1定时器，模式是8位自动重装载，波特率9600bps@12.000MHz
{
    SCON = 0x50; // 方式1接受
    PCON = 0x00; // 使能波特率倍速位SMOD
    TMOD = 0x20; // 设定定时器1为8位自动重装方式，当溢出时将TH1存放的值自动重装
    TL1 = 0xFD; // 设定定时初值
    TH1 = 0xFD; // 设定定时器重装值
    TR1 = 1;    // 启动定时器1
}

void UART_SendStr(char* str, uint8_t length)
{
    ES = 0;
    SCON = 0x40;    // 设置位方式1发送，不接收，不允许串行中断
    for(i = 0; i != length; ++i){
        SBUF = str[i];
        while(TI == 0);     //等待发送完毕后，立即发送下一字节
        TI = 0;
    }
    SCON = 0x50;    // 设置为方式1接收
    ES = 1;
}

void SetRange(uint8_t flag)
{
    regRead = TM7705_ReadReg(16);
    if(regRead < 0x08 && flag == 1) return;
    if(regRead >= 0x38 && flag == 0) return;

    if(flag){
        regRead -= 0x08;
        if(toSend[3] < '3')
            regRead -= 0x08;
        toSend[3]++;
    }
    else{
        regRead += 0x08;
        if(toSend[3] < '4')
            regRead += 0x08;
        toSend[3]--;
    }
    TM7705_WriteReg(16, regRead);
}

void SelectRange()
{
#ifdef WSK_DBG  //模拟量程切换
    if(rawValue > rangeUp[toSend[3] - '0'] && toSend[3] != '4'){
        if(toSend[3] < '3')
            rawValue /= 2;
        ++toSend[3];
        rawValue /= 2;
    }
    else if(rawValue < rangeDown[toSend[3] - '0'] && toSend[3] != '0'){
        if(toSend[3] < '4')
            rawValue *= 2;
        --toSend[3];
        rawValue *= 2;
    }
#else           //实际量程切换
    if(rawValue > rangeUp[toSend[3] - '0'] && toSend[3] != '4'){
        counterUp++;
        if(counterUp >= BAR)
            SetRange(1);
    }
    else if(rawValue < rangeDown[toSend[3] - '0'] && toSend[3] != '0'){
        counterDown++;
        if(counterDown >= BAR)
            SetRange(0);
    }
    else{
        counterDown = 0;
        counterUp = 0;
    }
#endif
}

void main()
{
    SPARK = 0;      //禁用蜂鸣器
    EA = 1;         //允许总中断
    ES = 1;         //允许串行中断
    P0 = 0x00;      //初始化前点亮所有LED
    UART_Init();    //初始化串口
#ifndef WSK_DBG     //非调试状态，初始化AD7705芯片，并自校准
    bsp_InitTM7705();
    TM7705_CalibSelf(2);
#endif
    P0 = 0xff;      //完成初始化后熄灭LED
    while (1){
#ifdef WSK_DBG      //模拟采样值，从最低量程开始递增
        rawValue += 250;
        if(toSend[3] == '4' && rawValue > 65000){
            toSend[3] = '0';
            rawValue = 0;
        }
        if(rawValue > 52428 && toSend[3] == '4')
            P0 = 0x00;
        else
            P0 = 0xFF;
#else              
        rawValue = TM7705_ReadAdc(2);   //从AD7705读取转换值
#endif
        toSend[4] = rawValue >> 8;  //原始值高8位装入数据帧第5字节
        toSend[5] = rawValue;       //原始值低8位装入数据帧第6字节
        UART_SendStr(toSend, 6);    //发送数据帧
        SelectRange();              //调用量程自动调整算法

        bsp_DelayMS(25);
    }
}

void serial() interrupt 4
{
    P0 = 0x00;      //点亮LED，提醒接收到指令
    ES = 0;
    RI = 0;
    recvBuffer = SBUF;
    switch(recvBuffer){
        case '0':
            TM7705_SytemCalibZero(2);
        case '1': 
            TM7705_SytemCalibFull(2);
    }
    ES = 1;
    P0 = 0xff;      //熄灭LED
}