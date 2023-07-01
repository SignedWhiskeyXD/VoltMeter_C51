#include <reg52.h>
#include "bsp.h"

#define AD_CHANNEL 2

uint16_t rawValue = 0;      //AD读数原始值，16位无符号整型
char recvBuffer;                //串口接受区单字节缓冲
uint8_t tempReg;
uint8_t toSend[6] = {'W', 'S', 'K', 1, '\0', '\0'};             //数据帧
sbit SPARK = P3^5;  //蜂鸣器位

enum{
    RANGE_DOWN = 12000,
    RANGE_UP = 54000,

    REG_SETTING = 16,
    BAR = 3
};

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
    uint8_t i = 0;
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

// 调整一级挡位，flag非0上档，为0下档
void SetRange(uint8_t flag)
{
    static uint8_t mask = 0x3Fu;    // 用于掩码运算的遮罩，判断第4到第6位是否全为0或者1

    uint8_t regRead = TM7705_ReadReg(REG_SETTING);      // 读取设置寄存器的值
    if((regRead & mask == 0) && flag) return;           // G位全为0，不得再提高量程
    if((regRead & mask == mask) && flag == 0) return;   // G位全为1，不得再降低量程

    // G0位加减1，PGA倍数即乘除2
    if(flag){
        regRead -= 0x08;
        toSend[3] /= 2;
    }
    else{
        regRead += 0x08;
        toSend[3] *= 2;
    }
    // 将修改后的寄存器内容写回去
    TM7705_WriteReg(REG_SETTING, regRead);
}

void SelectRange()
{
    // 计数器，表示连续几次触发调整量程的条件
    static uint8_t counterUp = 0, counterDown = 0;
    
    // 当前读数大于提高量程所需阈值，且不在最高档位
    if(rawValue > RANGE_UP && toSend[3] > 1){
        counterUp++;
        if(counterUp >= BAR)    // 连续BAR次触发则升高量程
            SetRange(1);
    }
    // 当前读数小于降低量程所需阈值，且不在最低档位
    else if(rawValue < RANGE_DOWN && toSend[3] < 128){
        counterDown++;
        if(counterDown >= BAR)  // 连续BAR次触发则降低量程
            SetRange(0);
    }
    else{
        // 不需要调整量程，计数器归零
        counterDown = 0;
        counterUp = 0;
    }
}

void main()
{
    SPARK = 0;      //禁用蜂鸣器
    EA = 1;         //允许总中断
    ES = 1;         //允许串行中断
    P0 = 0x00;      //初始化前点亮所有LED
    UART_Init();    //初始化串口       
    bsp_InitTM7705();   //初始化AD7705芯片，并自校准
    TM7705_CalibSelf(AD_CHANNEL);
    P0 = 0xff;      //完成初始化后熄灭LED

    while (1){            
        rawValue = TM7705_ReadAdc(AD_CHANNEL);   //从AD7705读取转换值

        toSend[4] = rawValue >> 8;  //原始值高8位装入数据帧第5字节
        toSend[5] = rawValue;       //原始值低8位装入数据帧第6字节
        UART_SendStr(toSend, 6);    //发送数据帧

        if(rawValue > 52428 &&  toSend[3] == 1)    //大于4V报警
            P0 = 0x00;
        else
            P0 = 0xFF;

        ES = 0;
        SelectRange();              //调用量程自动调整算法
        ES = 1;
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
        // 收到指令，执行零位校准
        case '0':
            tempReg = TM7705_ReadReg(REG_SETTING);
            TM7705_WriteReg(REG_SETTING, tempReg & 0xC7u);
            bsp_DelayMS(25);
            TM7705_SytemCalibZero(AD_CHANNEL);
            bsp_DelayMS(25);
            TM7705_WriteReg(REG_SETTING, tempReg);
            break;
        // 收到指令，执行满偏校准
        case '1': 
            TM7705_SytemCalibFull(AD_CHANNEL);
            break;
    }
    ES = 1;
    P0 = 0xff;      //熄灭LED
}