#include <reg52.h>
#include "bsp.h"
//#define WSK_DBG     //������ʹ�õĴ�����ʵ��������������

uint16_t rawValue = 0;      //AD����ԭʼֵ��16λ�޷�������
char recvBuffer;                //���ڽ��������ֽڻ���
uint8_t toSend[6] = {'W', 'S', 'K', '4', '\0', '\0'};             //����֡
const uint16_t rangeDown[5] = {0, 12000, 12000, 12000, 25000};      //����������ֵ     
const uint16_t rangeUp[5]   = {54000, 54000, 54000, 54000, 65535};  //���̽�����ֵ
sbit SPARK = P3^5;  //������λ
uint8_t i;    //ѭ����־
uint8_t regRead;
uint8_t counterUp = 0, counterDown = 0;
const uint8_t BAR = 3;

void UART_Init() // UART�����õ�T1��ʱ����ģʽ��8λ�Զ���װ�أ�������9600bps@12.000MHz
{
    SCON = 0x50; // ��ʽ1����
    PCON = 0x00; // ʹ�ܲ����ʱ���λSMOD
    TMOD = 0x20; // �趨��ʱ��1Ϊ8λ�Զ���װ��ʽ�������ʱ��TH1��ŵ�ֵ�Զ���װ
    TL1 = 0xFD; // �趨��ʱ��ֵ
    TH1 = 0xFD; // �趨��ʱ����װֵ
    TR1 = 1;    // ������ʱ��1
}

void UART_SendStr(char* str, uint8_t length)
{
    ES = 0;
    SCON = 0x40;    // ����λ��ʽ1���ͣ������գ����������ж�
    for(i = 0; i != length; ++i){
        SBUF = str[i];
        while(TI == 0);     //�ȴ�������Ϻ�����������һ�ֽ�
        TI = 0;
    }
    SCON = 0x50;    // ����Ϊ��ʽ1����
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
#ifdef WSK_DBG  //ģ�������л�
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
#else           //ʵ�������л�
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
    SPARK = 0;      //���÷�����
    EA = 1;         //�������ж�
    ES = 1;         //�������ж�
    P0 = 0x00;      //��ʼ��ǰ��������LED
    UART_Init();    //��ʼ������
#ifndef WSK_DBG     //�ǵ���״̬����ʼ��AD7705оƬ������У׼
    bsp_InitTM7705();
    TM7705_CalibSelf(2);
#endif
    P0 = 0xff;      //��ɳ�ʼ����Ϩ��LED
    while (1){
#ifdef WSK_DBG      //ģ�����ֵ����������̿�ʼ����
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
        rawValue = TM7705_ReadAdc(2);   //��AD7705��ȡת��ֵ
#endif
        toSend[4] = rawValue >> 8;  //ԭʼֵ��8λװ������֡��5�ֽ�
        toSend[5] = rawValue;       //ԭʼֵ��8λװ������֡��6�ֽ�
        UART_SendStr(toSend, 6);    //��������֡
        SelectRange();              //���������Զ������㷨

        bsp_DelayMS(25);
    }
}

void serial() interrupt 4
{
    P0 = 0x00;      //����LED�����ѽ��յ�ָ��
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
    P0 = 0xff;      //Ϩ��LED
}