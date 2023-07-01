#include <reg52.h>
#include "bsp.h"

#define AD_CHANNEL 2

uint16_t rawValue = 0;      //AD����ԭʼֵ��16λ�޷�������
char recvBuffer;                //���ڽ��������ֽڻ���
uint8_t tempReg;
uint8_t toSend[6] = {'W', 'S', 'K', 1, '\0', '\0'};             //����֡
sbit SPARK = P3^5;  //������λ

enum{
    RANGE_DOWN = 12000,
    RANGE_UP = 54000,

    REG_SETTING = 16,
    BAR = 3
};

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
    uint8_t i = 0;
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

// ����һ����λ��flag��0�ϵ���Ϊ0�µ�
void SetRange(uint8_t flag)
{
    static uint8_t mask = 0x3Fu;    // ����������������֣��жϵ�4����6λ�Ƿ�ȫΪ0����1

    uint8_t regRead = TM7705_ReadReg(REG_SETTING);      // ��ȡ���üĴ�����ֵ
    if((regRead & mask == 0) && flag) return;           // GλȫΪ0���������������
    if((regRead & mask == mask) && flag == 0) return;   // GλȫΪ1�������ٽ�������

    // G0λ�Ӽ�1��PGA�������˳�2
    if(flag){
        regRead -= 0x08;
        toSend[3] /= 2;
    }
    else{
        regRead += 0x08;
        toSend[3] *= 2;
    }
    // ���޸ĺ�ļĴ�������д��ȥ
    TM7705_WriteReg(REG_SETTING, regRead);
}

void SelectRange()
{
    // ����������ʾ�������δ����������̵�����
    static uint8_t counterUp = 0, counterDown = 0;
    
    // ��ǰ���������������������ֵ���Ҳ�����ߵ�λ
    if(rawValue > RANGE_UP && toSend[3] > 1){
        counterUp++;
        if(counterUp >= BAR)    // ����BAR�δ�������������
            SetRange(1);
    }
    // ��ǰ����С�ڽ�������������ֵ���Ҳ�����͵�λ
    else if(rawValue < RANGE_DOWN && toSend[3] < 128){
        counterDown++;
        if(counterDown >= BAR)  // ����BAR�δ����򽵵�����
            SetRange(0);
    }
    else{
        // ����Ҫ�������̣�����������
        counterDown = 0;
        counterUp = 0;
    }
}

void main()
{
    SPARK = 0;      //���÷�����
    EA = 1;         //�������ж�
    ES = 1;         //�������ж�
    P0 = 0x00;      //��ʼ��ǰ��������LED
    UART_Init();    //��ʼ������       
    bsp_InitTM7705();   //��ʼ��AD7705оƬ������У׼
    TM7705_CalibSelf(AD_CHANNEL);
    P0 = 0xff;      //��ɳ�ʼ����Ϩ��LED

    while (1){            
        rawValue = TM7705_ReadAdc(AD_CHANNEL);   //��AD7705��ȡת��ֵ

        toSend[4] = rawValue >> 8;  //ԭʼֵ��8λװ������֡��5�ֽ�
        toSend[5] = rawValue;       //ԭʼֵ��8λװ������֡��6�ֽ�
        UART_SendStr(toSend, 6);    //��������֡

        if(rawValue > 52428 &&  toSend[3] == 1)    //����4V����
            P0 = 0x00;
        else
            P0 = 0xFF;

        ES = 0;
        SelectRange();              //���������Զ������㷨
        ES = 1;
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
        // �յ�ָ�ִ����λУ׼
        case '0':
            tempReg = TM7705_ReadReg(REG_SETTING);
            TM7705_WriteReg(REG_SETTING, tempReg & 0xC7u);
            bsp_DelayMS(25);
            TM7705_SytemCalibZero(AD_CHANNEL);
            bsp_DelayMS(25);
            TM7705_WriteReg(REG_SETTING, tempReg);
            break;
        // �յ�ָ�ִ����ƫУ׼
        case '1': 
            TM7705_SytemCalibFull(AD_CHANNEL);
            break;
    }
    ES = 1;
    P0 = 0xff;      //Ϩ��LED
}