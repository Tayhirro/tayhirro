/*
 * xiao_vofa.c
 *
 *  Created on: 2023��4��29��
 *      Author: Jayden_NaN
 */

#include "xiao_vofa.h"
//------------------------------------------------------------
//״̬��
uint8 Vofa_readStatusWired = 0;         //���ߴ����ȡ���ݵ�״̬��
uint8 Vofa_readStatusWireLess = 0;      //���ߴ����ȡ���ݵ�״̬��
uint8 Vofa_getDataStuts = 0;            //Vofa�õ�����״̬�� -> ��������,���߾�����
uint8 Vofa_processDataStatus = 0;       //��������״̬��
uint8 Vofa_setDataStatus = 0;           //��������״̬��

//------------------------------------------------------------
//����
uint8 Vofa_dataBuff[32];                //ָ����ջ�����
uint8 Vofa_dataLen = 0;                 //ָ���
float Vofa_changeData = 0.0;            //���ڸı������

/**
 * @brief                   ���ַ�����ת��Ϊ��������
 * @paremeter   len         ���ݵĳ��� - ����:123.4567�����ݳ���Ϊ8
 * @parameter   digitNum    ת������   - ����:1.0��ʾ�����������x.xxxx����;10.0��ʾ�����������xx.xxxx����
 * @parameter   dataPtr     ����ָ��   - ��Ҫ�����������������Ǹ���
 * @return                  ת����õ��ĸ�������
 */
static float Vofa_Char2Float(uint8 len, float digitNum, uint32 dataPtr) {
    float retData = 0.0;
    for (int i = 0; i < len; ++i) {
        if (Vofa_dataBuff[dataPtr + i] == '.')
            continue;
        retData += digitNum * (Vofa_dataBuff[dataPtr + i] - '0');
        digitNum /= 10;
    }
    return retData;
}

static void Vofa_Int2Str (char *str, int32 number)
{
    zf_assert(str != NULL);
    uint8 data_temp[16];                                                        // ������
    uint8 bit = 0;                                                              // ����λ��
    int32 number_temp = 0;

    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 > number)                                                          // ����
        {
            *str ++ = '-';
            number = -number;
        }
        else if(0 == number)                                                    // �������Ǹ� 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // ѭ��ֱ����ֵ����
        {
            number_temp = number % 10;
            data_temp[bit ++] = func_abs(number_temp);                          // ������ֵ��ȡ����
            number /= 10;                                                       // ��������ȡ�ĸ�λ��
        }
        while(0 != bit)                                                         // ��ȡ�����ָ����ݼ�����
        {
            *str ++ = (data_temp[bit - 1] + 0x30);                              // �����ִӵ��������е���ȡ�� �����������ַ���
            bit --;
        }
        *str = '\0';
    }while(0);
}

static void Vofa_Float2Str (char *str, float number, uint8 point_bit)
{
    zf_assert(str != NULL);
    int data_int = 0;                                                           // ��������
    int data_float = 0.0;                                                       // С������
    int data_temp[8];                                                           // �����ַ�����
    int data_temp_point[6];                                                     // С���ַ�����
    uint8 bit = point_bit;                                                      // ת������λ��

    do
    {
        if(NULL == str)
        {
            break;
        }

        // ��ȡ��������
        data_int = (int)number;                                                 // ֱ��ǿ��ת��Ϊ int
        if(0 > number)                                                          // �ж�Դ�������������Ǹ���
        {
            *str ++ = '-';
        }
        else if(0.0 == number)                                                  // ����Ǹ� 0
        {
            *str ++ = '0';
            *str ++ = '.';
            *str = '0';
            break;
        }

        // ��ȡС������
        number = number - data_int;                                             // ��ȥ�������ּ���
        while(bit --)
        {
            number = number * 10;                                               // ����Ҫ��С��λ����ȡ����������
        }
        data_float = (int)number;                                               // ��ȡ�ⲿ����ֵ

        // ��������תΪ�ַ���
        bit = 0;
        do
        {
            data_temp[bit ++] = data_int % 10;                                  // ���������ֵ���д���ַ�������
            data_int /= 10;
        }while(0 != data_int);
        *(str + sizeof(uint8) * bit) = '\0';
        while(0 != bit)
        {
            *str ++ = (func_abs(data_temp[bit - 1]) + 0x30);                    // �ٵ��򽫵������ֵд���ַ��� �õ�������ֵ
            bit --;
        }

        // С������תΪ�ַ���
        if(point_bit != 0)
        {
            bit = 0;
            *str ++ = '.';
            if(0 == data_float)
            {
                *str = '0';
            }
            else
            {
                while(0 != point_bit)                                           // �ж���Чλ��
                {
                    data_temp_point[bit ++] = data_float % 10;                  // ����д���ַ�������
                    data_float /= 10;
                    point_bit --;
                }
                while(0 != bit)
                {
                    *str ++ = (func_abs(data_temp_point[bit - 1]) + 0x30);      // �ٵ��򽫵������ֵд���ַ��� �õ�������ֵ
                    bit --;
                }
            }
        }
        *str = '\0';
    }while(0);
}

/**
 * @brief                   �����õ�������
 * @parameter   dataLen     �������������ݳ���
 * @return                  ���õ������ݽ���Ϊ����������
 * @example                 Vofa_GetData(Vofa_dataLen);
 */
static float Vofa_GetData(uint8 dataLen) {
    Vofa_processDataStatus = 0;
    float retData = 0.0;
    uint8 minusStatus = 0;
    uint8 dataPtr = 4;
    float Vofa_digitNum = 100.0;
    //����Ǹ����Ļ�,dataPtr�����һλ
    if (Vofa_dataBuff[dataPtr] == '-') {
        minusStatus = 1;
        ++dataPtr;
    }
    //����PID������
    //���������Ϊ : "MPL:��λ����,��λ����!"
    //��һ�����ݵ��ж�
    if (dataLen == 11) {
        //ֻ�����ڸ�λ���� + 4λС����ʱ�򳤶Ȳ�����11
        Vofa_digitNum = 1.0;
        retData = Vofa_Char2Float(7, Vofa_digitNum, dataPtr);
    }
    else if (dataLen == 12) {
        //�ж��Ǹ�����12λ����������ʮ��λ
        if (minusStatus == 1) {
            Vofa_digitNum = 1.0;
            retData = Vofa_Char2Float(7, Vofa_digitNum, dataPtr);
        }
        else {
            Vofa_digitNum = 10.0;
            retData = Vofa_Char2Float(7, Vofa_digitNum, dataPtr);
        }
    }
    else if (dataLen == 13) {
        //�ж��Ǹ�����13λ����������13λ
        if (minusStatus == 1) {
            Vofa_digitNum = 10.0;
            retData = Vofa_Char2Float(7, Vofa_digitNum, dataPtr);
        }
        else {
            Vofa_digitNum = 100.0;
            retData = Vofa_Char2Float(8, Vofa_digitNum, dataPtr);
        }
    }
    else if (dataLen == 14) {
        //�����ֻ���Ǹ�����
        Vofa_digitNum = 100.0;
        retData = Vofa_Char2Float(8, Vofa_digitNum, dataPtr);
    }

    if (minusStatus == 1)
        retData = -retData;
    Vofa_processDataStatus = 1;
    return retData;
}


/*************************************************************
 * ����ģʽ˵��
 * 1. ��ʽ:
 * ���� + ���� + (��������) + : + ���� + ��ֹ��(!)
 * 2. ����:
 *      MP1:-29.2451!
 * 3. ����:
 *      ���1��P��������Ϊ-29.245
 * 4. ָ���б�
 *     _1.--------------------0������λ--------------------
 *      1.M - ���;     2.E - ���
 *     _2.--------------------1������λ--------------------
 *      1.P - P����     2.I - I����     3. D - D����      4. S - �����޷�     5. C - �����޷�
 *      6.T - Ŀ��ֵ
 *      7.A - ��ȺͲ�A����   8.B - ��ȺͲ�B����   9.C - ��ȺͲ�C����
 *     _3.--------------------2������λ--------------------
 *      1.1 - ���1     2.2 - ���2
 *      3.1 - Elec��PID 4.2 - Elec����ȺͲ����
 *************************************************************/
static void Vofa_SetData(void) {
    Vofa_setDataStatus = 0;
    //=========================================================

    //���õ���Ĳ���
    if (Vofa_dataBuff[0] == 'M') {
        //------------------------------------------------------------
        //���1
        if (Vofa_dataBuff[2] == '1') {
            if (Vofa_dataBuff[1] == 'P') {              //P����
                Motor_1P = Vofa_changeData;
                Motor_1SetPIDP(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'I') {         //I����
                Motor_1I = Vofa_changeData;
                Motor_1SetPIDI(Vofa_changeData);
            }
            else if(Vofa_dataBuff[1] == 'D') {          //D����
                Motor_1D = Vofa_changeData;
                Motor_1SetPIDD(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'S') {         //�����޷� - �����޷�����Ϊ��ͬ��ֵ(�ݶ�)
                Motor_pLimit = Vofa_changeData;
                Motor_1SetPIDLimit(Vofa_changeData);
                Motor_2SetPIDLimit(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'C') {         //�����޷�
                Motor_coLimit = Vofa_changeData;
                Motor_1SetPIDCoLimit(Vofa_changeData);
                Motor_2SetPIDCoLimit(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'T') {
                Motor_1Target = Vofa_changeData;
            }
        }
        //------------------------------------------------------------
        //���2
        else if (Vofa_dataBuff[2] == '2') {
            if (Vofa_dataBuff[1] == 'P') {              //P����
                Motor_2P = Vofa_changeData;
                Motor_2SetPIDP(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'I') {         //I����
                Motor_2I = Vofa_changeData;
                Motor_2SetPIDI(Vofa_changeData);
            }
            else if(Vofa_dataBuff[1] == 'D') {          //D����
                Motor_2D = Vofa_changeData;
                Motor_2SetPIDD(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'S') {         //�����޷�
                Motor_pLimit = Vofa_changeData;
                Motor_1SetPIDLimit(Vofa_changeData);
                Motor_2SetPIDLimit(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'C') {         //�����޷�
                Motor_coLimit = Vofa_changeData;
                Motor_1SetPIDCoLimit(Vofa_changeData);
                Motor_2SetPIDCoLimit(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'T') {         //Ŀ��ֵ
                Motor_2Target = Vofa_changeData;
            }
        }
        //------------------------------------------------------------
        //�ٶ�
        else if (Vofa_dataBuff[2] == 'S'){
            if (Vofa_dataBuff[1] == 'S'){
                Speed_set = Vofa_changeData;
            }
        }
    }
    //============================================================
    //���õ�ŵ�����
    else if (Vofa_dataBuff[0] == 'E') {
        //ips200_show_string(0, 2, "1");
        if (Vofa_dataBuff[2] == '1') {
            if (Vofa_dataBuff[1] == 'P') {
                Elec_P = Vofa_changeData;
             //   Elec_SetPIDP(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'I') {
                Elec_I = Vofa_changeData;
              //  Elec_SetPIDI(Vofa_changeData);
            }
            else if(Vofa_dataBuff[1] == 'D') {
                Elec_D = Vofa_changeData;
             //   Elec_SetPIDD(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'S') {
                Elec_pLimit = Vofa_changeData;
            //    Elec_SetPIDSumLimit(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'C') {
                Elec_coLimit = Vofa_changeData;
             //   Elec_SetPIDCoLimit(Vofa_changeData);
            }
        }
    }
    Vofa_setDataStatus = 1;
}

/*
 * @brief               ���������(�ڲ�����)
 *                      ���������,�Ͱѵõ�����״̬��λ1
 * @parameter device    ѡ��ʹ�õĴ����豸
 *                      VOFA_WIRELESS -     ���ߴ���(����)
 *                      VOFA_WIRED    -     ���ߴ���(����)
 * @return              ��
 * @example             Vofa_Mointor(VOFA_WIRELESS);
 */
static void Vofa_Monitor(VOFA_TRAN_DEVICE device) {
    //----------------------------------------
    //����ģ���ж�
    if (device == VOFA_WIRELESS) {
        if (Vofa_readStatusWireLess == 1) {
         //   Vofa_dataLen = (uint8)wireless_uart_read_buff(Vofa_dataBuff, 32);
            Vofa_dataBuff[Vofa_dataLen] = '\0';
            Vofa_readStatusWireLess = 0;
            Vofa_getDataStuts = 1;
        }
    }
    //----------------------------------------
    //����ģ���ж�
    else if (device == VOFA_WIRED) {
        if (Vofa_readStatusWired == 1) {
            //���ߵĻ�ֱ�ӵ�����debug��������
            //��һ��ǧ���ܸ�,���˾Ͳ�һ��������!!!!!!!!!!
            //�������Ҫ >= 14,���������Ǹ�����
            //���˻��޸�,����С�Ļ������޸�,�ͻᵼ�����ݶ��ٵ�����
            //����Ĳ�������Ϊ32,������Ч��һЩ�����ӵ�
            Vofa_dataLen = (uint8)debug_read_ring_buffer(Vofa_dataBuff, 32);
            Vofa_dataBuff[Vofa_dataLen] = '\0';
            Vofa_readStatusWired = 0;
            Vofa_getDataStuts = 1;
        }
    }
}

/*
 * @brief               ��Ϣ����(�ڲ�����)
 * @parameter device    �����豸
 *                      1. VOFA_WIRELESS -     ���ߴ���(����)
 *                      2. VOFA_WIRED    -     ���ߴ���(����)
 * @example             Vofa_CallBack(VOFA_WIRELESS);
 */
static void Vofa_CallBack(VOFA_TRAN_DEVICE device) {
    //�ش����յ�������
    if (device == VOFA_WIRELESS) {
        //�ش����յ�����
      //  //wireless_uart_send_buff(Vofa_dataBuff, strlen((const char *)Vofa_dataBuff));    // ��ʾ�յ������ݸ���
      //  wireless_uart_send_string("\r\n");
    }
    else if (device == VOFA_WIRED) {
        //�ش����յ�������
        printf("%s\r\n", Vofa_dataBuff);
    }
    memset(Vofa_dataBuff, 0, 32);

}

/*
 * @brief               Vofa��������
 * @parameter device    ѡ�����ģʽ
 *                      1 - VOFA_WIRELESS       ���ߴ���
 *                      2 - VOFA_WIRED          ���ߴ���
 * @example     Vofa_DataAnalyze(VOFA_WIRELESS);
 */
void Vofa_DataAnalyze(VOFA_TRAN_DEVICE device) {
    //------------------------------
    //�����Ƿ�������
    Vofa_Monitor(device);
    //------------------------------
    //��������
    if (Vofa_getDataStuts == 1) {
        Vofa_getDataStuts = 0;
        Vofa_changeData = Vofa_GetData(Vofa_dataLen);
    }
    //------------------------------
    //���ò���
    if (Vofa_processDataStatus == 1) {
        Vofa_processDataStatus = 0;
        Vofa_SetData();
    }
    //------------------------------
    //������Ϣ
    if (Vofa_setDataStatus == 1) {
        Vofa_setDataStatus = 0;
        Vofa_CallBack(device);
    }
}

/*
 * @brief                   ������ӡ
 * @paremeter dataType      ��������
 * @parameter num           ���ݸ���
 * @parameter ...           �����б�
 * @example                 Vofa_WireLessPrintf(VOFA_INT, 3, 1, 2, 3);
 */
void Vofa_WireLessPrintf(VOFA_DATA_TYPE dataType, int num, ...) {
    float decimalPart = 0.0;
    int16 data_int;
    float data_float;
    int16 integerPart = 0;
    uint8 dataBuff[64];
    va_list valist;
    va_start(valist, num);
    for (int i = 0; i < num; ++i) {
        //------------------------------
        //�������ݴ���
        if (dataType == VOFA_INT) {
            //���������ݽ��д���
            data_int = va_arg(valist, int);
            if (data_int == 0) {
                wireless_uart_send_string("0");
            }
            else {
                Vofa_Int2Str(dataBuff, data_int);
              //  //wireless_uart_send_buff(dataBuff, strlen(dataBuff));
            }
        }
        //------------------------------
        //���������ݴ���
        else if (dataType == VOFA_FLOAT) {
            data_float = va_arg(valist, float);
            //�������ݴ���
            if (data_float == 0.0) {
                wireless_uart_send_string("0.0000");
            }
            else {
                integerPart = (int16)data_float;
                decimalPart = data_float - integerPart;
                if (decimalPart == 0.0) {
                    Vofa_Int2Str(dataBuff, integerPart);
                 //   //wireless_uart_send_buff(dataBuff, strlen(dataBuff));
                    wireless_uart_send_string(".0000");
                    printf("%.4f\n", data_float);
                }
                else {
                    Vofa_Float2Str(dataBuff, data_float, 4);
                //    //wireless_uart_send_buff(dataBuff, strlen(dataBuff));
                }
            }
        }

        //------------------------------
        //��Vofa�������ݴ�����
        if (i + 1 >= num) {
            wireless_uart_send_string("\n");
        }
        else {
            wireless_uart_send_string(",");
        }
    }
    va_end(valist);
}
