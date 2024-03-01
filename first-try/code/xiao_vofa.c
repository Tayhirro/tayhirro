/*
 * xiao_vofa.c
 *
 *  Created on: 2023年4月29日
 *      Author: Jayden_NaN
 */

#include "xiao_vofa.h"
//------------------------------------------------------------
//状态机
uint8 Vofa_readStatusWired = 0;         //有线传输读取数据的状态机
uint8 Vofa_readStatusWireLess = 0;      //无线传输读取数据的状态机
uint8 Vofa_getDataStuts = 0;            //Vofa得到数据状态机 -> 对于有线,无线均适用
uint8 Vofa_processDataStatus = 0;       //处理数据状态机
uint8 Vofa_setDataStatus = 0;           //设置数据状态机

//------------------------------------------------------------
//数据
uint8 Vofa_dataBuff[32];                //指令接收缓冲区
uint8 Vofa_dataLen = 0;                 //指令长度
float Vofa_changeData = 0.0;            //用于改变的数据

/**
 * @brief                   将字符数据转换为浮点数据
 * @paremeter   len         数据的长度 - 例如:123.4567的数据长度为8
 * @parameter   digitNum    转换精度   - 例如:1.0表示处理的数据是x.xxxx类型;10.0表示处理的数据是xx.xxxx类型
 * @parameter   dataPtr     数据指针   - 主要用来解析是整数还是负数
 * @return                  转换后得到的浮点数据
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
    uint8 data_temp[16];                                                        // 缓冲区
    uint8 bit = 0;                                                              // 数字位数
    int32 number_temp = 0;

    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 > number)                                                          // 负数
        {
            *str ++ = '-';
            number = -number;
        }
        else if(0 == number)                                                    // 或者这是个 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // 循环直到数值归零
        {
            number_temp = number % 10;
            data_temp[bit ++] = func_abs(number_temp);                          // 倒序将数值提取出来
            number /= 10;                                                       // 削减被提取的个位数
        }
        while(0 != bit)                                                         // 提取的数字个数递减处理
        {
            *str ++ = (data_temp[bit - 1] + 0x30);                              // 将数字从倒序数组中倒序取出 变成正序放入字符串
            bit --;
        }
        *str = '\0';
    }while(0);
}

static void Vofa_Float2Str (char *str, float number, uint8 point_bit)
{
    zf_assert(str != NULL);
    int data_int = 0;                                                           // 整数部分
    int data_float = 0.0;                                                       // 小数部分
    int data_temp[8];                                                           // 整数字符缓冲
    int data_temp_point[6];                                                     // 小数字符缓冲
    uint8 bit = point_bit;                                                      // 转换精度位数

    do
    {
        if(NULL == str)
        {
            break;
        }

        // 提取整数部分
        data_int = (int)number;                                                 // 直接强制转换为 int
        if(0 > number)                                                          // 判断源数据是正数还是负数
        {
            *str ++ = '-';
        }
        else if(0.0 == number)                                                  // 如果是个 0
        {
            *str ++ = '0';
            *str ++ = '.';
            *str = '0';
            break;
        }

        // 提取小数部分
        number = number - data_int;                                             // 减去整数部分即可
        while(bit --)
        {
            number = number * 10;                                               // 将需要的小数位数提取到整数部分
        }
        data_float = (int)number;                                               // 获取这部分数值

        // 整数部分转为字符串
        bit = 0;
        do
        {
            data_temp[bit ++] = data_int % 10;                                  // 将整数部分倒序写入字符缓冲区
            data_int /= 10;
        }while(0 != data_int);
        *(str + sizeof(uint8) * bit) = '\0';
        while(0 != bit)
        {
            *str ++ = (func_abs(data_temp[bit - 1]) + 0x30);                    // 再倒序将倒序的数值写入字符串 得到正序数值
            bit --;
        }

        // 小数部分转为字符串
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
                while(0 != point_bit)                                           // 判断有效位数
                {
                    data_temp_point[bit ++] = data_float % 10;                  // 倒序写入字符缓冲区
                    data_float /= 10;
                    point_bit --;
                }
                while(0 != bit)
                {
                    *str ++ = (func_abs(data_temp_point[bit - 1]) + 0x30);      // 再倒序将倒序的数值写入字符串 得到正序数值
                    bit --;
                }
            }
        }
        *str = '\0';
    }while(0);
}

/**
 * @brief                   解析得到的数据
 * @parameter   dataLen     用来解析的数据长度
 * @return                  将得到的数据解析为浮点型数据
 * @example                 Vofa_GetData(Vofa_dataLen);
 */
static float Vofa_GetData(uint8 dataLen) {
    Vofa_processDataStatus = 0;
    float retData = 0.0;
    uint8 minusStatus = 0;
    uint8 dataPtr = 4;
    float Vofa_digitNum = 100.0;
    //如果是负数的话,dataPtr向后移一位
    if (Vofa_dataBuff[dataPtr] == '-') {
        minusStatus = 1;
        ++dataPtr;
    }
    //处理PID的数据
    //命令的类型为 : "MPL:三位数据,四位数据!"
    //做一下数据的判断
    if (dataLen == 11) {
        //只有是在个位数据 + 4位小数的时候长度才能是11
        Vofa_digitNum = 1.0;
        retData = Vofa_Char2Float(7, Vofa_digitNum, dataPtr);
    }
    else if (dataLen == 12) {
        //判断是负数的12位还是正数的十二位
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
        //判断是负数的13位还是整数的13位
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
        //这个就只能是负数了
        Vofa_digitNum = 100.0;
        retData = Vofa_Char2Float(8, Vofa_digitNum, dataPtr);
    }

    if (minusStatus == 1)
        retData = -retData;
    Vofa_processDataStatus = 1;
    return retData;
}


/*************************************************************
 * 数据模式说明
 * 1. 格式:
 * 器件 + 参数 + (额外数据) + : + 数据 + 终止符(!)
 * 2. 例子:
 *      MP1:-29.2451!
 * 3. 解释:
 *      电机1的P参数设置为-29.245
 * 4. 指代列表
 *     _1.--------------------0号数据位--------------------
 *      1.M - 电机;     2.E - 电磁
 *     _2.--------------------1号数据位--------------------
 *      1.P - P参数     2.I - I参数     3. D - D参数      4. S - 积分限幅     5. C - 修正限幅
 *      6.T - 目标值
 *      7.A - 差比和差A参数   8.B - 差比和差B参数   9.C - 差比和差C参数
 *     _3.--------------------2号数据位--------------------
 *      1.1 - 电机1     2.2 - 电机2
 *      3.1 - Elec调PID 4.2 - Elec调差比和差参数
 *************************************************************/
static void Vofa_SetData(void) {
    Vofa_setDataStatus = 0;
    //=========================================================

    //设置电机的参数
    if (Vofa_dataBuff[0] == 'M') {
        //------------------------------------------------------------
        //电机1
        if (Vofa_dataBuff[2] == '1') {
            if (Vofa_dataBuff[1] == 'P') {              //P参数
                Motor_1P = Vofa_changeData;
                Motor_1SetPIDP(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'I') {         //I参数
                Motor_1I = Vofa_changeData;
                Motor_1SetPIDI(Vofa_changeData);
            }
            else if(Vofa_dataBuff[1] == 'D') {          //D参数
                Motor_1D = Vofa_changeData;
                Motor_1SetPIDD(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'S') {         //积分限幅 - 两个限幅设置为相同的值(暂定)
                Motor_pLimit = Vofa_changeData;
                Motor_1SetPIDLimit(Vofa_changeData);
                Motor_2SetPIDLimit(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'C') {         //修正限幅
                Motor_coLimit = Vofa_changeData;
                Motor_1SetPIDCoLimit(Vofa_changeData);
                Motor_2SetPIDCoLimit(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'T') {
                Motor_1Target = Vofa_changeData;
            }
        }
        //------------------------------------------------------------
        //电机2
        else if (Vofa_dataBuff[2] == '2') {
            if (Vofa_dataBuff[1] == 'P') {              //P参数
                Motor_2P = Vofa_changeData;
                Motor_2SetPIDP(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'I') {         //I参数
                Motor_2I = Vofa_changeData;
                Motor_2SetPIDI(Vofa_changeData);
            }
            else if(Vofa_dataBuff[1] == 'D') {          //D参数
                Motor_2D = Vofa_changeData;
                Motor_2SetPIDD(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'S') {         //积分限幅
                Motor_pLimit = Vofa_changeData;
                Motor_1SetPIDLimit(Vofa_changeData);
                Motor_2SetPIDLimit(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'C') {         //修正限幅
                Motor_coLimit = Vofa_changeData;
                Motor_1SetPIDCoLimit(Vofa_changeData);
                Motor_2SetPIDCoLimit(Vofa_changeData);
            }
            else if (Vofa_dataBuff[1] == 'T') {         //目标值
                Motor_2Target = Vofa_changeData;
            }
        }
        //------------------------------------------------------------
        //速度
        else if (Vofa_dataBuff[2] == 'S'){
            if (Vofa_dataBuff[1] == 'S'){
                Speed_set = Vofa_changeData;
            }
        }
    }
    //============================================================
    //设置电磁的数据
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
 * @brief               监控数据流(内部函数)
 *                      如果有数据,就把得到数据状态置位1
 * @parameter device    选择使用的传输设备
 *                      VOFA_WIRELESS -     无线传输(蓝牙)
 *                      VOFA_WIRED    -     有线传输(串口)
 * @return              无
 * @example             Vofa_Mointor(VOFA_WIRELESS);
 */
static void Vofa_Monitor(VOFA_TRAN_DEVICE device) {
    //----------------------------------------
    //无线模块判断
    if (device == VOFA_WIRELESS) {
        if (Vofa_readStatusWireLess == 1) {
         //   Vofa_dataLen = (uint8)wireless_uart_read_buff(Vofa_dataBuff, 32);
            Vofa_dataBuff[Vofa_dataLen] = '\0';
            Vofa_readStatusWireLess = 0;
            Vofa_getDataStuts = 1;
        }
    }
    //----------------------------------------
    //有线模块判断
    else if (device == VOFA_WIRED) {
        if (Vofa_readStatusWired == 1) {
            //有线的话直接调用了debug传输内容
            //这一句千万不能改,改了就不一定能用了!!!!!!!!!!
            //这个数据要 >= 14,就是最后的那个长度
            //长了会修改,但是小的话不会修改,就会导致数据读少的问题
            //这里的参数设置为32,可以有效把一些垃圾扔掉
            Vofa_dataLen = (uint8)debug_read_ring_buffer(Vofa_dataBuff, 32);
            Vofa_dataBuff[Vofa_dataLen] = '\0';
            Vofa_readStatusWired = 0;
            Vofa_getDataStuts = 1;
        }
    }
}

/*
 * @brief               信息回馈(内部函数)
 * @parameter device    传输设备
 *                      1. VOFA_WIRELESS -     无线传输(蓝牙)
 *                      2. VOFA_WIRED    -     有线传输(串口)
 * @example             Vofa_CallBack(VOFA_WIRELESS);
 */
static void Vofa_CallBack(VOFA_TRAN_DEVICE device) {
    //回传就收到的数据
    if (device == VOFA_WIRELESS) {
        //回传接收的数据
      //  //wireless_uart_send_buff(Vofa_dataBuff, strlen((const char *)Vofa_dataBuff));    // 显示收到的数据个数
      //  wireless_uart_send_string("\r\n");
    }
    else if (device == VOFA_WIRED) {
        //回传接收到的数据
        printf("%s\r\n", Vofa_dataBuff);
    }
    memset(Vofa_dataBuff, 0, 32);

}

/*
 * @brief               Vofa分析数据
 * @parameter device    选择传输的模式
 *                      1 - VOFA_WIRELESS       无线传输
 *                      2 - VOFA_WIRED          有线传输
 * @example     Vofa_DataAnalyze(VOFA_WIRELESS);
 */
void Vofa_DataAnalyze(VOFA_TRAN_DEVICE device) {
    //------------------------------
    //监视是否有数据
    Vofa_Monitor(device);
    //------------------------------
    //解析数据
    if (Vofa_getDataStuts == 1) {
        Vofa_getDataStuts = 0;
        Vofa_changeData = Vofa_GetData(Vofa_dataLen);
    }
    //------------------------------
    //设置参数
    if (Vofa_processDataStatus == 1) {
        Vofa_processDataStatus = 0;
        Vofa_SetData();
    }
    //------------------------------
    //回馈信息
    if (Vofa_setDataStatus == 1) {
        Vofa_setDataStatus = 0;
        Vofa_CallBack(device);
    }
}

/*
 * @brief                   蓝牙打印
 * @paremeter dataType      数据类型
 * @parameter num           数据个数
 * @parameter ...           数据列表
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
        //整型数据处理
        if (dataType == VOFA_INT) {
            //对特殊数据进行处理
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
        //浮点型数据处理
        else if (dataType == VOFA_FLOAT) {
            data_float = va_arg(valist, float);
            //特殊数据处理
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
        //给Vofa进行数据处理用
        if (i + 1 >= num) {
            wireless_uart_send_string("\n");
        }
        else {
            wireless_uart_send_string(",");
        }
    }
    va_end(valist);
}
