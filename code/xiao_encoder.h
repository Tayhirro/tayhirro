/*
 * xiao_encoder.h
 *
 *  Created on: 2023年4月29日
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_ENCODER_H_
#define CODE_XIAO_ENCODER_H_

#include "../libraries/zf_common/zf_common_headfile.h"

//------------------------------通用宏------------------------------
#define ENCODER_1_DIR               (TIM5_ENCODER)              //带方向编码器(电机1)
#define ENCODER_2_DIR               (TIM6_ENCODER)              //带方向编码器(电机2)
#define ENCODER_1_DIR_PULSE         (TIM5_ENCODER_CH1_P10_3)    //电机1计数引脚
#define ENCODER_1_DIR_DIR           (TIM5_ENCODER_CH2_P10_1)    //电机1DIR脚
#define ENCODER_2_DIR_PULSE         (TIM6_ENCODER_CH1_P20_3)    //电机2计数引脚
#define ENCODER_2_DIR_DIR           (TIM6_ENCODER_CH2_P20_0)    //电机2DIR脚

#define ENCODER_FILTER_MAX          (4)

//------------------------------数据定义相关------------------------------
typedef enum {
    ENCODER_MOTOR_1 = 0x00,
    ENCODER_MOTOR_2 = 0x01,
}ENCODER_MOTOR_SELECT;

//对外接口
extern int16 Encoder_1Data;
extern int16 Encoder_2Data;
extern int16 Encoder_1DataRead;
extern int16 Encoder_2DataRead;
extern uint8 Encoder_readFinishStatus;

extern int32 Encoder_sum_Motor1;                           //电机1积分
extern int32 Encoder_sum_Motor2;                           //电机2积分


extern int32 Encoder_sum_Motor1_global;
extern int32 Encoder_sum_Motor2_global;
void Encoder_Init(void);
void Encoder_SpeedRead(void);
void Encoder_Begin(ENCODER_MOTOR_SELECT motorSelect);
void Encoder_End(ENCODER_MOTOR_SELECT motorSelect);
void Encoder_Count(void);
void Encoder_Clear(ENCODER_MOTOR_SELECT motorSelect);

#endif /* CODE_XIAO_ENCODER_H_ */
