/*
 * xiao_encoder.h
 *
 *  Created on: 2023��4��29��
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_ENCODER_H_
#define CODE_XIAO_ENCODER_H_

#include "../libraries/zf_common/zf_common_headfile.h"

//------------------------------ͨ�ú�------------------------------
#define ENCODER_1_DIR               (TIM5_ENCODER)              //�����������(���1)
#define ENCODER_2_DIR               (TIM6_ENCODER)              //�����������(���2)
#define ENCODER_1_DIR_PULSE         (TIM5_ENCODER_CH1_P10_3)    //���1��������
#define ENCODER_1_DIR_DIR           (TIM5_ENCODER_CH2_P10_1)    //���1DIR��
#define ENCODER_2_DIR_PULSE         (TIM6_ENCODER_CH1_P20_3)    //���2��������
#define ENCODER_2_DIR_DIR           (TIM6_ENCODER_CH2_P20_0)    //���2DIR��

#define ENCODER_FILTER_MAX          (4)

//------------------------------���ݶ������------------------------------
typedef enum {
    ENCODER_MOTOR_1 = 0x00,
    ENCODER_MOTOR_2 = 0x01,
}ENCODER_MOTOR_SELECT;

//����ӿ�
extern int16 Encoder_1Data;
extern int16 Encoder_2Data;
extern int16 Encoder_1DataRead;
extern int16 Encoder_2DataRead;
extern uint8 Encoder_readFinishStatus;

extern int32 Encoder_sum_Motor1;                           //���1����
extern int32 Encoder_sum_Motor2;                           //���2����

void Encoder_Init(void);
void Encoder_SpeedRead(void);
void Encoder_Begin(ENCODER_MOTOR_SELECT motorSelect);
void Encoder_End(ENCODER_MOTOR_SELECT motorSelect);
void Encoder_Count(void);
void Encoder_Clear(ENCODER_MOTOR_SELECT motorSelect);

#endif /* CODE_XIAO_ENCODER_H_ */
