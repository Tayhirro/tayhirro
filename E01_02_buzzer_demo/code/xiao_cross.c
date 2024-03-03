/*
 * xiao_cross.c
 *
 *  Created on: 2024��3��3��
 *      Author: Tayhirro
 */
#include "xiao_cross.h"


//----------------------------------------
CROSS_STATUS Cross_status=CROSS_NONE;

//----------------------------------------
//ʮ�ּ��״̬
uint8 Cross_BeginStatus_Camera = 0;    //����ͷ�жϿ�ʼ״̬��
//uint8 Circle_leftBeginStatus_Elec = 0;      //����ж��󻷵���ʼ״̬��
//uint8 Circle_rightBeginStatus_Elec = 0;     //����ж��һ�����ʼ״̬��

//----------------------------------------
//����ͷ - �жϱ����Ƿ���ʧ
uint8 none_left_line_cross = 0,                   //�ж�������Ƿ���ʧ
        none_right_line_cross = 0;                //�ж��ұ����Ƿ���ʧ
uint8 have_left_line_cross = 0,                   //�ж��Ƿ���������
        have_right_line_cross = 0;                //�ж��Ƿ�����ұ���
//----------------------------------------


/*
 * @brief               ͨ�����ǵ��ֵ,�ж��Ƿ�Ϊʮ��·��
 * @attention           ʹ������ͷ���,��Ҫ������ͷ���г�ʼ��
 */

void Cross_CheckCamera(void) {
    //ʮ��
    if (Cross_status == CROSS_NONE && Image_LptLeft_Found && Image_LptRight_Found) {
        //���Դ���
        Circle_status = CROSS_BEGIN;
        Encoder_Begin(ENCODER_MOTOR_2);
//        put_int32(70, 1);
        gpio_set_level(P20_8, GPIO_LOW);
        gpio_set_level(P20_9, GPIO_LOW);
    }
}



