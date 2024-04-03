/*
 * xiao_grage.h
 *
 *  Created on: 2023��7��7��
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_GRAGE_H_
#define CODE_XIAO_GRAGE_H_
#include "xiao_common.h"
#include "xiao_image_processing.h"
#include "xiao_gyroscope.h"
#include "xiao_encoder.h"
#include "../user/cpu0_main.h"

//------------------------------ͨ�ú�------------------------------
#define GRAGE_OUT_PIN   (P11_2)             //������
#define GRAGE_PART_PIN  (P32_4)             //ͣ����

//------------------------------ͨ��ö��------------------------------
typedef enum {
    GRAGE_MAGNET = 0x00,
    GRAGE_CAMERA = 0x01,
}GRAGE_DETECTION_MODE;

typedef enum {
    GRAGE_NONE = 0x00,
    GRAGE_IN_BEGIN_LEFT = 0x10,
    GRAGE_IN_BEGIN_RIGHT = 0x11,
    GRAGE_IN_GO_STRAIGHT_LEFT = 0x12,
    GRAGE_IN_GO_STRAIGHT_RIGHT = 0x13,
    GRAGE_IN_END_LEFT = 0x14,
    GRAGE_IN_END_RIGHT = 0x15,
    GRAGE_OUT_BEGINT = 0x20,
    GRAGE_OUT_END = 0x21,
}GRAGE_STATUS;

//------------------------------�ⲿ�ӿ�------------------------------
extern GRAGE_STATUS Grage_grageStatus;
extern uint8 Grage_isDeparture;
//------------------------------_��������_------------------------------
extern int8 Grage_stroageSpeedVar_Motor1_Left;        //�������1�仯��       -       ����:1
extern int8 Grage_stroageSpeedVar_Motor2_Left;        //�������2�仯��       -       ����:1
extern float Grage_inAngle_Thre;                      //���Ƕ���ֵ           -        ����:0.5
extern int16 Grage_inStraight_Thre;                   //���ֱ����ֵ           -        ����:100
extern uint8 Grage_outWarehous_Status;                  //����״̬��
//------------------------------_��������_------------------------------

void Grage_Departure_Check(void);
void Grage_Storage(void);
void Grage_Storage_Check(GRAGE_DETECTION_MODE detectMode);
extern double isConvexHull_left(int numPoints);
extern double isConvexHull_right(int numPoints) ;
extern void garage_check();
#endif /* CODE_XIAO_GRAGE_H_ */
