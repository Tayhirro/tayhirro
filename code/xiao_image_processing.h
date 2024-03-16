/*
 * xiao_image_processing.h
 *
 *  Created on: 2023��5��14��
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_IMAGE_PROCESSING_H_
#define CODE_XIAO_IMAGE_PROCESSING_H_

#include "../libraries/zf_common/zf_common_headfile.h"
#include "../libraries/zf_common/zf_common_font.h"
#include "xiao_camera_processing.h"
#include "xiao_basic_function.h"
#include "pico_link_ii.h"
#include "pico_gy.h"
#include "xiao_grage.h"
#include "xiao_circle.h"
#include "xiao_show.h"

//CAMERA_SELECTION���ݽ���
//Ϊ 1 ��ʱ��,ѡ�������
//Ϊ 2 ��ʱ��,ѡ��С���
//Ϊ 3 ��ʱ��,ѡ����ͫ
#define CAMERA_SELECTION        (1)
#if CAMERA_SELECTION == 1       //ѡ�������
#define IMAGE_HEIGHT            (120)
#define IMAGE_WIDTH             (188)
#elif CAMERA_SELECTION == 2     //ѡ��С���
#define IMAGE_HEIGHT            (OV7725_H)
#define IMAGE_WIDTH             (OV7725_W)
#elif CAMERA_SELECTION == 3     //ѡ����ͫ
#define IMAGE_HEIGHT            (SCC8660_H)
#define IMAGE_WIDTH             (SCC8660_W)
#endif

#define IMAGE_LINE_MAX_NUM (90)         //ͼƬ�������ص�������
//------------------------------------------------------------
//����״̬��
extern uint8 Image_Process_Status;
extern uint8 Image_Process_Status_inv;
//------------------------------------------------------------
//��������
//1. ԭͼ���ұ���
//2. ������������
extern uint8   Image_iptsLeft[90][2];           //���������洢 - 0����x����,1����y����
extern uint8   Image_iptsRight[90][2];          //�ұ�������洢
extern uint8   Image_iptsLeftNum;               //��������ص����
extern uint8   Image_iptsRightNum;              //�ұ������ص����


//��͸�ӱ任����
extern uint8 Image_rptsLeft[90][2];             //�����ȥ����+��͸�ӱ任����洢
extern uint8 Image_rptsRight[90][2];            //�ұ���ȥ����+��͸�ӱ任����洢
extern uint8 Image_rptsLeftNum;                 //�����ȥ����+��͸�ӱ任���ص����
extern uint8 Image_rptsRightNum;                //�ұ���ȥ����+��͸�ӱ任���ص����



//------------------------------
//�㼯�����˲����
extern const uint8 Image_linerBlurKernel;       //�����˲�ʱ�������߳�
extern uint8 Image_rptsLeftb[90][2];            //�����˲�������������洢
extern uint8 Image_rptsRightb[90][2];           //�����˲�����ұ�������洢
extern uint8 Image_rptsLeftbNum;                //�����˲��������߳���
extern uint8 Image_rptsRightbNum;               //�����˲�����ұ��߳���
//�㼯�����˲����
extern uint8 Image_iptsLeftb[90][2];            //�����˲�������������洢
extern uint8 Image_iptsRightb[90][2];           //�����˲�����ұ�������洢
extern uint8 Image_iptsLeftbNum;                //�����˲��������߳���
extern uint8 Image_iptsRightbNum;               //�����˲�����ұ��߳���
//------------------------------
//�Ⱦ�������
extern const float Image_sampleDist;           //�Ⱦ���ò����ľ���
extern uint8 Image_rptsLefts[90][2];           //�Ⱦ����������������洢
extern uint8 Image_rptsRights[90][2];          //�Ⱦ��������ұ�������洢
extern uint8 Image_rptsLeftsNum;               //�Ⱦ�����������߳���
extern uint8 Image_rptsRightsNum;              //�Ⱦ��������ұ��߳���
//�Ⱦ�������
extern uint8 Image_iptsLefts[90][2];           //�Ⱦ����������������洢
extern uint8 Image_iptsRights[90][2];          //�Ⱦ��������ұ�������洢
extern uint8 Image_iptsLeftsNum;               //�Ⱦ�����������߳���
extern uint8 Image_iptsRightsNum;              //�Ⱦ��������ұ��߳���
//------------------------------
//���߾ֲ��Ƕȱ仯�����
extern const float Image_angleDist;             //�������ת��ʱ,���������ľ���
extern float Image_rptsLefta[90];               //����߶�Ӧ�㴦�ĽǶȴ�С
extern float Image_rptsRighta[90];              //�ұ��߶�Ӧ�㴦�ĽǶȴ�С
extern uint8 Image_rptsLeftaNum;                //����ߵ�ĸ���
extern uint8 Image_rptsRightaNum;               //�ұ��ߵ�ĸ���
//���߾ֲ��Ƕȱ仯�����
extern float Image_iptsLefta[90];               //����߶�Ӧ�㴦�ĽǶȴ�С
extern float Image_iptsRighta[90];              //�ұ��߶�Ӧ�㴦�ĽǶȴ�С
extern uint8 Image_iptsLeftaNum;                //����ߵ�ĸ���
extern uint8 Image_iptsRightaNum;               //�ұ��ߵ�ĸ���

//------------------------------
//�Ƕȱ仯�ʷǼ����������
extern float Image_rptsLeftan[90];              //������������Ǵ洢
extern float Image_rptsRightan[90];             //�ұ����������Ǵ洢
extern uint8 Image_rptsLeftanNum;               //����ߵ�ĸ���
extern uint8 Image_rptsRightanNum;              //�ұ��ߵ�ĸ���
//------------------------------
extern float Image_iptsLeftan[90];              //������������Ǵ洢
extern float Image_iptsRightan[90];             //�ұ����������Ǵ洢
extern uint8 Image_iptsLeftanNum;               //����ߵ�ĸ���
extern uint8 Image_iptsRightanNum;              //�ұ��ߵ�ĸ���
//------------------------------
//���ұ��߸������
extern uint8 Image_rptsLeftc[90][2];            //����߸��ٵõ�����������
extern uint8 Image_rptsRightc[90][2];           //�ұ��߸��ٵõ�����������
extern uint8 Image_rptsLeftcNum;                //����߸��ٵõ������ߵ��߳�
extern uint8 Image_rptsRightcNum;               //�ұ��߸��ٵõ������ߵ��߳�
//----------------------------------
extern uint8 Image_centerLine[IMAGE_LINE_MAX_NUM][2];
extern uint8 Image_centerLineNum;          //���߳���
extern uint8 Image_centerLine_Bak[IMAGE_LINE_MAX_NUM][2];
extern uint8 Image_centerLineNum_Bak;

//------------------------------�ǵ�Ѱ�����------------------------------
//------------------------------
//Y�ǵ�
extern uint8 Image_YptLeft_rptsLefts_id;                       //�����Y�ǵ�id
extern uint8 Image_YptRight_rptsRights_id;                     //�ұ���Y�ǵ�id
extern uint8 Image_HptLeft_rptsLefts_id;
extern bool  Image_YptLeft_Found;                              //�����Y�ǵ��ҵ��ж�
extern bool  Image_YptRight_Found;                             //�ұ���Y�ǵ��ҵ��ж�
extern bool Image_HptLeft_Found;
extern bool Image_HptRight_Found;
//------------------------------
//L�ǵ�
extern uint8 Image_LptLeft_rptsLefts_id;                       //�����L�ǵ�id
extern uint8 Image_LptRight_rptsRights_id;                     //�ұ���L�ǵ�id
extern bool  Image_LptLeft_Found;                              //�����L�ǵ��ҵ��ж�
extern bool  Image_LptRight_Found;                             //�ұ���L�ǵ��ҵ��ж�
//------------------------------
//��ֱ��
extern bool  Image_isStraightLeft;                             //������Ƿ�Ϊֱ��
extern bool  Image_isStraightRight;                            //�ұ����Ƿ�Ϊֱ��
//------------------------------
//���
extern bool  Image_isTurnLeft;                                 //������Ƿ�Ϊ���
extern bool  Image_isTurnRight;                                //�ұ����Ƿ�Ϊ���

//------------------------------���Բ�������------------------------------
//���ڵ��ԵĲ���(Ϊ��������,����ı�ͷ��ʼ��ĸ��Сд����, ͬʱʹ���»���������)
extern uint8 image_thre;                                       //���ߴ���ĳ�ʼ��ֵ
extern uint8 image_begin_x;                                    //���ߴ������ʼx����ƫ�����ĵľ���
extern uint8 image_begin_y;                                    //���ߴ�����ʼ��y����
extern uint8 image_block_size;                                 //�����ֵ��������߳�
extern uint8 image_block_clip_value;                           //�����ľ������(һ��Ϊ2~5)
extern uint8 localThressss;
extern uint8 Bak_Status;
extern uint8 Image_threCnt_Thre;

typedef enum {
    IMAGE_IPS200 = 0x00,
    IMAGE_TFT180 = 0x01,
}IMAGE_SCREEN;

typedef enum {
    IMAGE_ORIGIN = 0x00,
    IMAGE_MAPPING = 0x01,
    IMAGE_MIDLINE_LEFT = 0x02,
    IMAGE_MIDLINE_RIGHT = 0x03,
    IMAGE_CLEAR_ORIGIN = 0x04,
    IMAGE_CLEAR_MAPPING = 0x05,
    IMAGE_CLEAR_MIDLINE_LEFT = 0x06,
    IMAGE_CLEAR_MIDLINE_RIGHT = 0x07,
}IMAGE_SHOW_TYPE;

typedef enum {
    IMAGE_LCORNER_NONE = 0x10,
    IMAGE_LCORNER_BEGIN_LEFT = 0x00,
    IMAGE_LCORNER_BEGIN_RIGHT = 0x01,
    IMAGE_LCORNER_IS_GRAGE_LEFT = 0x02,
    IMAGE_LCORNER_IS_GRAGE_RIGHT = 0x03,
    IMAGE_LCORNER_IS_CIRCLE_LEFT = 0x04,
    IMAGE_LCORNER_IS_CIRCLE_RIGHT = 0x05,
}IMAGE_LCORNER_JUDGE;

//------------------------------------------------------------
//������
void Image_Init(void);
uint8 Image_Processing_OtsuGetThresh(const uint8* image);
void Image_ShowLine(uint16 beg_x, uint16 beg_y, IMAGE_SCREEN screen, IMAGE_SHOW_TYPE showType);
void Image_Process(uint8* image);
void Image_Process_inv(uint8* image_inv);
void Image_GetAngleInit(void);
void Image_GetAngle(uint8 beg_x, uint8 beg_y, IMAGE_SCREEN screen);
void Image_ShowCorners(uint8 beg_x, uint8 beg_y, IMAGE_SCREEN screen);
//void Image_FindCorners(void);
uint8 IMAGE_AT(uint8* image, int16 x, int16 y);
void Image_ShowCorner(uint8 x, uint8 y, rgb565_color_enum color);
void Image_LCornerCheck(void);
bool Image_LineIsClosed(uint8 select);


#endif /* CODE_XIAO_IMAGE_PROCESSING_H_ */
