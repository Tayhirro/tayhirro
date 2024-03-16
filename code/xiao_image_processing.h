/*
 * xiao_image_processing.h
 *
 *  Created on: 2023年5月14日
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

//CAMERA_SELECTION内容解释
//为 1 的时候,选择总钻风
//为 2 的时候,选择小钻风
//为 3 的时候,选择凌瞳
#define CAMERA_SELECTION        (1)
#if CAMERA_SELECTION == 1       //选择总钻风
#define IMAGE_HEIGHT            (120)
#define IMAGE_WIDTH             (188)
#elif CAMERA_SELECTION == 2     //选择小钻风
#define IMAGE_HEIGHT            (OV7725_H)
#define IMAGE_WIDTH             (OV7725_W)
#elif CAMERA_SELECTION == 3     //选择凌瞳
#define IMAGE_HEIGHT            (SCC8660_H)
#define IMAGE_WIDTH             (SCC8660_W)
#endif

#define IMAGE_LINE_MAX_NUM (90)         //图片边线像素点最大个数
//------------------------------------------------------------
//所有状态机
extern uint8 Image_Process_Status;
extern uint8 Image_Process_Status_inv;
//------------------------------------------------------------
//基本数据
//1. 原图左右边线
//2. 边线左右数据
extern uint8   Image_iptsLeft[90][2];           //左边线坐标存储 - 0代表x坐标,1代表y坐标
extern uint8   Image_iptsRight[90][2];          //右边线坐标存储
extern uint8   Image_iptsLeftNum;               //左边线像素点个数
extern uint8   Image_iptsRightNum;              //右边线像素点个数


//逆透视变换曲线
extern uint8 Image_rptsLeft[90][2];             //左边线去畸变+逆透视变换坐标存储
extern uint8 Image_rptsRight[90][2];            //右边线去畸变+逆透视变换坐标存储
extern uint8 Image_rptsLeftNum;                 //左边线去畸变+逆透视变换像素点个数
extern uint8 Image_rptsRightNum;                //右边线去畸变+逆透视变换像素点个数



//------------------------------
//点集三角滤波相关
extern const uint8 Image_linerBlurKernel;       //三角滤波时候的区块边长
extern uint8 Image_rptsLeftb[90][2];            //三角滤波后的左边线坐标存储
extern uint8 Image_rptsRightb[90][2];           //三角滤波后的右边线坐标存储
extern uint8 Image_rptsLeftbNum;                //三角滤波后的左边线长度
extern uint8 Image_rptsRightbNum;               //三角滤波后的右边线长度
//点集三角滤波相关
extern uint8 Image_iptsLeftb[90][2];            //三角滤波后的左边线坐标存储
extern uint8 Image_iptsRightb[90][2];           //三角滤波后的右边线坐标存储
extern uint8 Image_iptsLeftbNum;                //三角滤波后的左边线长度
extern uint8 Image_iptsRightbNum;               //三角滤波后的右边线长度
//------------------------------
//等距采样相关
extern const float Image_sampleDist;           //等距采用采样的距离
extern uint8 Image_rptsLefts[90][2];           //等距采样后的左边线坐标存储
extern uint8 Image_rptsRights[90][2];          //等距采样后的右边线坐标存储
extern uint8 Image_rptsLeftsNum;               //等距采样后的左边线长度
extern uint8 Image_rptsRightsNum;              //等距采样后的右边线长度
//等距采样相关
extern uint8 Image_iptsLefts[90][2];           //等距采样后的左边线坐标存储
extern uint8 Image_iptsRights[90][2];          //等距采样后的右边线坐标存储
extern uint8 Image_iptsLeftsNum;               //等距采样后的左边线长度
extern uint8 Image_iptsRightsNum;              //等距采样后的右边线长度
//------------------------------
//边线局部角度变化率相关
extern const float Image_angleDist;             //计算边线转角时,三个计算点的距离
extern float Image_rptsLefta[90];               //左边线对应点处的角度大小
extern float Image_rptsRighta[90];              //右边线对应点处的角度大小
extern uint8 Image_rptsLeftaNum;                //左边线点的个数
extern uint8 Image_rptsRightaNum;               //右边线点的个数
//边线局部角度变化率相关
extern float Image_iptsLefta[90];               //左边线对应点处的角度大小
extern float Image_iptsRighta[90];              //右边线对应点处的角度大小
extern uint8 Image_iptsLeftaNum;                //左边线点的个数
extern uint8 Image_iptsRightaNum;               //右边线点的个数

//------------------------------
//角度变化率非极大抑制相关
extern float Image_rptsLeftan[90];              //左边线区域最大角存储
extern float Image_rptsRightan[90];             //右边线区域最大角存储
extern uint8 Image_rptsLeftanNum;               //左边线点的个数
extern uint8 Image_rptsRightanNum;              //右边线点的个数
//------------------------------
extern float Image_iptsLeftan[90];              //左边线区域最大角存储
extern float Image_iptsRightan[90];             //右边线区域最大角存储
extern uint8 Image_iptsLeftanNum;               //左边线点的个数
extern uint8 Image_iptsRightanNum;              //右边线点的个数
//------------------------------
//左右变线跟踪相关
extern uint8 Image_rptsLeftc[90][2];            //左边线跟踪得到的中线数据
extern uint8 Image_rptsRightc[90][2];           //右边线跟踪得到的中线数据
extern uint8 Image_rptsLeftcNum;                //左边线跟踪得到的中线的线长
extern uint8 Image_rptsRightcNum;               //右边线跟踪得到的中线的线长
//----------------------------------
extern uint8 Image_centerLine[IMAGE_LINE_MAX_NUM][2];
extern uint8 Image_centerLineNum;          //中线长度
extern uint8 Image_centerLine_Bak[IMAGE_LINE_MAX_NUM][2];
extern uint8 Image_centerLineNum_Bak;

//------------------------------角点寻找相关------------------------------
//------------------------------
//Y角点
extern uint8 Image_YptLeft_rptsLefts_id;                       //左边线Y角点id
extern uint8 Image_YptRight_rptsRights_id;                     //右边线Y角点id
extern uint8 Image_HptLeft_rptsLefts_id;
extern bool  Image_YptLeft_Found;                              //左边线Y角点找到判定
extern bool  Image_YptRight_Found;                             //右边线Y角点找到判断
extern bool Image_HptLeft_Found;
extern bool Image_HptRight_Found;
//------------------------------
//L角点
extern uint8 Image_LptLeft_rptsLefts_id;                       //左边线L角点id
extern uint8 Image_LptRight_rptsRights_id;                     //右边线L角点id
extern bool  Image_LptLeft_Found;                              //左边线L角点找到判定
extern bool  Image_LptRight_Found;                             //右边线L角点找到判断
//------------------------------
//长直道
extern bool  Image_isStraightLeft;                             //左边线是否为直道
extern bool  Image_isStraightRight;                            //右边线是否为直道
//------------------------------
//弯道
extern bool  Image_isTurnLeft;                                 //左边线是否为弯道
extern bool  Image_isTurnRight;                                //右边线是否为弯道

//------------------------------调试参数处理------------------------------
//用于调试的参数(为了作区分,这里的标头起始字母用小写处理, 同时使用下划线命名法)
extern uint8 image_thre;                                       //边线处理的初始阈值
extern uint8 image_begin_x;                                    //边线处理的起始x坐标偏离中心的距离
extern uint8 image_begin_y;                                    //边线处理起始的y坐标
extern uint8 image_block_size;                                 //区域二值化的区域边长
extern uint8 image_block_clip_value;                           //修正的经验参数(一般为2~5)
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
//处理函数
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
