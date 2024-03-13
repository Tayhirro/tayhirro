/*
 * xiao_image_processing.c
 *
 *  Created on: 2023年5月14日
 *      Author: Jayden_NaN
 */

#include "xiao_image_processing.h"

//------------------------------------------------------------
//图像坐标为
// :------------------>x
// |
// |
// |
// |
// |
// |
// V
// y
//迷宫巡线相关内容
/*
 * 前进方向定义:
 *  0
 *3   1
 *  2
 */
//------------------------------------------------------------
//基本信息说明
//1. 像素点与实际距离的对应
//      45cm ~ 45pixel
//2. 坐标数组说明
//      规定,所有坐标数组 0 代表x坐标; 1 代表y坐标
//3. 角度变化率非极大抑制相关
//      区域内非极大角的角设置为了0


//------------------------------用到的宏------------------------------
#define IMAGE_ANGLE_CHANGE_UP_KEY       (P22_3)
#define IMAGE_ANGLE_CHANGE_DOWN_KEY     (P22_2)
#define IMAGE_ANGLE_LINE_SELCET_KEY     (P21_1)
#define IMAGE_ANGLE_COLOR               (RGB565_YELLOW)

//------------------------------状态机------------------------------
uint8 Image_Process_Status = 0;
uint8 Image_Process_Status_inv = 0;
uint8 Image_isUsefulData_Status = 0;
//------------------------------变量------------------------------
//------------------------------
//基本变量
const float Image_pixelPreMeter = 100;              //每米118.42个像素点(38cm ~ 45pixel)
const float Image_roadWidth     = 0.45;             //赛道的宽度 - (单位为m)
const float Image_PI = 3.14159265358;               //PI(手打的)
uint8 Image_disPictureCnt = 0;                      //丢弃图片的数量计数
const uint8 Image_disPictureCnt_Thre = 10;          //丢弃图片的数量阈值
uint16 Image_threSum = 0;                           //图片阈值积分
uint8 Image_threCnt_Thre = 10;                      //用来计算大津法的图片数量
uint8 clip_middle=5;

//------------------------------图片处理相关------------------------------
//------------------------------
//找左右边线
uint8   Image_iptsLeft[IMAGE_LINE_MAX_NUM][2];          //左边线坐标存储 - 0代表x坐标,1代表y坐标
uint8   Image_iptsRight[IMAGE_LINE_MAX_NUM][2];         //右边线坐标存储
uint8   Image_iptsLeft_Bak[IMAGE_LINE_MAX_NUM][2];      //左边线坐标备份 - 记录上一次处理后的结果
uint8   Image_iptsRight_Bak[IMAGE_LINE_MAX_NUM][2];     //右边线坐标备份

uint8   Image_rptsLeft_Bak[IMAGE_LINE_MAX_NUM][2];
uint8   Image_rptsRight_Bak[IMAGE_LINE_MAX_NUM][2];

uint8   Image_iptsLeftNum;                              //左边线像素点个数
uint8   Image_iptsRightNum;                             //右边线像素点个数
uint8   Image_iptsLeftrNum;                             //左边点实际有用的个数
uint8   Image_iptsRightrNum;                            //右边点实际有用的个数
uint8   Image_iptsLeftNum_Bak;                          //左边线像素点个数备份
uint8   Image_iptsRightNum_Bak;                         //右边线像素点个数备份

uint8   Image_rptsLeftNum_Bak;
uint8   Image_rptsRightNum_Bak;
static int8 Image_dir_front[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1 ,0}};             //转向为前方时转向模型
static int8 Image_dir_frontLeft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1 ,1}};       //转向为左前方时转向模型
static int8 Image_dir_frontRight[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1 ,-1}};      //转向为右前方时转向模型
//------------------------------
//把边线进行去畸变+逆透视变换
uint8 Image_rptsLeft[IMAGE_LINE_MAX_NUM][2];            //左边线去畸变+逆透视变换坐标存储
uint8 Image_rptsRight[IMAGE_LINE_MAX_NUM][2];           //右边线去畸变+逆透视变换坐标存储
uint8 Image_rptsLeftNum;                                //左边线去畸变+逆透视变换像素点个数
uint8 Image_rptsRightNum;                               //右边线去畸变+逆透视变换像素点个数
uint8 Image_rptsLeftrNum = 0;                           //左边线实际有用的点的个数
uint8 Image_rptsRightrNum = 0;                          //右边线实际有用的点的个数
uint8 localThressss;
//------------------------------
//左右边线去逆透视(就是 去畸变 + 逆透视 -> 去畸变)
uint8 Image_invRptsLeft[IMAGE_LINE_MAX_NUM][2];         //左边线去畸变坐标存储
uint8 Image_invRptsRight[IMAGE_LINE_MAX_NUM][2];        //右边线去畸变坐标存储
uint8 Image_invRptsLeftNum;                             //左边线去畸变像素点个数
uint8 Image_invRptsRightNum;                            //右边线像素点去畸变个数
//------------------------------


//点集三角滤波相关
const uint8 Image_linerBlurKernel = 7;                  //三角滤波时候的区块边长
uint8 Image_rptsLeftb[IMAGE_LINE_MAX_NUM][2];           //三角滤波后的左边线坐标存储
uint8 Image_rptsRightb[IMAGE_LINE_MAX_NUM][2];          //三角滤波后的右边线坐标存储
uint8 Image_rptsLeftbNum;                               //三角滤波后的左边线长度
uint8 Image_rptsRightbNum;                              //三角滤波后的右边线长度
//点集三角滤波相关
uint8 Image_iptsLeftb[IMAGE_LINE_MAX_NUM][2];           //三角滤波后的左边线坐标存储
uint8 Image_iptsRightb[IMAGE_LINE_MAX_NUM][2];          //三角滤波后的右边线坐标存储
uint8 Image_iptsLeftbNum;                               //三角滤波后的左边线长度
uint8 Image_iptsRightbNum;                              //三角滤波后的右边线长度


//------------------------------
//等距采样相关
const float Image_sampleDist = 0.01;                    //等距采用采样的距离
uint8 Image_rptsLefts[IMAGE_LINE_MAX_NUM][2];           //等距采样后的左边线坐标存储
uint8 Image_rptsRights[IMAGE_LINE_MAX_NUM][2];          //等距采样后的右边线坐标存储
uint8 Image_rptsLefts_Bak[IMAGE_LINE_MAX_NUM][2];       //等距采样后的左边线坐标存储 - 备份
uint8 Image_rptsRights_Bak[IMAGE_LINE_MAX_NUM][2];      //等距采样后的右边线坐标存储 - 备份
uint8 Image_rptsLeftsNum;                               //等距采样后的左边线长度
uint8 Image_rptsRightsNum;                              //等距采样后的右边线长度
uint8 Image_rptsLeftsNum_Bak;                           //等距采样后的左边线长度    - 备份
uint8 Image_rptsRightsNum_Bak;                          //等距采样后的右边线长度    - 备份
//等距采样相关
uint8 Image_iptsLefts[IMAGE_LINE_MAX_NUM][2];           //等距采样后的左边线坐标存储
uint8 Image_iptsRights[IMAGE_LINE_MAX_NUM][2];          //等距采样后的右边线坐标存储
uint8 Image_iptsLefts_Bak[IMAGE_LINE_MAX_NUM][2];       //等距采样后的左边线坐标存储 - 备份
uint8 Image_iptsRights_Bak[IMAGE_LINE_MAX_NUM][2];      //等距采样后的右边线坐标存储 - 备份
uint8 Image_iptsLeftsNum;                               //等距采样后的左边线长度
uint8 Image_iptsRightsNum;                              //等距采样后的右边线长度
uint8 Image_iptsLeftsNum_Bak;                           //等距采样后的左边线长度    - 备份
uint8 Image_iptsRightsNum_Bak;                          //等距采样后的右边线长度    - 备份
//------------------------------
//------------------------------
//边线局部角度变化率相关
const float Image_angleDist = 0.1;                      //计算边线转角时,三个计算点的距离
float Image_rptsLefta[IMAGE_LINE_MAX_NUM];              //左边线对应点处的角度大小
float Image_rptsRighta[IMAGE_LINE_MAX_NUM];             //右边线对应点处的角度大小
uint8 Image_rptsLeftaNum;                               //左边线点的个数
uint8 Image_rptsRightaNum;                              //右边线点的个数
//边线局部角度变化率相关
float Image_iptsLefta[IMAGE_LINE_MAX_NUM];              //左边线对应点处的角度大小
float Image_iptsRighta[IMAGE_LINE_MAX_NUM];             //右边线对应点处的角度大小
uint8 Image_iptsLeftaNum;                               //左边线点的个数
uint8 Image_iptsRightaNum;                              //右边线点的个数
//------------------------------
//角度变化率非极大抑制相关
float Image_rptsLeftan[IMAGE_LINE_MAX_NUM];             //左边线区域最大角存储
float Image_rptsRightan[IMAGE_LINE_MAX_NUM];            //右边线区域最大角存储
uint8 Image_rptsLeftanNum;                              //左边线点的个数
uint8 Image_rptsRightanNum;                             //右边线点的个数
//角度变化率非极大抑制相关
float Image_iptsLeftan[IMAGE_LINE_MAX_NUM];             //左边线区域最大角存储
float Image_iptsRightan[IMAGE_LINE_MAX_NUM];            //右边线区域最大角存储
uint8 Image_iptsLeftanNum;                              //左边线点的个数
uint8 Image_iptsRightanNum;                             //右边线点的个数

//------------------------------
//左右边线跟踪相关
uint8 Image_rptsLeftc[IMAGE_LINE_MAX_NUM][2];           //左边线跟踪得到的中线数据
uint8 Image_rptsRightc[IMAGE_LINE_MAX_NUM][2];          //右边线跟踪得到的中线数据
uint8 Image_rptsLeftc_Bak[IMAGE_LINE_MAX_NUM][2];       //左边线跟踪得到的中线数据  -   备份
uint8 Image_rptsRightc_Bak[IMAGE_LINE_MAX_NUM][2];      //右边线跟踪得到的中线数据  -   备份
uint8 Image_rptsLeftcNum;                               //左边线跟踪得到的中线的线长
uint8 Image_rptsRightcNum;                              //右边线跟踪得到的中线的线长
uint8 Image_rptsLeftcNum_Bak;                           //左边线跟踪得到的中线的线长 -   备份
uint8 Image_rptsRightcNum_Bak;                          //右边线跟踪得到的中线的线长 -   备份
//左右边线跟踪相关
uint8 Image_iptsLeftc[IMAGE_LINE_MAX_NUM][2];           //左边线跟踪得到的中线数据
uint8 Image_iptsRightc[IMAGE_LINE_MAX_NUM][2];          //右边线跟踪得到的中线数据
uint8 Image_iptsLeftc_Bak[IMAGE_LINE_MAX_NUM][2];       //左边线跟踪得到的中线数据  -   备份
uint8 Image_iptsRightc_Bak[IMAGE_LINE_MAX_NUM][2];      //右边线跟踪得到的中线数据  -   备份
uint8 Image_iptsLeftcNum;                               //左边线跟踪得到的中线的线长
uint8 Image_iptsRightcNum;                              //右边线跟踪得到的中线的线长
uint8 Image_iptsLeftcNum_Bak;                           //左边线跟踪得到的中线的线长 -   备份
uint8 Image_iptsRightcNum_Bak;                          //右边线跟踪得到的中线的线长 -   备份
//中线存储数据
uint8 Image_centerLine[IMAGE_LINE_MAX_NUM][2];
uint8 Image_centerLineNum;          //中线长度
uint8 Image_centerLine_Bak[IMAGE_LINE_MAX_NUM][2];
uint8 Image_centerLineNum_Bak;


//------------------------------角点寻找相关------------------------------
//------------------------------
//角点个数
uint8 Image_cornerNumLeft = 0;                          //角点个数
uint8 Image_cornerNumRight = 0;                         //角点个数
//------------------------------
//Y角点
uint8 Image_YptLeft_rptsLefts_id;                       //左边线Y角点id
uint8 Image_YptRight_rptsRights_id;                     //右边线Y角点id
bool  Image_YptLeft_Found;                              //左边线Y角点找到判定
bool  Image_YptRight_Found;
//右边线Y角点找到判
bool  Image_HptLeft_Found;
bool  Image_HptRight_Found;
uint8 Image_HptLeft_rptsLefts_id;
//------------------------------
//L角点
uint8 Image_LptLeft_rptsLefts_id;                       //左边线L角点id
uint8 Image_LptRight_rptsRights_id;                     //右边线L角点id
bool  Image_LptLeft_Found;                              //左边线L角点找到判定
bool  Image_LptRight_Found;                             //右边线L角点找到判断
//------------------------------
//长直道
bool  Image_isStraightLeft=true;                             //左边线是否为直道
bool  Image_isStraightRight=true;                            //右边线是否为直道
//------------------------------
//弯道
bool  Image_isTurnLeft;                                 //左边线是否为弯道
bool  Image_isTurnRight;                                //右边线是否为弯道
//------------------------------
//获取选中角点数据
uint8 Image_angleCntLeft = 0;                           //记录当前选中的是第几个点 - 左边线
uint8 Image_angleCntRight = 0;                          //记录当前选中的是第几个点 - 右边线
uint8 Image_lineSeclet = 0;                             //边线选择:0 - 左边线; 1 - 右边线
uint8 Image_angleBlockLen = 3;                          //显示区块的边线长度

//------------------------------元素判断相关------------------------------
bool Image_isGrage = 0;                                 //检测是否是车库
bool Image_isCircle = 0;                                //检测是否是环岛
bool Image_LjudgeFinish = 0;                            //L角点元素判断完成状态机
IMAGE_LCORNER_JUDGE Image_LCornerJude_Status = IMAGE_LCORNER_NONE;           //L角点判断
uint16 Image_GrageJudge_Thre = 10000;                   //车库判断的编码器积分距离

//---------------------------元素判断变量--------------------------------
//------------------------------调试参数处理------------------------------
//用于调试的参数(为了作区分,这里的标头起始字母用小写处理, 同时使用下划线命名法)
uint8 image_thre = 140;                                  //边线处理的初始阈值
uint8 image_begin_x = IMAGE_WIDTH / 2;                  //边线处理的起始x坐标偏离中心的距离
uint8 image_begin_y = IMAGE_HEIGHT -10;                //边线处理起始的y坐标
uint8 image_block_size = 7;                             //区域二值化的区域边长
uint8 image_block_clip_value = 1;                       //修正的经验参数(一般为2~5)

/*
 * @brief               获取图像数据
 * @parameter image     图片
 * @parameter x         像素点的x坐标
 * @parameter y         像素点的y坐标
 * @return               像素点的灰度值
 */
uint8 IMAGE_AT(uint8* image, int16 x, int16 y) {
    return *(image + y * IMAGE_WIDTH + x);
}

/*
 * @brief               显示边线
 * @parameter beg_x     在屏幕上显示的起始X坐标
 * @parameter beg_y     在屏幕上显示的起始y坐标
 * @parameter screen    显示的屏幕
 * @parameter showType  显示的类型 - IMAGE_ORIGIN:显示原图像的边线; IMAGE_MAPPING:显示映射后的图像边线
 * @example             Image_ShowLine(0, 0, IMAGE_IPS200, IMAGE_ORIGIN);
 * @attention           左边线 - 绿色
 *                      右边线 - 红色
 */
void Image_ShowLine(uint16 beg_x, uint16 beg_y, IMAGE_SCREEN screen, IMAGE_SHOW_TYPE showType) {
    //只有当图像处理工作完成才能进行显示图像信息的工作
    if (Image_Process_Status == 1) {
        //如果是IMAGE_IPS200的话这样去处理
        if (screen == IMAGE_IPS200) {
            if (showType == IMAGE_ORIGIN) {
                //显示左边线
                for (int i = 0; i < Image_iptsLeftNum; ++i) {
                    ips200_draw_point(beg_x + Image_iptsLeft[i][0], beg_y + Image_iptsLeft[i][1], RGB565_GREEN);
                }
                //显示右边线
                for (int i = 0; i < Image_iptsRightNum; ++i) {
                    ips200_draw_point(beg_x + Image_iptsRight[i][0],beg_y + Image_iptsRight[i][1], RGB565_RED);
                }
                for(int i=0;i<Image_centerLineNum;++i){
                    ips200_draw_point(beg_x + Image_centerLine[i][0],beg_y + Image_centerLine[i][1], RGB565_BLUE);
                                }
            }
                //显示中线

            else if (showType == IMAGE_MAPPING) {
                //显示左边线
                for (int i = 0; i < Image_rptsLeftsNum; ++i) {
                    ips200_draw_point(beg_x + Image_rptsLefts[i][0], beg_y + Image_rptsLefts[i][1], RGB565_GREEN);
                }
                //显示右边线
                for (int i = 0; i < Image_rptsRightsNum; ++i) {
                    ips200_draw_point(beg_x + Image_rptsRights[i][0],beg_y + Image_rptsRights[i][1], RGB565_RED);
                }
            }
            else if (showType == IMAGE_MIDLINE_LEFT) {
                //显示左边线的中线
                for (int i = 0; i < Image_rptsLeftcNum; ++i) {
                    ips200_draw_point(beg_x + Image_rptsLeftc[i][0], beg_y + Image_rptsLeftc[i][1], RGB565_BLUE);
                }
            }
            else if (showType == IMAGE_MIDLINE_RIGHT) {
                //显示右边线的中线
                for (int i = 0; i < Image_rptsRightcNum; ++i) {
                    ips200_draw_point(beg_x + Image_rptsRightc[i][0], beg_y + Image_rptsRightc[i][1], RGB565_BLUE);
                }
            }

            if (Image_isUsefulData_Status) {
                if (showType == IMAGE_CLEAR_ORIGIN) {
                    //清除左边线
                    for (int i = 0; i < Image_iptsLeftNum_Bak; ++i) {
                        ips200_draw_point(beg_x + Image_iptsLeft_Bak[i][0], beg_y + Image_iptsLeft_Bak[i][1], RGB565_WHITE);
                    }
                    //清除右边线
                    for (int i = 0; i < Image_iptsRightNum; ++i) {
                        ips200_draw_point(beg_x + Image_iptsRight_Bak[i][0],beg_y + Image_iptsRight_Bak[i][1], RGB565_WHITE);
                    }
                    //清除中线
                    for(int i=0;i<Image_centerLineNum_Bak;++i){
                        ips200_draw_point(beg_x + Image_centerLine_Bak[i][0],beg_y + Image_centerLine_Bak[i][1], RGB565_WHITE);
                    }
                }
                else if (showType == IMAGE_CLEAR_MAPPING) {
                    //清除左边线 - 映射后
                    for (int i = 0; i < Image_rptsLeftsNum_Bak; ++i) {
                        ips200_draw_point(beg_x + Image_rptsLefts_Bak[i][0], beg_y + Image_rptsLefts_Bak[i][1], RGB565_WHITE);
                    }
                    //清除右边线 - 映射后
                    for (int i = 0; i < Image_rptsRightsNum_Bak; ++i) {
                        ips200_draw_point(beg_x + Image_rptsRights_Bak[i][0],beg_y + Image_rptsRights_Bak[i][1], RGB565_WHITE);
                    }
                }
                else if (showType == IMAGE_CLEAR_MIDLINE_LEFT) {
                    //清除左边线的中线 - 映射后
                    for (int i = 0; i < Image_rptsLeftcNum_Bak; ++i) {
                        ips200_draw_point(beg_x + Image_rptsLeftc_Bak[i][0], beg_y + Image_rptsLeftc_Bak[i][1], RGB565_WHITE);
                    }
                }
                else if (showType == IMAGE_CLEAR_MIDLINE_RIGHT) {
                    //清除右边线的中线 - 映射后
                    for (int i = 0; i < Image_rptsRightcNum_Bak; ++i) {
                        ips200_draw_point(beg_x + Image_rptsRightc_Bak[i][0], beg_y + Image_rptsRightc_Bak[i][1], RGB565_WHITE);
                    }
                }
            }
        }
    }
}

/*
 * @brief               显示角点(蓝色点为角点)
 * @parameter beg_x     在屏幕上显示的起始X坐标
 * @parameter beg_y     在屏幕上显示的起始y坐标
 * @parameter screen    显示的屏幕
 * @example             Image_ShowCorners(0, 130, IMAGE_IPS200);
 */
void Image_ShowCorners(uint8 beg_x, uint8 beg_y, IMAGE_SCREEN screen) {
    if (screen == SCREEN_IPS200) {
        //----------------------------------------
        //显示左边线的角点
        for (uint8 i = 0; i < Image_rptsLeftanNum; ++i) {
            if (Image_rptsLeftan[i] != 0) {
                ips200_draw_point(beg_x + Image_rptsLefts[i][0], beg_y + Image_rptsLefts[i][1], RGB565_BLUE);
            }
        }
        //----------------------------------------
        //显示右边线的角点
        for (uint8 i = 0; i < Image_rptsRightanNum; ++i) {
            if (Image_rptsRightan[i] != 0) {
                ips200_draw_point(beg_x + Image_rptsRights[i][0],beg_y + Image_rptsRights[i][1], RGB565_BLUE);
            }
        }
    }
}

/*
 * @brief                   迷宫巡线 - 自动二值化处理
 * @parameter image         图片
 * @parameter block_size    自适应二值化区域长度(必须为奇数)
 * @parameter clip_value    二值化修正值(经验值,一般为2-5)
 * @parameter x             边线(初始)x坐标
 * @parameter y             边线(初始)y坐标
 * @return                  无
 * @example                 Image_FindLine_LeftHand_Adaptive(image, image_block_size, image_block_clip_value, x1, y1);
 */
static void Image_FindLine_LeftHand_Adaptive(uint8* image, uint8 block_size, int8 clip_value, uint8 x, uint8 y) {
    //传入的块的长度必须为奇数!
    int8 half = block_size / 2;
    uint16 step = 0;
    uint8 dir = 0, turn = 0;
    while (step < Image_iptsLeftNum && half < x && x < IMAGE_WIDTH - half - 1 && half < y && y < IMAGE_HEIGHT - half - 1 && turn < 4) {
        //使用自适应阈值法求解阈值
        //clip_value为经验常数,一般取2~5
        int16 localThres = 0;
        for (int16 dy = -half; dy <= half; ++dy) {
            for (int16 dx = -half; dx <= half; ++dx) {
                localThres += IMAGE_AT(image, x + dx, y + dy);
            }
        }
        localThres /= block_size * block_size;
        localThres -= clip_value;
        localThressss=localThres;
        //做阈值判断
        uint8 frontValue = IMAGE_AT(image, x + Image_dir_front[dir][0], y + Image_dir_front[dir][1]);
        uint8 frontLeftValue = IMAGE_AT(image, x + Image_dir_frontLeft[dir][0], y + Image_dir_frontLeft[dir][1]);

        if (frontValue < localThres) {
            //前方是黑色"墙壁" -- 向右转
            //方向改变,记录转向一次
            //向右转是 顺时针 - 所以要(+1)%4处理
            dir = (dir + 1) % 4;
            ++turn;
        }
        else if (frontLeftValue < localThres){
            //前方是白色,但是左前方是"黑色" - 这种情况就是直道走
            x += Image_dir_front[dir][0];
            y += Image_dir_front[dir][1];
            Image_iptsLeft[step][0] = x;
            Image_iptsLeft[step][1] = y;
            ++step;
            turn = 0;
        }
        else {
            //前方是白色,同时左前方也是"白色" - 这种情况下说明需要向左转弯"贴墙走"
            x += Image_dir_frontLeft[dir][0];
            y += Image_dir_frontLeft[dir][1];
            //向左转是 逆时针 - 所以要(+3)%4处理 (或者说(-1)%4处理)
            dir = (dir + 3) % 4;
            Image_iptsLeft[step][0] = x;
            Image_iptsLeft[step][1] = y;
            ++step;
            turn = 0;
        }
    }
    Image_iptsLeftNum = step;
}
static void Image_FindLine_LeftHand_Adaptive_inv(uint8* image, uint8 block_size, int8 clip_value, uint8 x, uint8 y) {
    //传入的块的长度必须为奇数!
    int8 half = block_size / 2;
    uint16 step = 0;
    uint8 dir = 0, turn = 0;
    while (step < Image_rptsLeftNum && half < x && x < IMAGE_WIDTH - half - 1 && half < y && y < IMAGE_HEIGHT - half - 1 && turn < 4) {
        //使用自适应阈值法求解阈值
        //clip_value为经验常数,一般取2~5
        int16 localThres = 0;
        for (int16 dy = -half; dy <= half; ++dy) {
            for (int16 dx = -half; dx <= half; ++dx) {
                localThres += IMAGE_AT(image, x + dx, y + dy);
            }
        }
        localThres /= block_size * block_size;
        localThres -= clip_value;
        localThressss=localThres;
        //做阈值判断
        uint8 frontValue = IMAGE_AT(image, x + Image_dir_front[dir][0], y + Image_dir_front[dir][1]);
        uint8 frontLeftValue = IMAGE_AT(image, x + Image_dir_frontLeft[dir][0], y + Image_dir_frontLeft[dir][1]);

        if (frontValue < localThres) {
            //前方是黑色"墙壁" -- 向右转
            //方向改变,记录转向一次
            //向右转是 顺时针 - 所以要(+1)%4处理
            dir = (dir + 1) % 4;
            ++turn;
        }
        else if (frontLeftValue < localThres){
            //前方是白色,但是左前方是"黑色" - 这种情况就是直道走
            x += Image_dir_front[dir][0];
            y += Image_dir_front[dir][1];
            Image_rptsLeft[step][0] = x;
            Image_rptsLeft[step][1] = y;
            ++step;
            turn = 0;
        }
        else {
            //前方是白色,同时左前方也是"白色" - 这种情况下说明需要向左转弯"贴墙走"
            x += Image_dir_frontLeft[dir][0];
            y += Image_dir_frontLeft[dir][1];
            //向左转是 逆时针 - 所以要(+3)%4处理 (或者说(-1)%4处理)
            dir = (dir + 3) % 4;
            Image_rptsLeft[step][0] = x;
            Image_rptsLeft[step][1] = y;
            ++step;
            turn = 0;
        }
    }
    Image_rptsLeftNum = step;
}
/*
 * @brief                   迷宫巡线 - 自动二值化处理
 * @parameter image         图片
 * @parameter block_size    自适应二值化区域长度(必须为奇数)
 * @parameter clip_value    二值化修正值(经验值,一般为2-5)
 * @parameter x             边线(初始)x坐标
 * @parameter y             边线(初始)y坐标
 *                          作用:1.防止边线过长数据溢出
 *                              2.边线较短的时候改变num,防止外部访问到"非法"数据
 * @return                  无
 * @example                 Image_FindLine_RightHand_Adaptive(image, image_block_size, image_block_clip_value, x2, y2);
 */
static void Image_FindLine_RightHand_Adaptive(uint8* image, int block_size, int clip_value, int x, int y) {
    //传入的块的长度必须为奇数!
    uint8 half = block_size / 2;
    uint16 step = 0;
    uint8 dir = 0, turn = 0;
    while (step < Image_iptsRightNum && half < x && x < IMAGE_WIDTH - half - 1&& half < y && y < IMAGE_HEIGHT - half - 1 && turn < 4) {
        //使用自适应阈值法求解阈值
        //clip_value为经验常数,一般取2~5
        int16 localThres = 0;
        for (int16 dy = -half; dy <= half; ++dy) {
            for (int16 dx = -half; dx <= half; ++dx) {
                localThres += IMAGE_AT(image, x + dx, y + dy);
            }
        }
        localThres /= block_size * block_size;
        localThres -= clip_value;
        localThressss=localThres;
        //做阈值判断
        uint8 frontValue = IMAGE_AT(image, x + Image_dir_front[dir][0], y + Image_dir_front[dir][1]);
        uint8 frontRightValue = IMAGE_AT(image, x + Image_dir_frontRight[dir][0], y + Image_dir_frontRight[dir][1]);

        if (frontValue < localThres) {
            //前方是黑色"墙壁" -- 向左转
            //方向改变,记录转向一次
            //向左转是 顺时针 - 所以要(+3)%4处理(或者说(-1)%4处理)
            dir = (dir + 3) % 4;
            ++turn;
        }
        else if (frontRightValue < localThres){
            //前方是白色,同时是右前方是"黑色" - 这种情况就是直道走
            x += Image_dir_front[dir][0];
            y += Image_dir_front[dir][1];
            Image_iptsRight[step][0] = x;
            Image_iptsRight[step][1] = y;
            ++step;
            turn = 0;
        }
        else {
            //前方是白色,但是右前方也是"白色" - 这种情况下说明需要向右转弯"贴墙走"
            x += Image_dir_frontRight[dir][0];
            y += Image_dir_frontRight[dir][1];
            //向右转是 顺时针 - 所以要(+1)%4处理
            dir = (dir + 1) % 4;
            Image_iptsRight[step][0] = x;
            Image_iptsRight[step][1] = y;
            ++step;
            turn = 0;
        }
    }
    Image_iptsRightNum = step;
}
static void Image_FindLine_RightHand_Adaptive_inv(uint8* image, int block_size, int clip_value, int x, int y) {
    //传入的块的长度必须为奇数!
    uint8 half = block_size / 2;
    uint16 step = 0;
    uint8 dir = 0, turn = 0;
    while (step < Image_rptsRightNum && half < x && x < IMAGE_WIDTH - half - 1&& half < y && y < IMAGE_HEIGHT - half - 1 && turn < 4) {
        //使用自适应阈值法求解阈值
        //clip_value为经验常数,一般取2~5
        int16 localThres = 0;
        for (int16 dy = -half; dy <= half; ++dy) {
            for (int16 dx = -half; dx <= half; ++dx) {
                localThres += IMAGE_AT(image, x + dx, y + dy);
            }
        }
        localThres /= block_size * block_size;
        localThres -= clip_value;
        localThressss=localThres;
        //做阈值判断
        uint8 frontValue = IMAGE_AT(image, x + Image_dir_front[dir][0], y + Image_dir_front[dir][1]);
        uint8 frontRightValue = IMAGE_AT(image, x + Image_dir_frontRight[dir][0], y + Image_dir_frontRight[dir][1]);

        if (frontValue < localThres) {
            //前方是黑色"墙壁" -- 向左转
            //方向改变,记录转向一次
            //向左转是 顺时针 - 所以要(+3)%4处理(或者说(-1)%4处理)
            dir = (dir + 3) % 4;
            ++turn;
        }
        else if (frontRightValue < localThres){
            //前方是白色,同时是右前方是"黑色" - 这种情况就是直道走
            x += Image_dir_front[dir][0];
            y += Image_dir_front[dir][1];
            Image_rptsRight[step][0] = x;
            Image_rptsRight[step][1] = y;
            ++step;
            turn = 0;
        }
        else {
            //前方是白色,但是右前方也是"白色" - 这种情况下说明需要向右转弯"贴墙走"
            x += Image_dir_frontRight[dir][0];
            y += Image_dir_frontRight[dir][1];
            //向右转是 顺时针 - 所以要(+1)%4处理
            dir = (dir + 1) % 4;
            Image_rptsRight[step][0] = x;
            Image_rptsRight[step][1] = y;
            ++step;
            turn = 0;
        }
    }
    Image_rptsRightNum = step;
}

/*
 * @brief               三角点集滤波
 * @parameter pts_in    需要滤波的曲线数组
 * @parameter lineNum   曲线的长度
 * @parameter pts_out   滤波输出
 * @parameter kernel    "三角形底边长"
 * @example
 */
void Image_BlurPoints(uint8 pts_in[][2], uint8 lineNum, uint8 pts_out[][2], uint8 kernerl) {
    int8 half = kernerl / 2;
    uint16 tmpPtsOut[2];
    for (int i = 0; i < lineNum; ++i) {
        tmpPtsOut[0] = 0;
        tmpPtsOut[1] = 0;
        //后面几个点就直接不用了
        for (int j = -half; j < half; ++j) {
            tmpPtsOut[0] += pts_in[bf_clip(i + j, 0, lineNum -1)][0] * (half + 1 - abs(j));
            tmpPtsOut[1] += pts_in[bf_clip(i + j, 0, lineNum -1)][1] * (half + 1 - abs(j));
        }
        pts_out[i][0] = tmpPtsOut[0] / ((2 * half + 2) * (half + 1) / 2);
        pts_out[i][1] = tmpPtsOut[1] / ((2 * half + 2) * (half + 1) / 2);
    }
}

/*
 * @brief               点集等距采样(是走过的采样前折线段的距离为dist)
 * @parameter pts_in    需要处理的线
 * @parameter num1      待处理线的长度
 * @parameter pts_out   数据输出数组
 * @parameter num2      输出数组长度限幅
 * @parameter dist      采样之后点与点之间的距离(单位为m)
 * @example
 */

void Image_ResamplePoints(uint8 pts_in[][2], uint8 num1, uint8 pts_out[][2], uint8* num2, float dist) {
    float remain = 0;
    uint8 len = 0;
    for (uint8 i = 0; i < num1 - 1 && len < *num2; ++i) {
        uint8 x0 = pts_in[i][0];
        uint8 y0 = pts_in[i][1];
        float dx = (float)(pts_in[i + 1][0] - x0);
        float dy = (float)(pts_in[i + 1][1] - y0);
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;

        //进行等距采样
        while(remain < dn && len < *num2) {
            x0 += (uint8)(dx * remain);
            pts_out[len][0] = x0;
            y0 += (uint8)(dy * remain);
            pts_out[len][1] = y0;

            ++len;
            dn -= remain;
            remain = dist;
        }
        remain -= dn;
    }
    *num2 = len;
}



/*
 * @brief               求解点集局部角度变化率
 * @parameter pts_in    需要处理的线
 * @parameter num       线的长度
 * @parameter angle_out 角度输出值
 * @parameter dist      角度采样的距离(实际距离)
 * @formula             tan(x + y) = (tan(x) - tan(y)) / (1 + tan(x) * tan())
 */
void Image_LocalAnglePoints(uint8 pts_in[][2], uint8 num, float angle_out[], uint16 dist) {
    for (uint8 i = 0; i < num; ++i) {
        if (i <= 0 || i >= num - 1) {
            //线的边缘没有角度,进行处理设置为0
            angle_out[i] = 0;
            continue;
        }
        //用向量的方法计算
        //这个算的其实是对应的补角
        //因为如果直接计算点为两个向量的起点的话,那么直线算出来的是180°,实际上想要的是0°
        float dx1 = pts_in[i][0] - pts_in[bf_clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[bf_clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[bf_clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[bf_clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1) * 180 / Image_PI;
    }
}

/*
 * @brief               角度变化率非极大抑制(向两边搜索查看最值)
 * @parameter angle_in  所要处理的角度数组
 * @parameter num       线的长度
 * @parameter angle_out 角度输出(只保留区域化最值)
 * @parameter kernel    区块边长
 * @parameter cornerNum 角点个数
 * @example
 */
void Image_NmsAngle(float angle_in[], int lineNum, float angle_out[], int kernel, uint8* cornerNum) {
    int8 half = kernel / 2;
    *cornerNum = lineNum;
    for (uint8 i = 0; i < lineNum; ++i) {
        angle_out[i] = angle_in[i];
        for (int8 j = -half; j <= half; ++j) {
            if (fabs(angle_in[bf_clip(i + j, 0, lineNum - 1)]) > fabs(angle_out[i])) {  // 对于非极大值点,直接置0，在这个点的左half和右half这个区间内都不是极大值
                angle_out[i] = 0;
                --(*cornerNum);
                break;
            }
        }
    }
}

/*
 * @brief                   左边线跟踪中线
 * @parameter pts_in        需要处理的边线数据
 * @parameter lineNum       边线的长度
 * @parameter pts_out       中线的输出数组
 * @parameter approx_num    求斜率的时候中点距离上点(下点)的像素点个数
 * @parameter dist          赛道的宽度
 * @example
 */
void Image_TrackLeftLine(uint8 pts_in[][2], uint8 num, uint8 pts_out[][2], uint8 approx_num, float dist) {
    for (int i = 0; i < num; ++i) {
        float dx = (float)(pts_in[bf_clip(i + approx_num, 0, num - 1)][0] - pts_in[bf_clip(i - approx_num, 0, num-1)][0]);
        float dy = (float)(pts_in[bf_clip(i + approx_num, 0, num - 1)][1] - pts_in[bf_clip(i - approx_num, 0, num-1)][1]);
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        pts_out[i][0] = (uint8)((float)pts_in[i][0] - dy * dist);
        pts_out[i][1] = (uint8)((float)pts_in[i][1] + dx * dist);
    }
}

/*
 * @brief                   左边线跟踪中线
 * @parameter pts_in        需要处理的边线数据
 * @parameter lineNum       边线的长度
 * @parameter pts_out       中线的输出数组
 * @parameter approx_num    求斜率的时候中点距离上点(下点)的像素点个数
 * @parameter dist          赛道的宽度
 * @example
 */
void Image_TrackRightLine(uint8 pts_in[][2], uint8 num, uint8 pts_out[][2], uint8 approx_num, float dist) {
    for (int i = 0; i < num; ++i) {
        float dx = (float)(pts_in[bf_clip(i + approx_num, 0, num - 1)][0] - pts_in[bf_clip(i - approx_num, 0, num-1)][0]);
        float dy = (float)(pts_in[bf_clip(i + approx_num, 0, num - 1)][1] - pts_in[bf_clip(i - approx_num, 0, num-1)][1]);
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        pts_out[i][0] = (uint8)((float)pts_in[i][0] + dy * dist);
        pts_out[i][1] = (uint8)((float)pts_in[i][1] - dx * dist);
    }
}

void Image_Handle_LeftLine(uint8 *image,uint8 x,uint8 y,uint8 thre) {
        //迷宫法找左边线,使用thre
    uint8 step=0;
    uint8 dir = 0, turn = 0;
    while(step<Image_iptsLeftNum&&turn<4&&x>0&&x<IMAGE_WIDTH-1&&y>0&&y<IMAGE_HEIGHT-1){
        uint8 frontValue = IMAGE_AT(image, x + Image_dir_front[dir][0], y + Image_dir_front[dir][1]);
        uint8 frontLeftValue = IMAGE_AT(image, x + Image_dir_frontLeft[dir][0], y + Image_dir_frontLeft[dir][1]);
        if (frontValue < thre) {
            //前方是黑色"墙壁" -- 向右转
            //方向改变,记录转向一次
            //向右转是 顺时针 - 所以要(+1)%4处理
            dir = (dir + 1) % 4;
            ++turn;
        }
        else if (frontLeftValue < thre){
            //前方是白色,但是左前方是"黑色" - 这种情况就是直道走
            x += Image_dir_front[dir][0];
            y += Image_dir_front[dir][1];
            Image_iptsLeft[step][0] = x;
            Image_iptsLeft[step][1] = y;
            turn = 0;
            ++step;
        }
        else {
            //前方是白色,同时左前方也是"白色" - 这种情况下说明需要向左转弯"贴墙走"
            x += Image_dir_frontLeft[dir][0];
            y += Image_dir_frontLeft[dir][1];
            //向左转是 逆时针 - 所以要(+3)%4处理 (或者说(-1)%4处理)
            dir = (dir + 3) % 4;
            Image_iptsLeft[step][0] = x;
            Image_iptsLeft[step][1] = y;
            ++step;
            turn = 0;
        }
    }
    Image_iptsLeftNum=step;
}

void Image_Handle_RightLine(uint8 *image,uint8 x,uint8 y,uint8 thre){
    uint8 step=0;
    uint8 dir = 0, turn = 0;
    while(step<Image_iptsRightNum&&turn<4&&x>0&&x<IMAGE_WIDTH-3&&y>0&&y<IMAGE_HEIGHT-1){
        uint8 frontValue = IMAGE_AT(image, x + Image_dir_front[dir][0], y + Image_dir_front[dir][1]);
        uint8 frontRightValue = IMAGE_AT(image, x + Image_dir_frontRight[dir][0], y + Image_dir_frontRight[dir][1]);
        if (frontValue < thre) {
            //前方是黑色"墙壁" -- 向左转
            //方向改变,记录转向一次
            //向左转是 顺时针 - 所以要(+3)%4处理(或者说(-1)%4处理)
            dir = (dir + 3) % 4;
            ++turn;
        }
        else if (frontRightValue < thre){
            //前方是白色,同时是右前方是"黑色" - 这种情况就是直道走
            x += Image_dir_front[dir][0];
            y += Image_dir_front[dir][1];
            Image_iptsRight[step][0] = x;
            Image_iptsRight[step][1] = y;
            ++step;
            turn = 0;
        }
        else {
            //前方是白色,但是右前方也是"白色" - 这种情况下说明需要向右转弯"贴墙走"
            x += Image_dir_frontRight[dir][0];
            y += Image_dir_frontRight[dir][1];
            //向右转是 顺时针 - 所以要(+1)%4处理
            dir = (dir + 1) % 4;
            Image_iptsRight[step][0] = x;
            Image_iptsRight[step][1] = y;
            ++step;
            turn = 0;
        }
    }
    Image_iptsRightNum=step;
}
void Image_getcenterLine(){
        //Image_centerLine[IMAGE_LINE_MAX_NUM][2]
    if (Image_isUsefulData_Status) {
        Image_centerLineNum_Bak =Image_centerLineNum;
        //对数据进行备份
        for (uint8 i = 0; i < Image_centerLineNum; ++i) {
            Image_centerLine_Bak[i][0] = Image_centerLine[i][0];
            Image_centerLine_Bak[i][1] = Image_centerLine[i][1];
        }
    }
    Image_centerLineNum=sizeof(Image_centerLine)/sizeof(Image_centerLine[0]);
    uint8 cx;
    uint8 cy;
    for(uint8 i;i<Image_centerLineNum;++i){
        cx=(Image_iptsLeft[i][0]+Image_iptsRight[i][0])/2;
        cy=(Image_iptsLeft[i][1]+Image_iptsRight[i][1])/2;
        Image_centerLine[i][0]=cx;
        Image_centerLine[i][1]=cy;
        if(i>Image_iptsLeftNum||i>Image_iptsRightNum){
            Image_centerLineNum=i;
            break;
        }
    }
    if (Image_isUsefulData_Status== 0) {
               Image_isUsefulData_Status = 1;
           }
}
void  Image_Process_inv(uint8* image_inv){
    image_thre=Image_Processing_OtsuGetThresh(image_inv);
    Image_Process_Status_inv= 0;
    if (Image_isUsefulData_Status) {
            Image_rptsLeftNum_Bak = Image_rptsLeftNum;
            //对数据进行备份
            for (uint8 i = 0; i < Image_rptsLeftNum; ++i) {
                Image_rptsLeft_Bak[i][0] = Image_rptsLeft[i][0];
                Image_rptsRight_Bak[i][1] = Image_rptsLeft[i][1];
            }
        }
    //-------------------状态机-------------------


    if(Trace_Status==TRACE_CROSS){
        if(Trace_traceType==TRACE_Camera_Far){image_begin_y=IMAGE_HEIGHT-70;}
        if(Trace_traceType==TRACE_Camera_Near){image_begin_y=IMAGE_HEIGHT-10;}
    }


    //-------------------状态机------------------
    //找左边线
       // image_thre=Image_Processing_OtsuGetThresh(image_inv);
       Image_rptsLeftNum = sizeof(Image_rptsLeft) / sizeof(Image_rptsLeft[0]);
       uint8 rx1 = image_begin_x;
       uint8 ry1 = image_begin_y-10;
       for (; rx1 > 0; --rx1) if (IMAGE_AT(image_inv, rx1 - 1, ry1) < image_thre) break;           //查找边界上的第一个点
       if (IMAGE_AT(image_inv, rx1, ry1) >= image_thre){//没有到边界就正常处理
           Image_FindLine_LeftHand_Adaptive_inv(image_inv,image_block_size,image_block_clip_value, rx1,ry1);
       }
       else{
           Image_rptsLeftNum = 0;                                                          //边界的话就是0了
       }
       //----------------------------------------
       //找右边线
       Image_rptsRightNum = sizeof(Image_rptsRight) / sizeof(Image_rptsRight[0]);
       uint8 rx2 = image_begin_x;
       uint8 ry2 = image_begin_y-10;
       for (; rx2 < IMAGE_WIDTH - 1; ++rx2) if (IMAGE_AT(image_inv, rx2 + 1, ry2) < image_thre) break;     //查找边界上的第一个点
       if (IMAGE_AT(image_inv, rx2, ry2) >= image_thre)                                                  //没有到边界就正常处理
           Image_FindLine_RightHand_Adaptive_inv(image_inv,image_block_size,image_block_clip_value, rx2,ry2);
       else{
           Image_rptsRightNum = 0;
       }
       for (uint8 i = 1; i < Image_rptsLeftNum - 1; ++i) {
               if (Image_rptsLeft[i + 1][0] != 0 || Image_rptsLeft[i + 1][1] != 0) {
                   Image_rptsLeft[i][0] = Image_rptsLeft[i + 1][0];
                   Image_rptsLeft[i][1] = Image_rptsLeft[i + 1][1];
                   ++Image_rptsLeftrNum;
               }
           }
           Image_rptsLeftNum = Image_rptsLeftrNum;
           Image_rptsLeftrNum = 0;
           for (uint8 i = 1; i < Image_rptsRightNum - 1; ++i) {
               if (Image_rptsRight[i + 1][0] != 0 || Image_rptsRight[i + 1][1] != 0) {
                   Image_rptsRight[i][0] = Image_rptsRight[i + 1][0];
                   Image_rptsRight[i][1] = Image_rptsRight[i + 1][1];
                   ++Image_rptsRightrNum;
               }
           }
           Image_rptsRightNum = Image_rptsRightrNum;
           Image_rptsRightrNum = 0;


           //----------------------------------------
           //对数据进行三角滤波
           //左边线
           Image_BlurPoints(Image_rptsLeft, Image_rptsLeftNum, Image_rptsLeftb, Image_linerBlurKernel);
           Image_rptsLeftbNum = Image_rptsLeftNum;
           //右边线
           Image_BlurPoints(Image_rptsRight, Image_rptsRightNum, Image_rptsRightb, Image_linerBlurKernel);
           Image_rptsRightbNum = Image_rptsRightNum;

           //----------------------------------------
           //对滤波后数据进行等距采样
           //----------------------------------------
           //对数据进行备份
           if (Image_isUsefulData_Status) {
               Image_rptsLeftsNum_Bak = Image_rptsLeftsNum;
               Image_rptsRightsNum_Bak = Image_rptsRightsNum;
               for (uint8 i = 0; i < Image_rptsLeftsNum; ++i) {
                   Image_rptsLefts_Bak[i][0] = Image_rptsLefts[i][0];
                   Image_rptsLefts_Bak[i][1] = Image_rptsLefts[i][1];
               }
               for (uint8 i = 0; i < Image_rptsRightsNum; ++i) {
                   Image_rptsRights_Bak[i][0] = Image_rptsRights[i][0];
                   Image_rptsRights_Bak[i][1] = Image_rptsRights[i][1];
               }
           }
           //----------------------------------------
           //等距采样
           Image_rptsLeftsNum = sizeof(Image_rptsLefts) / sizeof(Image_rptsLefts[0]);
           Image_ResamplePoints(Image_rptsLeftb, Image_rptsLeftbNum, Image_rptsLefts, &Image_rptsLeftsNum, Image_sampleDist * Image_pixelPreMeter);
           Image_rptsRightsNum = sizeof(Image_rptsRights) / sizeof(Image_rptsRights[0]);
           Image_ResamplePoints(Image_rptsRightb, Image_rptsRightbNum, Image_rptsRights, &Image_rptsRightsNum, Image_sampleDist * Image_pixelPreMeter);

           //----------------------------------------


           //求解边线局部角度变化率
           Image_LocalAnglePoints(Image_rptsLefts, Image_rptsLeftsNum, Image_rptsLefta, (uint16)round(Image_angleDist / Image_sampleDist));
           Image_rptsLeftaNum = Image_rptsLeftsNum;
           Image_LocalAnglePoints(Image_rptsRights, Image_rptsRightsNum, Image_rptsRighta, (uint16)round(Image_angleDist / Image_sampleDist));
           Image_rptsRightaNum = Image_rptsRightsNum;

           //----------------------------------------
           //对角度变化率进行非极大抑制(只保留一段边线中数据最大的点)
           Image_NmsAngle(Image_rptsLefta, Image_rptsLeftaNum, Image_rptsLeftan, (uint16)round(Image_angleDist / Image_sampleDist) * 2 + 1, &Image_cornerNumLeft);
           Image_rptsLeftanNum = Image_rptsLeftaNum;
           Image_NmsAngle(Image_rptsRighta, Image_rptsRightaNum, Image_rptsRightan, (uint16)round(Image_angleDist / Image_sampleDist) * 2 + 1, &Image_cornerNumRight);
           Image_rptsRightanNum = Image_rptsRightaNum;

           if (Image_isUsefulData_Status) {
               Image_rptsLeftcNum_Bak = Image_rptsLeftcNum;
               for (uint8 i = 0; i < Image_rptsLeftcNum; ++i) {
                   Image_rptsLeftc_Bak[i][0] = Image_rptsLeftc[i][0];
                   Image_rptsLeftc_Bak[i][1] = Image_rptsLeftc[i][1];
               }

               Image_rptsRightcNum_Bak = Image_rptsRightcNum;
               for (uint8 i = 0; i < Image_rptsRightcNum; ++i) {
                   Image_rptsRightc_Bak[i][0] = Image_rptsRightc[i][0];
                   Image_rptsRightc_Bak[i][1] = Image_rptsRightc[i][1];
               }
           }
           //----------------------------------------
           //找左右中线
           Image_TrackLeftLine(Image_rptsLefts, Image_rptsLeftsNum, Image_rptsLeftc, (uint8)round(Image_angleDist / Image_sampleDist), Image_pixelPreMeter * Image_roadWidth / 2-clip_middle);
           Image_rptsLeftcNum = Image_rptsLeftsNum;
           Image_TrackRightLine(Image_rptsRights, Image_rptsRightsNum, Image_rptsRightc, (uint8)round(Image_angleDist / Image_sampleDist), Image_pixelPreMeter * Image_roadWidth / 2+clip_middle);
           Image_rptsRightcNum = Image_rptsRightsNum;
       if (Image_isUsefulData_Status== 0) {
           Image_isUsefulData_Status = 1;
       }
       Image_Process_Status_inv = 1;
}


/*
 * @brief               图像处理,包括边缘提取,去畸变+逆透视,滤波,等距采样,计算角度,计算中线
 * @parameter image     需要处理的图片
 * @example             Image_Process(mt9v03x_image[0]);
 */
void Image_Process(uint8* image) {
    Image_Process_Status = 0;
    //----------------------------------------
    //原图找左右边线
    //----------------------------------------
    if (Image_isUsefulData_Status) {
        Image_iptsLeftNum_Bak = Image_iptsLeftNum;
        //对数据进行备份
        for (uint8 i = 0; i < Image_iptsLeftNum; ++i) {
            Image_iptsLeft_Bak[i][0] = Image_iptsLeft[i][0];
            Image_iptsLeft_Bak[i][1] = Image_iptsLeft[i][1];
        }
    }
    //-----------------------------------------


    if(Trace_Status==TRACE_CROSS){
        if(Trace_traceType==TRACE_Camera_Far){image_begin_y=IMAGE_HEIGHT-70;}
        if(Trace_traceType==TRACE_Camera_Near){image_begin_y=IMAGE_HEIGHT-10;}
    }


    //-----------------------------------------

    //找左边线
    Image_iptsLeftNum = sizeof(Image_iptsLeft) / sizeof(Image_iptsLeft[0]);
    uint8 x1 = image_begin_x;
    uint8 y1 = image_begin_y;
    for (; x1 > 0; --x1) if (IMAGE_AT(image, x1 - 1, y1) < image_thre) break;           //查找边界上的第一个点
    if (IMAGE_AT(image, x1, y1) >= image_thre){//没有到边界就正常处理
        Image_FindLine_LeftHand_Adaptive(image,image_block_size,image_block_clip_value, x1,y1);
    }
    else
        Image_iptsLeftNum = 0;                                                          //边界的话就是0了


    //----------------------------------------
    if (Image_isUsefulData_Status) {
        Image_iptsLeftNum_Bak = Image_iptsLeftNum;
        //对数据进行备份
        for (uint8 i = 0; i < Image_iptsRightNum; ++i) {
            Image_iptsLeft_Bak[i][0] = Image_iptsLeft[i][0];
            Image_iptsLeft_Bak[i][1] = Image_iptsLeft[i][1];
        }
    }
    //找右边线
    Image_iptsRightNum = sizeof(Image_iptsRight) / sizeof(Image_iptsRight[0]);
    uint8 x2 = image_begin_x;
    uint8 y2 = image_begin_y;
    for (; x2 < IMAGE_WIDTH - 1; ++x2) if (IMAGE_AT(image, x2 + 1, y2) < image_thre) break;     //查找边界上的第一个点
    if (IMAGE_AT(image, x2, y2) >= image_thre)                                                  //没有到边界就正常处理
        Image_FindLine_RightHand_Adaptive(image,image_block_size,image_block_clip_value, x2,y2);
    else
        Image_iptsRightNum = 0;                                                                  //边界的话就是0了
    if (Image_isUsefulData_Status) {
            Image_iptsRightNum_Bak = Image_iptsRightNum;
            //对数据进行备份
            for (uint8 i = 0; i < Image_iptsRightNum; ++i) {
                Image_iptsRight_Bak[i][0] = Image_iptsRight[i][0];
                Image_iptsRight_Bak[i][1] = Image_iptsRight[i][1];
            }
        }//边界的话就是0了
   if(Trace_Status==TRACE_CENTERLINENEAR||Trace_Status==TRACE_CENTERLINEFAR&&pid_type==PID_ORIGIN){
    Image_getcenterLine();
   }
    // Image_Processing_OtsuGetThresh(image);
        //对数据进行三角滤波
//        //左边线
//        Image_BlurPoints(Image_iptsLeft, Image_iptsLeftNum, Image_iptsLeftb, Image_linerBlurKernel);
//        Image_iptsLeftbNum = Image_iptsLeftNum;
//        //右边线
//        Image_BlurPoints(Image_iptsRight, Image_iptsRightNum, Image_iptsRightb, Image_linerBlurKernel);
//        Image_iptsRightbNum = Image_iptsRightNum;
//
//            //----------------------------------------
//            //对滤波后数据进行等距采样
//            //----------------------------------------
//            //对数据进行备份
//            if (Image_isUsefulData_Status) {
//                Image_iptsLeftsNum_Bak = Image_iptsLeftsNum;
//                Image_iptsRightsNum_Bak = Image_iptsRightsNum;
//                for (uint8 i = 0; i < Image_iptsLeftsNum; ++i) {
//                    Image_iptsLefts_Bak[i][0] = Image_iptsLefts[i][0];
//                    Image_iptsLefts_Bak[i][1] = Image_iptsLefts[i][1];
//                }
//                for (uint8 i = 0; i < Image_iptsRightsNum; ++i) {
//                    Image_iptsRights_Bak[i][0] = Image_itsRights[i][0];
//                    Image_iptsRights_Bak[i][1] = Image_iptsRights[i][1];
//                }
//            }
//                //等距采样
//                Image_iptsLeftsNum = sizeof(Image_iptsLefts) / sizeof(Image_iptsLefts[0]);
//                Image_ResamplePoints(Image_iptsLeftb, Image_iptsLeftbNum, Image_iptsLefts, &Image_iptsLeftsNum, Image_sampleDist * Image_pixelPreMeter);
//                Image_iptsRightsNum = sizeof(Image_iptsRights) / sizeof(Image_iptsRights[0]);
//                Image_ResamplePoints(Image_iptsRightb, Image_iptsRightbNum, Image_iptsRights, &Image_iptsRightsNum, Image_sampleDist * Image_pixelPreMeter);
//
//
//                    //求解边线局部角度变化率
//                    Image_LocalAnglePoints(Image_iptsLefts, Image_iptsLeftsNum, Image_iptsLefta, (uint16)round(Image_angleDist / Image_sampleDist));
//                    Image_iptsLeftaNum = Image_iptsLeftsNum;
//                    Image_LocalAnglePoints(Image_iptsRights, Image_iptsRightsNum, Image_iptsRighta, (uint16)round(Image_angleDist / Image_sampleDist));
//                    Image_iptsRightaNum = Image_iptsRightsNum;
//
//                    //----------------------------------------
//                    //对角度变化率进行非极大抑制(只保留一段边线中数据最大的点)
//                    Image_NmsAngle(Image_iptsLefta, Image_iptsLeftaNum, Image_iptsLeftan, (uint16)round(Image_angleDist / Image_sampleDist) * 2 + 1, &Image_cornerNumLeft);
//                    Image_iptsLeftanNum = Image_iptsLeftaNum;
//                    Image_NmsAngle(Image_iptsRighta, Image_iptsRightaNum, Image_iptsRightan, (uint16)round(Image_angleDist / Image_sampleDist) * 2 + 1, &Image_cornerNumRight);
//                    Image_iptsRightanNum = Image_iptsRightaNum;
//
//                    if (Image_isUsefulData_Status) {
//                        Image_iptsLeftcNum_Bak = Image_iptsLeftcNum;
//                        for (uint8 i = 0; i < Image_iptsLeftcNum; ++i) {
//                            Image_iptsLeftc_Bak[i][0] = Image_iptsLeftc[i][0];
//                            Image_iptsLeftc_Bak[i][1] = Image_iptsLeftc[i][1];
//                        }
//
//                        Image_iptsRightcNum_Bak = Image_iptsRightcNum;
//                        for (uint8 i = 0; i < Image_iptsRightcNum; ++i) {
//                            Image_iptsRightc_Bak[i][0] = Image_iptsRightc[i][0];
//                            Image_iptsRightc_Bak[i][1] = Image_iptsRightc[i][1];
//                        }
//                    }
//                    //中线计算
//                    Image_getcenterLine();
//

//逆透视方式寻找边线，现在暂时通过原图
    //----------------------------------------
    //对边线进行去畸变 + 逆透视变换
//    for (uint8 i = 0; i < Image_iptsLeftNum; ++i) {
//        Image_rptsLeft[i][0] = inv_map_x[Image_iptsLeft[i][1]][Image_iptsLeft[i][0]];
//        Image_rptsLeft[i][1] = inv_map_y[Image_iptsLeft[i][1]][Image_iptsLeft[i][0]];
//    }
//   Image_rptsLeftNum = Image_iptsLeftNum;
//
//    for (uint8 i = 0; i < Image_iptsRightNum; ++i) {
//        Image_rptsRight[i][0] = inv_map_x[Image_iptsRight[i][1]][Image_iptsRight[i][0]];
//        Image_rptsRight[i][1] = inv_map_y[Image_iptsRight[i][1]][Image_iptsRight[i][0]];
//    }
//    Image_rptsRightNum = Image_iptsRightNum;


    //去除(0,0)点
//    for (uint8 i = 1; i < Image_rptsLeftNum - 1; ++i) {
//        if (Image_rptsLeft[i + 1][0] != 0 || Image_rptsLeft[i + 1][1] != 0) {
//            Image_rptsLeft[i][0] = Image_rptsLeft[i + 1][0];
//            Image_rptsLeft[i][1] = Image_rptsLeft[i + 1][1];
//            ++Image_rptsLeftrNum;
//        }
//    }
//    Image_rptsLeftNum = Image_rptsLeftrNum;
//    Image_rptsLeftrNum = 0;
//    for (uint8 i = 1; i < Image_rptsRightNum - 1; ++i) {
//        if (Image_rptsRight[i + 1][0] != 0 || Image_rptsRight[i + 1][1] != 0) {
//            Image_rptsRight[i][0] = Image_rptsRight[i + 1][0];
//            Image_rptsRight[i][1] = Image_rptsRight[i + 1][1];
//            ++Image_rptsRightrNum;
//        }
//    }
//    Image_rptsRightNum = Image_rptsRightrNum;
//    Image_rptsRightrNum = 0;
//
//
//    //----------------------------------------
//    //对数据进行三角滤波
//    //左边线
//    Image_BlurPoints(Image_rptsLeft, Image_rptsLeftNum, Image_rptsLeftb, Image_linerBlurKernel);
//    Image_rptsLeftbNum = Image_rptsLeftNum;
//    //右边线
//    Image_BlurPoints(Image_rptsRight, Image_rptsRightNum, Image_rptsRightb, Image_linerBlurKernel);
//    Image_rptsRightbNum = Image_rptsRightNum;
//
//    //----------------------------------------
//    //对滤波后数据进行等距采样
//    //----------------------------------------
//    //对数据进行备份
//    if (Image_isUsefulData_Status) {
//        Image_rptsLeftsNum_Bak = Image_rptsLeftsNum;
//        Image_rptsRightsNum_Bak = Image_rptsRightsNum;
//        for (uint8 i = 0; i < Image_rptsLeftsNum; ++i) {
//            Image_rptsLefts_Bak[i][0] = Image_rptsLefts[i][0];
//            Image_rptsLefts_Bak[i][1] = Image_rptsLefts[i][1];
//        }
//        for (uint8 i = 0; i < Image_rptsRightsNum; ++i) {
//            Image_rptsRights_Bak[i][0] = Image_rptsRights[i][0];
//            Image_rptsRights_Bak[i][1] = Image_rptsRights[i][1];
//        }
//    }
//    //----------------------------------------
//    //等距采样
//    Image_rptsLeftsNum = sizeof(Image_rptsLefts) / sizeof(Image_rptsLefts[0]);
//    Image_ResamplePoints(Image_rptsLeftb, Image_rptsLeftbNum, Image_rptsLefts, &Image_rptsLeftsNum, Image_sampleDist * Image_pixelPreMeter);
//    Image_rptsRightsNum = sizeof(Image_rptsRights) / sizeof(Image_rptsRights[0]);
//    Image_ResamplePoints(Image_rptsRightb, Image_rptsRightbNum, Image_rptsRights, &Image_rptsRightsNum, Image_sampleDist * Image_pixelPreMeter);
//
//    //----------------------------------------
//
//
//    //求解边线局部角度变化率
//    Image_LocalAnglePoints(Image_rptsLefts, Image_rptsLeftsNum, Image_rptsLefta, (uint16)round(Image_angleDist / Image_sampleDist));
//    Image_rptsLeftaNum = Image_rptsLeftsNum;
//    Image_LocalAnglePoints(Image_rptsRights, Image_rptsRightsNum, Image_rptsRighta, (uint16)round(Image_angleDist / Image_sampleDist));
//    Image_rptsRightaNum = Image_rptsRightsNum;
//
//    //----------------------------------------
//    //对角度变化率进行非极大抑制(只保留一段边线中数据最大的点)
//    Image_NmsAngle(Image_rptsLefta, Image_rptsLeftaNum, Image_rptsLeftan, (uint16)round(Image_angleDist / Image_sampleDist) * 2 + 1, &Image_cornerNumLeft);
//    Image_rptsLeftanNum = Image_rptsLeftaNum;
//    Image_NmsAngle(Image_rptsRighta, Image_rptsRightaNum, Image_rptsRightan, (uint16)round(Image_angleDist / Image_sampleDist) * 2 + 1, &Image_cornerNumRight);
//    Image_rptsRightanNum = Image_rptsRightaNum;
//
//    if (Image_isUsefulData_Status) {
//        Image_rptsLeftcNum_Bak = Image_rptsLeftcNum;
//        for (uint8 i = 0; i < Image_rptsLeftcNum; ++i) {
//            Image_rptsLeftc_Bak[i][0] = Image_rptsLeftc[i][0];
//            Image_rptsLeftc_Bak[i][1] = Image_rptsLeftc[i][1];
//        }
//
//        Image_rptsRightcNum_Bak = Image_rptsRightcNum;
//        for (uint8 i = 0; i < Image_rptsRightcNum; ++i) {
//            Image_rptsRightc_Bak[i][0] = Image_rptsRightc[i][0];
//            Image_rptsRightc_Bak[i][1] = Image_rptsRightc[i][1];
//        }
//    }
//    //----------------------------------------
//    //找左右中线
//    Image_TrackLeftLine(Image_rptsLefts, Image_rptsLeftsNum, Image_rptsLeftc, (uint8)round(Image_angleDist / Image_sampleDist), Image_pixelPreMeter * Image_roadWidth / 2);
//    Image_rptsLeftcNum = Image_rptsLeftsNum;
//    Image_TrackRightLine(Image_rptsRights, Image_rptsRightsNum, Image_rptsRightc, (uint8)round(Image_angleDist / Image_sampleDist), Image_pixelPreMeter * Image_roadWidth / 2);
//    Image_rptsRightcNum = Image_rptsRightsNum;

    if (Image_isUsefulData_Status == 0) {
        Image_isUsefulData_Status = 1;
    }
    Image_Process_Status = 1;
}

/*
 * @brief               图像处理初始化 - 计算处理阈值
 * @attention           需要在摄像头初始化以后再调用该函数
 */
void Image_Init(void) {
    //系统延时一段时间,然后计算大津法
    system_delay_ms(100);
    while (1) {
        if (mt9v03x_finish_flag == 1) {                 //初始化完成
            ++Image_disPictureCnt;                      //丢弃图片计数
            mt9v03x_finish_flag = 0;
            if (Image_disPictureCnt > Image_disPictureCnt_Thre) {         //丢弃图片计数大于阈值
                Image_disPictureCnt = 0;
                break;
            }
        }
    }
    for (uint8 i = 0; i < Image_threCnt_Thre; ++i) {
        Image_threSum += Image_Processing_OtsuGetThresh(mt9v03x_image[0]);
    }
    image_thre = Image_threSum / Image_threCnt_Thre;
    system_delay_ms(10000);
}
//void Image_FindCorners(void){
//        Image_YptLeft_Found = false;
//        Image_YptRight_Found = false;
//        Image_LptLeft_Found = false;
//        Image_LptRight_Found = false;
//
//
//}
//void Image_FindCorners(void) {
//    // 识别 Y,L拐点
//    //把角点判断置false
//    Image_YptLeft_Found = false;
//    Image_YptRight_Found = false;
//    Image_LptLeft_Found = false;
//    Image_LptRight_Found = false;
//    //判断是否是直道
//    //判断是否超过50个像素点,如果没有超过,那就说明肯定不是直道了
//    Image_isStraightLeft = Image_rptsLeftsNum > 0.5 / Image_sampleDist;
//    Image_isStraightRight = Image_rptsRightsNum > 0.5 / Image_sampleDist;
//
//    //左边线判断 - 初始角点不参与判断
//    for (uint8 i = 5; i < Image_rptsLeftanNum; ++i) {
//        if (Image_rptsLeftan[i] == 0)
//            continue;
//        //当检测到有角度的点然后去判断
//        uint8 im1 = bf_clip(i - (uint8)round(Image_angleDist / Image_sampleDist), 0, Image_rptsLeftsNum - 1);
//        uint8 ip1 = bf_clip(i + (uint8)round(Image_angleDist / Image_sampleDist), 0, Image_rptsLeftsNum - 1);
//        float conf = fabs(Image_rptsLefta[i]) - fabs(Image_rptsLefta[im1] + Image_rptsLefta[ip1]) / 2;
//
//        //Y角点判断
//        if (Image_YptLeft_Found == false && 30 < conf && 65 > conf && i < 0.4 / Image_sampleDist) {
//            Image_YptLeft_rptsLefts_id = i;
//            Image_YptLeft_Found = true;
//        }
//        //L角点判断
//        if (Image_LptLeft_Found == false && 80 < conf && 130 > conf && i < 0.4 / Image_sampleDist) {
//            Image_LptLeft_rptsLefts_id = i;
//            Image_LptLeft_Found = true;
//        }
//        //长直道判断
//        if (conf > 15.0 && i < 0.5 / Image_sampleDist) {
//            put_float(200, conf);
//            put_int32(201, i);
//            Image_isStraightLeft = false;
//        }
//
//        //找到一组后,退出
//        if (Image_isStraightLeft == false && Image_LptLeft_Found == true && Image_YptLeft_Found == true)
//            break;
//    }
//
//    //右边线判断 - 初始角点不参与判断
//    for (uint8 i = 5; i < Image_rptsRightsNum; ++i) {
//        if (Image_rptsRightan[i] == 0.0)
//            continue;
//        //当检测到有角度的点然后去判断
//        uint8 im1 = bf_clip(i - (uint8)round(Image_angleDist / Image_sampleDist), 0, Image_rptsRightsNum - 1);
//        uint8 ip1 = bf_clip(i + (uint8)round(Image_angleDist / Image_sampleDist), 0, Image_rptsRightsNum - 1);
//        float conf = fabs(Image_rptsRighta[i]) - fabs(Image_rptsRighta[im1] + Image_rptsRighta[ip1]) / 2;
//        //Y角点判断
//        if (Image_YptRight_Found == false && 30 < conf && 65 > conf && i < 0.4 / Image_sampleDist) {
//            Image_YptRight_rptsRights_id = i;
//            Image_YptRight_Found = true;
//        }
//        //L角点判断
//        if (Image_LptRight_Found == false && 80 < conf && 130 > conf && i < 0.4 / Image_sampleDist) {
//            Image_LptRight_rptsRights_id = i;
//            Image_LptRight_Found = true;
//        }
//        //长直道判断
//        if (conf > 15.0 && i < 0.5 / Image_sampleDist) {
//            put_float(202, conf);
//            put_int32(203, i);
//            Image_isStraightRight = false;
//        }
//
//        //找到一组后,退出
//        if (Image_isStraightRight == false && Image_LptRight_Found == true && Image_YptRight_Found == true)
//            break;
//    }
//
//    //时间紧迫,先搁置在这
//    //Y角点二次检查,依据两角点距离及角点张开特性 (理论上不用做Y角点的 - 针对第十八届比赛赛道)
//  //  if (Image_YptLeft_Found && Image_YptRight_Found) {
//
//  //  }
//    //L角点二次检查 - 依据L角点距离及角点张开特性 (理论上,依据电磁寻迹,这个两个L角点同时出现的情况也不用做处理)        根据摄像头循迹，这个角点需要进行判断
//  //  if (Image_LptLeft_Found && Image_LptRight_Found) {
//  //  }
//}

/*
 * @brief               大津法 - 通过类间方差,计算图像进行二值化的阈值(计算的时候使用的平均数,不是加权平均数)
 * @parameter image     图像指针
 * @parameter imgHeight 图像的高度
 * @parameter imgWidth  图像的宽度
 * @return              处理后得到的阈值
 * @example             Image_threshold = Image_Processing_OtsuGetThresh(mt9v03x_image[0]);
 */
uint8 Image_Processing_OtsuGetThresh(const uint8* image) {
    gpio_set_level(P20_9, 0);
    //返回出去的阈值
    uint8 threshold;

    //8bit灰度值
    uint16 grayHistogram[256];

    //最小灰度值
    uint8 minGrayValue;

    //最大灰度值
    uint8 maxGrayValue;

    //得到像素的总数
    uint16 numOfPix = IMAGE_HEIGHT * IMAGE_WIDTH;

    //灰度值总值
    uint32 imgTotalGrayValue = 0;

    //前景像素点数
    uint16 pixelOfFore = 0;
    //背景像素点数
    uint16 pixelOfBack = 0;

    //前景元素的占比
    float omegaFore = 0.0;
    //背景元素的占比
    float omegaBack = 0.0;

    //前景元素灰度值总值
    uint32 imgTotalGrayValueOfFore = 0;
    //背景元素灰度值总值
    uint32 imgTotalGrayValueOfBack = 0;

    //前景元素灰度平均数
    float miuFore = 0.0;
    //背景元素灰度平均数
    float miuBack = 0.0;

    //临时的方差
    float sigmaTmp = 0.0;

    //保存了最大方差数据的方差
    float sigma = 0.0;

    //初始化灰度直方图
    for (int i = 0; i < 256; ++i)
        grayHistogram[i] = 0;


    uint8 index = 0;

    //梳理每个灰度级别中每个像素在整幅图中的个数
    for (uint8 i = 0; i < IMAGE_HEIGHT; ++i) {
        for (uint8 j = 0; j < IMAGE_WIDTH; ++j) {
            index = *(image + (i * IMAGE_WIDTH + j));
            ++grayHistogram[index];
        }
    }
    //获取最小的灰度值
    for (minGrayValue = 0; minGrayValue < 256 && grayHistogram[minGrayValue] == 0; ++minGrayValue);

    //获取最大的灰度值
    for (maxGrayValue = 255; maxGrayValue > minGrayValue && grayHistogram[maxGrayValue] == 0; --maxGrayValue);

    if (minGrayValue == maxGrayValue)       //只有一个颜色
        return minGrayValue;

    if (minGrayValue + 1 == maxGrayValue)   //只有两个颜色
        return minGrayValue;

    //像素总数 和 灰度值总数
    for (uint8 i = minGrayValue; i < maxGrayValue; ++i) {
        imgTotalGrayValue += grayHistogram[i] * i;
    }


    //进行二值化计算处理
    for (uint8 i = minGrayValue; i < maxGrayValue; ++i) {
        pixelOfFore += grayHistogram[i];                    //前景像素个数
        pixelOfBack = numOfPix - pixelOfFore;               //背景像素个数

        omegaFore = (float)pixelOfFore / (float)numOfPix;   //前景像素百分比
        omegaBack = 1 - omegaFore;                          //背景像素百分比

        imgTotalGrayValueOfFore += grayHistogram[i] * i;                            //前景像素灰度值
        imgTotalGrayValueOfBack = imgTotalGrayValue - imgTotalGrayValueOfFore;      //背景像素灰度值

        //前景灰度平均数
        miuFore = (float)imgTotalGrayValueOfFore / (float)pixelOfFore;              //前景像素灰度平均数
        miuBack = (float)imgTotalGrayValueOfBack / (float)pixelOfBack;              //背景像素灰度平均数

        //计算类间方差
        sigmaTmp = omegaFore * omegaBack * (miuFore - miuBack) * (miuFore - miuBack);
        if (sigmaTmp > sigma) {
            sigma = sigmaTmp;
            threshold = i;
        }
    }
    //gpio_set_level(P20_9, 1);
    return threshold;
}

void Image_ShowCorner(uint8 x, uint8 y, rgb565_color_enum color) {
    ips200_draw_point(bf_clip(x, 0, ips200_width_max - 1), bf_clip(y, 0, ips200_height_max - 1), color);
    ips200_draw_point(bf_clip(x - 1, 0, ips200_width_max - 1), bf_clip(y, 0, ips200_height_max - 1), color);
    ips200_draw_point(bf_clip(x - 2, 0, ips200_width_max - 1), bf_clip(y, 0, ips200_height_max - 1), color);
    ips200_draw_point(bf_clip(x + 1, 0, ips200_width_max - 1), bf_clip(y, 0, ips200_height_max - 1), color);
    ips200_draw_point(bf_clip(x + 2, 0, ips200_width_max - 1), bf_clip(y, 0, ips200_height_max - 1), color);
    ips200_draw_point(bf_clip(x, 0, ips200_width_max - 1), bf_clip(y - 1, 0, ips200_height_max - 1), color);
    ips200_draw_point(bf_clip(x, 0, ips200_width_max - 1), bf_clip(y - 2, 0, ips200_height_max - 1), color);
    ips200_draw_point(bf_clip(x, 0, ips200_width_max - 1), bf_clip(y + 1, 0, ips200_height_max - 1), color);
    ips200_draw_point(bf_clip(x, 0, ips200_width_max - 1), bf_clip(y + 2, 0, ips200_height_max - 1), color);
}

void Image_GetAngleInit(void) {
    //只显示区域最大角,显示的度数
    gpio_init(IMAGE_ANGLE_CHANGE_UP_KEY, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(IMAGE_ANGLE_CHANGE_DOWN_KEY, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(IMAGE_ANGLE_LINE_SELCET_KEY, GPI, GPIO_HIGH, GPI_PULL_UP);
}


void Image_GetAngle(uint8 beg_x, uint8 beg_y, IMAGE_SCREEN screen) {
    //只有当图像处理完成的时候才能进行处理
    if (Image_Process_Status == 1) {
        //需要丢到循环里面,同时要和屏幕搭配使用
        //------------------------------
        //边线选择
        if (gpio_get_level(IMAGE_ANGLE_LINE_SELCET_KEY) == 0) {
            if (Image_lineSeclet == 0)
                Image_lineSeclet = 1;
            else
                Image_lineSeclet = 0;
        }

        //------------------------------
        //角点选择
        if (gpio_get_level(IMAGE_ANGLE_CHANGE_UP_KEY) == 0) {
            if (Image_lineSeclet == 0) {
                ++Image_angleCntLeft;
                if (Image_angleCntLeft >= Image_cornerNumLeft)
                    Image_angleCntLeft = Image_cornerNumLeft - 1;
            }
            else if (Image_lineSeclet == 1) {
                ++Image_angleCntRight;
                if (Image_angleCntRight >= Image_cornerNumRight)
                    Image_angleCntRight = Image_cornerNumRight - 1;
            }
        }
        else if (gpio_get_level(IMAGE_ANGLE_CHANGE_DOWN_KEY) == 0) {
            if (Image_lineSeclet == 0) {
                --Image_angleCntLeft;
                if (Image_angleCntLeft < 0)
                    Image_angleCntLeft = Image_cornerNumLeft - 1;
            }
            else if (Image_lineSeclet == 1) {
                --Image_angleCntRight;
                if (Image_angleCntRight < 0)
                    Image_angleCntRight = Image_cornerNumRight - 1;
            }
        }

        //------------------------------
        //确定显示内容
        uint8 cornerCnt = 0;
        uint8 cornerIndex = -1;
        if (Image_lineSeclet == 0) {
            for (uint8 i = 0; i < Image_rptsLeftanNum; ++i) {
                if (Image_rptsLeftan[i] != 0.0) {
                    ++cornerCnt;
                    if (Image_angleCntLeft == cornerCnt) {
                        cornerIndex = i;
                        break;
                    }
                }
            }
        }
        else if (Image_lineSeclet == 1) {
            for (uint8 i = 0; i < Image_rptsRightanNum; ++i) {
                if (Image_rptsRightan[i] != 0.0) {
                    ++cornerCnt;
                    if (Image_angleCntRight == cornerCnt) {
                        cornerIndex = i;
                        break;
                    }
                }
            }

        }

        //------------------------------
        //屏幕显示
        //显示的话,显示一个Image_angleB的正方形,代表选中,同时在屏幕的右上角打印相关的数据
        if (Image_lineSeclet == 0) {
            if (screen == IMAGE_IPS200) {
                int8 half = Image_angleBlockLen / 2;
                for (int8 i = -half; i <= half; ++i) {
                    for (int8 j = -half; j <= half; ++j) {
                        ips200_draw_point(bf_clip(beg_x + Image_rptsLefts[cornerIndex][0] + i, 0, ips200_width_max - 1), bf_clip(beg_y + Image_rptsLefts[cornerIndex][1] + j, 0, ips200_height_max - 1), IMAGE_ANGLE_COLOR);
                    }
                }
//                ips200_show_float(ips200_width_max - 50, ips200_height_max - 20, Image_rptsLeftan[cornerIndex], 3, 4);
            }
            else if (screen == IMAGE_TFT180) {
                int8 half = Image_angleBlockLen / 2;
                for (int8 i = -half; i <= half; ++i) {
                    for (int8 j = -half; j <= half; ++j) {
                        tft180_draw_point(bf_clip(beg_x + Image_rptsLefts[cornerIndex][0] + i, 0, tft180_width_max - 1), bf_clip(beg_y + Image_rptsLefts[cornerIndex][1] + j, 0, tft180_height_max - 1), IMAGE_ANGLE_COLOR);
                    }
                }
                tft180_show_float(tft180_width_max - 50, tft180_height_max - 20, Image_rptsLeftan[cornerIndex], 3, 4);
            }
        }
        else if (Image_lineSeclet == 1) {
            if (screen == IMAGE_IPS200) {
                int8 half = Image_angleBlockLen / 2;
                for (int8 i = -half; i <= half; ++i) {
                    for (int8 j = -half; j <= half; ++j) {
                        ips200_draw_point(bf_clip(beg_x + Image_rptsLefts[cornerIndex][0] + i, 0, ips200_width_max - 1), bf_clip(beg_y + Image_rptsLefts[cornerIndex][1] + j, 0, ips200_height_max - 1), IMAGE_ANGLE_COLOR);
                    }
                }
//                ips200_show_float(ips200_width_max - 50, ips200_height_max - 20, Image_rptsLeftan[cornerIndex], 3, 4);
            }
            /*else if (screen == IMAGE_TFT180) {
                int8 half = Image_angleBlockLen / 2;
                for (int8 i = -half; i <= half; ++i) {
                    for (int8 j = -half; j <= half; ++j) {
                        tft180_draw_point(bf_clip(beg_x + Image_rptsLefts[cornerIndex][0] + i, 0, tft180_width_max - 1), bf_clip(beg_y + Image_rptsLefts[cornerIndex][1] + j, 0, tft180_height_max - 1), IMAGE_ANGLE_COLOR);
                    }
                }
                tft180_show_float(tft180_width_max - 50, tft180_height_max - 20, Image_rptsLeftan[cornerIndex], 3, 4);
            }*/
        }
    }
}

/*
 * @brief               判断边线是否闭合
 * @parameter slect     选择的边线： 0 - 左边线
 *                                 1 - 右边线
 * @return              true : 边线闭合
 *                      false : 边线不闭合
 */
bool Image_LineIsClosed(uint8 select) {
    if (select == 0) {
        if (Image_iptsLeftNum > 10) {
            uint8 begin_coor[2] = {Image_iptsLeft[0][0], Image_iptsLeft[0][1]};
            for (uint8 i = 1; i < Image_iptsLeftNum; ++i) {
                if (Image_iptsLeft[i][0] == begin_coor[0] && Image_iptsLeft[i][1] == begin_coor[1])
                    return true;
            }
        }
        return false;
    }
    else if (select == 1) {
        if (Image_iptsRightNum > 10) {
            uint8 begin_coor[2] = {Image_iptsRight[0][0], Image_iptsRight[0][1]};
            for (uint8 i = 1; i < Image_iptsRightNum; ++i) {
                if (Image_iptsRight[i][0] == begin_coor[0] && Image_iptsRight[i][1] == begin_coor[1])
                    return true;
            }
        }
        return false;

    }
    return false;
}
/*
 * @brief               当检测到L角点的时候进行判断处理
 * @return              NULL
 */
void Image_LCornerCheck(void) {
    if (Image_LCornerJude_Status == IMAGE_LCORNER_NONE) {
        //左角点检测
        if (Image_LptLeft_Found && !Image_LptRight_Found && Image_isStraightRight) {
            gpio_set_level(P20_8, GPIO_LOW);
            Image_LCornerJude_Status = IMAGE_LCORNER_BEGIN_LEFT;
            Encoder_Begin(ENCODER_MOTOR_2);
        }

        //右角点检测
        if (Image_LptRight_Found && !Image_LptLeft_Found && Image_isStraightLeft) {
            gpio_set_level(P20_9, GPIO_LOW);
            Image_LCornerJude_Status = IMAGE_LCORNER_BEGIN_RIGHT;
            Encoder_Begin(ENCODER_MOTOR_1);
        }

    }
    else if (Image_LCornerJude_Status == IMAGE_LCORNER_BEGIN_LEFT) {
        //车库判断
//        if (abs(Encoder_sum_Motor1) < Circle_encoderLeft_Thre) {
//            if (Image_LineIsClosed(0) == true && Image_LineIsClosed(1) == true) {
//                Image_LCornerJude_Status = IMAGE_LCORNER_IS_GRAGE_LEFT;
//                gpio_set_level(P20_9, GPIO_LOW);
//            }
//        }
        //环岛
        if (abs(Encoder_sum_Motor1) > Image_GrageJudge_Thre) {
            Image_LCornerJude_Status = IMAGE_LCORNER_IS_CIRCLE_LEFT;
        }
    }
    else if (Image_LCornerJude_Status == IMAGE_LCORNER_BEGIN_RIGHT) {
//        if (abs(Encoder_sum_Motor2) < Circle_encoderRight_Thre) {
////            if (Image_LineIsClosed(0) == true && Image_LineIsClosed(1) == true) {
////                Image_LCornerJude_Status = IMAGE_LCORNER_IS_GRAGE_RIGHT;
////                gpio_set_level(P20_8, GPIO_LOW);
////            }
//        }
        //环岛
        if (abs(Encoder_sum_Motor2) > Image_GrageJudge_Thre) {
            Image_LCornerJude_Status = IMAGE_LCORNER_IS_CIRCLE_RIGHT;
        }
    }
    //入库开始状态机
//    else if (Image_LCornerJude_Status == IMAGE_LCORNER_IS_GRAGE_LEFT) {
//        Image_LCornerJude_Status = IMAGE_LCORNER_NONE;
//        if (Grage_grageStatus == GRAGE_NONE) {
//            Grage_grageStatus = GRAGE_IN_BEGIN_LEFT;
//            Gyroscope_Begin(GYROSCOPE_GYRO_X);
//        }
//    }
//    //入库开始状态机
//    else if (Image_LCornerJude_Status == IMAGE_LCORNER_IS_GRAGE_RIGHT) {
//        Image_LCornerJude_Status = IMAGE_LCORNER_NONE;
//        if (Grage_grageStatus == GRAGE_NONE) {
//            Grage_grageStatus = GRAGE_IN_BEGIN_RIGHT;
//            Gyroscope_Begin(GYROSCOPE_GYRO_X);
//        }
//
//
//    }
    //环岛开始状态机
    else if (Image_LCornerJude_Status == IMAGE_LCORNER_IS_CIRCLE_LEFT) {
        Image_LCornerJude_Status = IMAGE_LCORNER_NONE;
        if (Circle_status == CIRCLE_NONE) {
            Circle_status = CIRCLE_LEFT_BEGIN;
            Trace_Status=TRACE_CIRCLE;
            //Beep_Tweet();
        }
    }
    //环岛开始状态机
    else if (Image_LCornerJude_Status == IMAGE_LCORNER_IS_CIRCLE_RIGHT) {
        Image_LCornerJude_Status = IMAGE_LCORNER_NONE;
        if (Circle_status == CIRCLE_NONE) {
            Circle_status = CIRCLE_RIGHT_BEGIN;
            Trace_Status=TRACE_CIRCLE;
            //Beep_Tweet();
        }

    }
}

