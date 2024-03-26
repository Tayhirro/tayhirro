/*
 * xiao_trace.c
 *
 *  Created on: 2023年7月2日
 *      Author: Jayden_NaN
 */
#include "xiao_trace.h"
#include "xiao_pid.h"
//------------------------------------------------------------
//基本变量
TRACE_TYPE Trace_traceType = TRACE_Camera_MID;
extern fPID Trace_cameraLeftPID;                       //左边线获取的中线的PID
extern fPID Trace_cameraRightPID;                      //右边线获取的中线的PID
extern fPID Trace_cameraMidPID;                       //左+右获取的中线的PID
//==============================摄像头寻迹相关==============================
//------------------------------------------------------------
//中线标准值相关
uint8 Trace_middleStandard = 94;                //摄像头锁定的中点所在的列(基本上就是在IMAGE_WIDTH / 2 中间左右浮动)
//------------------------------------------------------------
//中线巡线处理
float Trace_angleError = 0.0;                   //角度误差

float Trace_angleError_bak[counter_number];               //备份
float Trace_angleErrorTher = 7.0;               //角度误差阈值
uint8 Trace_aimLine = 26;//电机1600 前瞻22 电机1800 前瞻24                      //中线向上找的第n个点作为目标前瞻
int counter=0;

float Trace_lineWeight[] = {0.5, 0.3, 0.2};     //处理中线时候三行计算的权重
int flag=0;

TRACE_CIRCLE_TYPE Trace_Circle_Type=TRACE_CIRCLE_CAREMA_GYROSCOPE_ENCODER;      //初始化为共同
TRACE_CROSS_TYPE Trace_Cross_Type=TRACE_CROSS_CAREMA_GYROSCOPE_ENCODER;         //初始化为共同
TRACE_STATUS Trace_Status=TRACE_CENTERLINENEAR;
//------------------------------------------------------------
//PID相关
//fPID Trace_cameraLeftPID;                       //左边线获取的中线的PID
//fPID Trace_cameraRightPID;                      //右边线获取的中线的PID
//fPID Trace_cameraMidPID;                       //左+右获取的中线的PID

//==============================电磁寻迹相关==============================


//==============================基本函数==============================

/*
 * @brief               根据速度,动作和图像确定前瞻
 * @return              追踪目标中点所在的行
 * @attention           需要读取当前的速度(也就是说需要把速度读取丢中断里面一直跑)
 */
static void Trace_GetProspect(void) {
    if (Encoder_readFinishStatus == 1) {
        //通过速度,动态改变前瞻的长度
        if (Trace_angleError > Trace_angleErrorTher) {
            if (Encoder_1Data > 80 && Encoder_2Data > 80)
                Trace_aimLine += 2;
            else if (Encoder_1Data > 70 && Encoder_2Data > 70)
                Trace_aimLine += 1;
            else
                Trace_aimLine += 0;
        }
        else if (Trace_angleError < -Trace_angleErrorTher) {
            if (Encoder_1Data > 80 && Encoder_2Data > 80)
                Trace_aimLine += 2;
            else if (Encoder_1Data > 70 && Encoder_2Data > 70)
                Trace_aimLine += 1;
            else
                Trace_aimLine += 0;
        }
        else
            Trace_aimLine += 0;
    }
}

/*
 * @brief               通过三行加权重获取angleError
 * @attention           只处理了摄像头的,电磁的还没做处理
 */
static float Trace_GetAngelError() {
    //------------------------------
    //获取目标行
    Trace_GetProspect();

    //------------------------------
    //获取误差
    if(pid_type==PID_ORIGIN){
        if (Trace_Status == TRACE_CENTERLINENEAR) {
                  Trace_angleError = Trace_lineWeight[0] * (float)Image_centerLine[bf_clip(Trace_aimLine, 0, Image_centerLineNum - 1)][0]
                                  + Trace_lineWeight[1] * (float)Image_centerLine[bf_clip(Trace_aimLine + 1, 0, Image_centerLineNum - 1)][0]
                                  + Trace_lineWeight[2] * (float)Image_centerLine[bf_clip(Trace_aimLine + 2, 0, Image_centerLineNum - 1)][0];
                  //ips200_show_float(0,270, Trace_angleError, 3, 3);
                  //counter++;
                  if(Trace_angleError!=0 && flag==0){Trace_angleError_bak[counter]=Trace_angleError;counter++;}
                  if(counter==counter_number){flag=1;}
                  if(Trace_angleError!=0 && flag==1){
                      for(int i=0;i<counter_number-1;i++){
                          Trace_angleError_bak[i]=Trace_angleError_bak[i+1];

                      }
                      Trace_angleError_bak[counter-1]=Trace_angleError;
                  }
                  //if(Trace_angleError!=0&& flag==1){counter=counter%counter_number;Trace_angleError_bak[counter]=Trace_angleError;counter++;}
                  if(Trace_angleError==0){Trace_angleError=Trace_angleError_bak[0];}
                   return Trace_angleError;
                  /*Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
                                             + Trace_lineWeight[1] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
                                             + Trace_lineWeight[2] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];*/

              }
           else if (Trace_Status == TRACE_RIGHTLOST) {
               Trace_angleError = Trace_lineWeight[0] * (float)Image_centerLine[bf_clip(Trace_aimLine, 0, Image_centerLineNum - 1)][0]
                               + Trace_lineWeight[1] * (float)Image_centerLine[bf_clip(Trace_aimLine + 1, 0, Image_centerLineNum - 1)][0]
                               + Trace_lineWeight[2] * (float)Image_centerLine[bf_clip(Trace_aimLine + 2, 0, Image_centerLineNum - 1)][0];
               return Trace_angleError;
           }
           else if (Trace_Status == TRACE_LEFTLOST) {
               Trace_angleError = Trace_lineWeight[0] * (float)Image_centerLine[bf_clip(Trace_aimLine, 0, Image_centerLineNum - 1)][0]
                               + Trace_lineWeight[1] * (float)Image_centerLine[bf_clip(Trace_aimLine + 1, 0, Image_centerLineNum - 1)][0]
                               + Trace_lineWeight[2] * (float)Image_centerLine[bf_clip(Trace_aimLine + 2, 0, Image_centerLineNum - 1)][0];
               return Trace_angleError;
           }
    }

    if(pid_type==PID_INV){
        if(Trace_Status == TRACE_CENTERLINENEAR){
                  /*Trace_angleError = Trace_lineWeight[0] * (float)Image_centerLine[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
                                  + Trace_lineWeight[1] * (float)Image_centerLine[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
                                  + Trace_lineWeight[2] * (float)Image_centerLine[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];*/

            if (Shift_Direction == SHIFT_RIGHT) {
                Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
                                + Trace_lineWeight[1] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
                                + Trace_lineWeight[2] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];

            }
            else if (Shift_Direction == SHIFT_LEFT) {
                Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsRightc[bf_clip(Trace_aimLine, 0, Image_rptsRightcNum - 1)][0]
                                + Trace_lineWeight[1] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 1, 0, Image_rptsRightcNum - 1)][0]
                                + Trace_lineWeight[2] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 2, 0, Image_rptsRightcNum - 1)][0];

            }
            else{
                Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsRightc[bf_clip(Trace_aimLine, 0, Image_rptsRightcNum - 1)][0]
                                               + Trace_lineWeight[1] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 1, 0, Image_rptsRightcNum - 1)][0]
                                               + Trace_lineWeight[2] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 2, 0, Image_rptsRightcNum - 1)][0]
                                               + Trace_lineWeight[0] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
                                               + Trace_lineWeight[1] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
                                               + Trace_lineWeight[2] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];
                Trace_angleError/=2;
            }
            if(Trace_angleError!=0 && flag==0){Trace_angleError_bak[counter]=Trace_angleError;counter++;}
                              if(counter==counter_number){flag=1;}
                              if(Trace_angleError!=0 && flag==1){
                                  for(int i=0;i<counter_number-1;i++){
                                      Trace_angleError_bak[i]=Trace_angleError_bak[i+1];

                                  }
                                  Trace_angleError_bak[counter-1]=Trace_angleError;
                              }
                              //if(Trace_angleError!=0&& flag==1){counter=counter%counter_number;Trace_angleError_bak[counter]=Trace_angleError;counter++;}
                              if(Trace_angleError==0){Trace_angleError=Trace_angleError_bak[0];}
                               return Trace_angleError;
                  return Trace_angleError;
              }
        if (Trace_Status == TRACE_CROSS) {        //在搜索中线的时候
                         /*Trace_angleError = Trace_lineWeight[0] * (float)Image_centerLine[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
                                         + Trace_lineWeight[1] * (float)Image_centerLine[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
                                         + Trace_lineWeight[2] * (float)Image_centerLine[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];*/


                        if (Shift_Direction == SHIFT_RIGHT) {
                           Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
                                           + Trace_lineWeight[1] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
                                           + Trace_lineWeight[2] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];

                       }
                       else if (Shift_Direction == SHIFT_LEFT) {
                           Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsRightc[bf_clip(Trace_aimLine, 0, Image_rptsRightcNum - 1)][0]
                                           + Trace_lineWeight[1] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 1, 0, Image_rptsRightcNum - 1)][0]
                                           + Trace_lineWeight[2] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 2, 0, Image_rptsRightcNum - 1)][0];
                       }
                   if(Trace_angleError!=0 && flag==0){Trace_angleError_bak[counter]=Trace_angleError;counter++;}
                                     if(counter==counter_number){flag=1;}
                                     if(Trace_angleError!=0 && flag==1){
                                         for(int i=0;i<counter_number-1;i++){
                                             Trace_angleError_bak[i]=Trace_angleError_bak[i+1];

                                         }
                                         Trace_angleError_bak[counter-1]=Trace_angleError;
                                     }
                                     //if(Trace_angleError!=0&& flag==1){counter=counter%counter_number;Trace_angleError_bak[counter]=Trace_angleError;counter++;}
                                     if(Trace_angleError==0){Trace_angleError=Trace_angleError_bak[0];}
                                      return Trace_angleError;
                         return Trace_angleError;
                     }


    return 84;
    }
}

/*
 * @brief               PID初始化(所有数据都初始化为0)
 * @return              NULL
 */
void Trace_PIDInit() {
   // PID_Init(&Trace_cameraLeftPID);
   // PID_Init(&Trace_cameraRightPID);
}

/*
 * @brief               寻迹PID参数设置
 * @parameter K_p_set   PID的P参数
 * @parameter K_d_set   PID的D参数
 * @parameter coLimit   修正限幅
 * @parameter boost
 * @parameter traceType 寻迹类型
 * @attention           本函数只提供了摄像头PID的参数设置,电磁数据设置无效
 */
void Trace_PID_Set(float K_p_set, float K_d_set, float coLimit, float boost, TRACE_TYPE traceType) {
    //------------------------------
    //寻摄像头左边线+右边线找到的中线的PID
    if (traceType == TRACE_Camera_MID) {
            PID_SetParameter(&Trace_cameraMidPID, K_p_set, 0, K_d_set, 0, coLimit, boost);
        }
    //寻摄像头左边线找到的中线的PID

    else if (traceType == TRACE_Camera_LEFT) {

        PID_SetParameter(&Trace_cameraLeftPID, K_p_set, 0, K_d_set, 0, coLimit, boost);
    //寻摄像头右边线找到的中线的PID
        PID_SetParameter(&Trace_cameraRightPID, K_p_set, 0, K_d_set, 0, coLimit, boost);
    }

}


/*
 * @brief               进行寻迹
 * @return              处理后的误差ut
 */
float Trace_Run() {
    //----------------------------------------
    //电磁寻迹
    /*if (Trace_traceType == TRACE_Camera_MID) {
        return 0;
    }*/
    //----------------------------------------
    //摄像头左加右寻中线
    if (Trace_Status == TRACE_CENTERLINENEAR) {
           //Trace_GetAngelError();
        if(Shift_Direction==SHIFT_RIGHT){
            //首先降速
//            return Trace_cameraLeftPID.output_val;
////            //Trace_GetAngelError();
////
//            direction_control(&Trace_cameraLeftPID,Trace_angleError,94);
////
////
////            //Trace_PID_Set(Trace_cameraLeftPID.Kp_Set, Trace_cameraLeftPID.Kd_Set, Trace_cameraLeftPID.utLimit, 1.0, Trace_traceType);
////            //PID_PostionalPID(&Trace_cameraLeftPID, 0, Trace_angleError);
        }
        else if (Shift_Direction==SHIFT_LEFT) {
            //首先降速
////               //Trace_GetAngelError();
////
////             //  direction_control(&Trace_cameraRightPID,Trace_angleError,94);
////
////               //Trace_PID_Set(Trace_cameraRightPID.Kp_Set, Trace_cameraRightPID.Kd_Set, Trace_cameraRightPID.utLimit, 1.0, Trace_traceType);
////               //PID_PostionalPID(&Trace_cameraRightPID, 0, Trace_angleError);
//               return Trace_cameraRightPID.output_val;
           }
           //direction_control(&Trace_cameraMidPID,Trace_angleError,94);
        direction_control(&Trace_cameraMidPID,Trace_GetAngelError(),84);

           //Trace_PID_Set(Trace_cameraLeftPID.Kp_Set, Trace_cameraLeftPID.Kd_Set, Trace_cameraLeftPID.utLimit, 1.0, Trace_traceType);
           //PID_PostionalPID(&Trace_cameraLeftPID, 0, Trace_angleError);
           return Trace_cameraMidPID.output_val;
       }
    if (Trace_Status ==TRACE_CROSS) {
               //Trace_GetAngelError();
            if(Shift_Direction==SHIFT_RIGHT){
                //首先降速
    //            return Trace_cameraLeftPID.output_val;
    ////            //Trace_GetAngelError();
    ////
    //            direction_control(&Trace_cameraLeftPID,Trace_angleError,94);
    ////
    ////
    ////            //Trace_PID_Set(Trace_cameraLeftPID.Kp_Set, Trace_cameraLeftPID.Kd_Set, Trace_cameraLeftPID.utLimit, 1.0, Trace_traceType);
    ////            //PID_PostionalPID(&Trace_cameraLeftPID, 0, Trace_angleError);
            }
            else if (Shift_Direction==SHIFT_LEFT) {
                //首先降速
    ////               //Trace_GetAngelError();
    ////
    ////             //  direction_control(&Trace_cameraRightPID,Trace_angleError,94);
    ////
    ////               //Trace_PID_Set(Trace_cameraRightPID.Kp_Set, Trace_cameraRightPID.Kd_Set, Trace_cameraRightPID.utLimit, 1.0, Trace_traceType);
    ////               //PID_PostionalPID(&Trace_cameraRightPID, 0, Trace_angleError);
    //               return Trace_cameraRightPID.output_val;
               }
               //direction_control(&Trace_cameraMidPID,Trace_angleError,94);
            if(Cross_status!=CROSS_BEGIN){
            direction_control(&Trace_cameraMidPID,Trace_GetAngelError(),84);
            }
               //Trace_PID_Set(Trace_cameraLeftPID.Kp_Set, Trace_cameraLeftPID.Kd_Set, Trace_cameraLeftPID.utLimit, 1.0, Trace_traceType);
               //PID_PostionalPID(&Trace_cameraLeftPID, 0, Trace_angleError);
               return Trace_cameraMidPID.output_val;
           }
    else
        return -1;
}

//------------------------------------------------------------
//驱动vofa的设置函数
void Trace_SetPIDP(float setP, TRACE_TYPE traceType) {
    //------------------------------
    //设置寻左边线时候的P参数
    if (traceType == TRACE_Camera_LEFT) {
        Trace_cameraLeftPID.Kp = setP;
        Trace_cameraLeftPID.Kp_Set = setP;
    }
    //------------------------------
    //设置寻右边线时候的P参数
    else if (traceType == TRACE_Camera_RIGHT) {
        Trace_cameraRightPID.Kp = setP;
        Trace_cameraRightPID.Kp_Set = setP;

    }
    else if (traceType == TRACE_Camera_MID) {

    }
}


void Trace_SetPIDI(float setI, TRACE_TYPE traceType) {
    //------------------------------
    //设置寻左边线时候的I参数
    if (traceType == TRACE_Camera_LEFT) {
        Trace_cameraLeftPID.Ki = setI;
        Trace_cameraLeftPID.Ki_Set = setI;
    }
    //------------------------------
    //设置寻右边线时候的I参数
    else if (traceType == TRACE_Camera_RIGHT) {
        Trace_cameraRightPID.Ki = setI;
        Trace_cameraRightPID.Ki_Set = setI;
    }
    else if (traceType == TRACE_Camera_MID) {

    }
}
void Trace_SetPIDD(float setD, TRACE_TYPE traceType) {
    //------------------------------
    //设置寻左边线时候的D参数
    if (traceType == TRACE_Camera_LEFT) {
        Trace_cameraLeftPID.Kd = setD;
        Trace_cameraLeftPID.Kd_Set = setD;

    }
    //------------------------------
    //设置寻右边线时候的D参数
    else if (traceType == TRACE_Camera_RIGHT) {
        Trace_cameraRightPID.Kd = setD;
        Trace_cameraRightPID.Kd_Set = setD;
    }
    else if (traceType == TRACE_Camera_MID) {

    }
}
void Trace_SetPIDSumLimit(float sumLimit, TRACE_TYPE traceType) {
    //------------------------------
    //设置寻左边线时候的sumLimit参数
    if (traceType == TRACE_Camera_LEFT) {
        Trace_cameraLeftPID.sumLimit = sumLimit;
    }
    //------------------------------
    //设置寻右边线时候的sumLimit参数
    else if (traceType == TRACE_Camera_RIGHT) {
        Trace_cameraRightPID.sumLimit = sumLimit;
    }
    else if (traceType == TRACE_Camera_MID) {

    }
}
void Trace_SetPIDCoLimit(float coLimit, TRACE_TYPE traceType) {
    //------------------------------
    //设置寻左边线时候的CoLimit参数
    if (traceType == TRACE_Camera_LEFT) {
        Trace_cameraLeftPID.utLimit = coLimit;
    }
    //------------------------------
    //设置寻右边线时候的CoLimit参数
    else if (traceType == TRACE_Camera_RIGHT) {
        Trace_cameraRightPID.utLimit = coLimit;
    }
    else if (traceType == TRACE_Camera_MID) {

    }
}
