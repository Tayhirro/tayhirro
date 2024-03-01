/*
 * xiao_trace.c
 *
 *  Created on: 2023��7��2��
 *      Author: Jayden_NaN
 */
#include "xiao_trace.h"
//------------------------------------------------------------
//��������
TRACE_TYPE Trace_traceType = TRACE_Elec;
//==============================����ͷѰ�����==============================
//------------------------------------------------------------
//���߱�׼ֵ���
uint8 Trace_middleStandard = 94;                //����ͷ�������е����ڵ���(�����Ͼ�����IMAGE_WIDTH / 2 �м����Ҹ���)
//------------------------------------------------------------
//����Ѳ�ߴ���
float Trace_angleError = 0.0;                   //�Ƕ����
float Trace_angleErrorTher = 7.0;               //�Ƕ������ֵ
uint8 Trace_aimLine = 30;                       //���������ҵĵ�n������ΪĿ��ǰհ
float Trace_lineWeight[] = {0.5, 0.3, 0.2};     //��������ʱ�����м����Ȩ��
//------------------------------------------------------------
//PID���
PID Trace_cameraLeftPID;                       //����߻�ȡ�����ߵ�PID
PID Trace_cameraRightPID;                      //�ұ��߻�ȡ�����ߵ�PID


//==============================���Ѱ�����==============================


//==============================��������==============================

/*
 * @brief               �����ٶ�,������ͼ��ȷ��ǰհ
 * @return              ׷��Ŀ���е����ڵ���
 * @attention           ��Ҫ��ȡ��ǰ���ٶ�(Ҳ����˵��Ҫ���ٶȶ�ȡ���ж�����һֱ��)
 */
static void Trace_GetProspect(void) {
    if (Encoder_readFinishStatus == 1) {
        //ͨ���ٶ�,��̬�ı�ǰհ�ĳ���
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
 * @brief               ͨ�����м�Ȩ�ػ�ȡangleError
 * @attention           ֻ����������ͷ��,��ŵĻ�û������
 */
static void Trace_GetAngelError() {
    //------------------------------
    //��ȡĿ����
    Trace_GetProspect();

    //------------------------------
    //��ȡ���
    if (Trace_traceType == TRACE_Camera_LEFT) {
        Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine, 0, Image_rptsLeftcNum - 1)][0]
                        + Trace_lineWeight[1] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 1, 0, Image_rptsLeftcNum - 1)][0]
                        + Trace_lineWeight[2] * (float)Image_rptsLeftc[bf_clip(Trace_aimLine + 2, 0, Image_rptsLeftcNum - 1)][0];
    }
    else if (Trace_traceType == TRACE_Camera_RIGHT) {
        Trace_angleError = Trace_lineWeight[0] * (float)Image_rptsRightc[bf_clip(Trace_aimLine, 0, Image_rptsRightcNum - 1)][0]
                        + Trace_lineWeight[1] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 1, 0, Image_rptsRightcNum - 1)][0]
                        + Trace_lineWeight[2] * (float)Image_rptsRightc[bf_clip(Trace_aimLine + 2, 0, Image_rptsRightcNum - 1)][0];
    }
}

/*
 * @brief               PID��ʼ��(�������ݶ���ʼ��Ϊ0)
 * @return              NULL
 */
void Trace_PIDInit() {
   // PID_Init(&Trace_cameraLeftPID);
   // PID_Init(&Trace_cameraRightPID);
}

/*
 * @brief               Ѱ��PID��������
 * @parameter K_p_set   PID��P����
 * @parameter K_d_set   PID��D����
 * @parameter coLimit   �����޷�
 * @parameter boost
 * @parameter traceType Ѱ������
 * @attention           ������ֻ�ṩ������ͷPID�Ĳ�������,�������������Ч
 */
void Trace_PID_Set(float K_p_set, float K_d_set, float coLimit, float boost, TRACE_TYPE traceType) {
    //------------------------------
    //Ѱ����ͷ������ҵ������ߵ�PID
    if (traceType == TRACE_Camera_LEFT) {
      //  PID_SetParameter(&Trace_cameraLeftPID, K_p_set, 0, K_d_set, 0, coLimit, boost);
    }
    //------------------------------
    //Ѱ����ͷ�ұ����ҵ������ߵ�PID
    else if (traceType == TRACE_Camera_RIGHT) {
      //  PID_SetParameter(&Trace_cameraRightPID, K_p_set, 0, K_d_set, 0, coLimit, boost);
    }
    //------------------------------
    //���Ѱ����PID
    else if (traceType == TRACE_Elec) {

    }
}


/*
 * @brief               ����Ѱ��
 * @return              ���������ut
 */
float Trace_Run() {
    //----------------------------------------
    //���Ѱ��
    if (Trace_traceType == TRACE_Elec) {
        return 0;
    }
    //----------------------------------------
    //����ͷѰ����
    else if (Trace_traceType == TRACE_Camera_LEFT) {
        Trace_GetAngelError();
        Trace_PID_Set(Trace_cameraLeftPID.kPSet, Trace_cameraLeftPID.kDSet, Trace_cameraLeftPID.utLimit, 1.0, Trace_traceType);
     //   PID_PostionalPID(&Trace_cameraLeftPID, 0, Trace_angleError);
        return Trace_cameraLeftPID.ut;
    }
    //----------------------------------------
    //����ͷѰ����
    else if (Trace_traceType == TRACE_Camera_RIGHT) {
        Trace_GetAngelError();
        Trace_PID_Set(Trace_cameraRightPID.kPSet, Trace_cameraRightPID.kDSet, Trace_cameraRightPID.utLimit, 1.0, Trace_traceType);
       // PID_PostionalPID(&Trace_cameraRightPID, 0, Trace_angleError);
        return Trace_cameraRightPID.ut;
    }
    else
        return -1;
}

//------------------------------------------------------------
//����vofa�����ú���
void Trace_SetPIDP(float setP, TRACE_TYPE traceType) {
    //------------------------------
    //����Ѱ�����ʱ���P����
    if (traceType == TRACE_Camera_LEFT) {
        Trace_cameraLeftPID.kP = setP;
        Trace_cameraLeftPID.kPSet = setP;
    }
    //------------------------------
    //����Ѱ�ұ���ʱ���P����
    else if (traceType == TRACE_Camera_RIGHT) {
        Trace_cameraRightPID.kP = setP;
        Trace_cameraRightPID.kPSet = setP;

    }
    else if (traceType == TRACE_Elec) {

    }
}


void Trace_SetPIDI(float setI, TRACE_TYPE traceType) {
    //------------------------------
    //����Ѱ�����ʱ���I����
    if (traceType == TRACE_Camera_LEFT) {
        Trace_cameraLeftPID.kI = setI;
        Trace_cameraLeftPID.kISet = setI;
    }
    //------------------------------
    //����Ѱ�ұ���ʱ���I����
    else if (traceType == TRACE_Camera_RIGHT) {
        Trace_cameraRightPID.kI = setI;
        Trace_cameraRightPID.kISet = setI;
    }
    else if (traceType == TRACE_Elec) {

    }
}
void Trace_SetPIDD(float setD, TRACE_TYPE traceType) {
    //------------------------------
    //����Ѱ�����ʱ���D����
    if (traceType == TRACE_Camera_LEFT) {
        Trace_cameraLeftPID.kD = setD;
        Trace_cameraLeftPID.kDSet = setD;

    }
    //------------------------------
    //����Ѱ�ұ���ʱ���D����
    else if (traceType == TRACE_Camera_RIGHT) {
        Trace_cameraRightPID.kD = setD;
        Trace_cameraRightPID.kDSet = setD;
    }
    else if (traceType == TRACE_Elec) {

    }
}
void Trace_SetPIDSumLimit(float sumLimit, TRACE_TYPE traceType) {
    //------------------------------
    //����Ѱ�����ʱ���sumLimit����
    if (traceType == TRACE_Camera_LEFT) {
        Trace_cameraLeftPID.sumLimit = sumLimit;
    }
    //------------------------------
    //����Ѱ�ұ���ʱ���sumLimit����
    else if (traceType == TRACE_Camera_RIGHT) {
        Trace_cameraRightPID.sumLimit = sumLimit;
    }
    else if (traceType == TRACE_Elec) {

    }
}
void Trace_SetPIDCoLimit(float coLimit, TRACE_TYPE traceType) {
    //------------------------------
    //����Ѱ�����ʱ���CoLimit����
    if (traceType == TRACE_Camera_LEFT) {
        Trace_cameraLeftPID.utLimit = coLimit;
    }
    //------------------------------
    //����Ѱ�ұ���ʱ���CoLimit����
    else if (traceType == TRACE_Camera_RIGHT) {
        Trace_cameraRightPID.utLimit = coLimit;
    }
    else if (traceType == TRACE_Elec) {

    }
}
