/*
 * xiao_elec.c
 *
 *  Created on: 2023年5月1日
 *      Author: Jayden_NaN
 */
#include "xiao_show.h"
uint8 Show_status=1;
uint8 X;     uint8 Y;
uint8 ZOOMX;  uint8 ZOOMY;
uint8 P1y;  uint8 P2y;  uint8 P3y;  uint8 P4y;
uint8 P1x;  uint8 P2x;

uint8 Show_paramStatus[5] = {1,1,1,1,1};
uint8 Show_direction = 1;
uint8 Show_boost = 1;       //boost是用于
/**
 * @brief 显示功能初始化
 * @return NULL
 */
void Show_Init(void){
    if(SHOW_SCREEN == SCREEN_TFT180){
        tft180_init();
        tft180_set_font(TFT180_6X8_FONT);
        //tft坐标x最大分辨率160，坐标y最大分辨率128
    }
    else if(SHOW_SCREEN == SCREEN_IPS200){
        ips200_init(IPS200_TYPE_PARALLEL8);
        ips200_set_font(IPS200_8X16_FONT);
    }
    key_init(10);
    gpio_init(P33_13, GPI, 0, GPI_PULL_UP);
    gpio_init(P33_12, GPI, 0, GPI_PULL_UP);
}

/**
 * @brief 显示切换功能
 * @parameter Show_list[]     需要的书页
 * @parameter Show_max        总页数
 * KEY1，负责向前翻页
 * KEY1，负责向后翻页
 * @return NULL
 */
void Show_Switch(SHOW_MODE_enum Show_list[],uint8 Show_max){
    key_scanner();
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
        if (Show_status < Show_max) Show_status += 1;
        else Show_status = 1;
        if(SHOW_SCREEN == SCREEN_TFT180)tft180_clear();
        else if(SHOW_SCREEN == SCREEN_IPS200)ips200_clear();
    }
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
        if(Show_direction == 1){
            if(Show_list[Show_status-1] == Page1){
                if (Show_paramStatus[0] < 5) Show_paramStatus[0] += 1;
                else Show_paramStatus[0] = 1;
            }
            else if(Show_list[Show_status-1] == Page2){
                if (Show_paramStatus[1] < 3) Show_paramStatus[1] += 1;
                else Show_paramStatus[1] = 1;
            }
            else if(Show_list[Show_status-1] == Page3){
                if (Show_paramStatus[2] < 5) Show_paramStatus[2] += 1;
                else Show_paramStatus[2] = 1;
           }
            else if(Show_list[Show_status-1] == Page4){
                if (Show_paramStatus[3] < 6) Show_paramStatus[3] += 1;
                else Show_paramStatus[3] = 1;
           }
            else if(Show_list[Show_status-1] == Page5){
                if (Show_paramStatus[4] < 4) Show_paramStatus[4] += 1;
                else Show_paramStatus[4] = 1;
           }
        }
        else if(Show_direction == 0){
            if(Show_list[Show_status-1] == Page1){
                if (Show_paramStatus[0] > 1) Show_paramStatus[0] -= 1;
                else Show_paramStatus[0] = 5;
            }
            else if(Show_list[Show_status-1] == Page2){
                if (Show_paramStatus[1] > 1) Show_paramStatus[1] -= 1;
                else Show_paramStatus[1] = 3;
            }
            else if(Show_list[Show_status-1] == Page3){
                if (Show_paramStatus[2] > 1) Show_paramStatus[2] -= 1;
                else Show_paramStatus[2] = 5;
           }
            else if(Show_list[Show_status-1] == Page4){
                if (Show_paramStatus[3] > 1) Show_paramStatus[3] -= 1;
                else Show_paramStatus[3] = 6;
           }
            else if(Show_list[Show_status-1] == Page5){
                if (Show_paramStatus[4] > 1) Show_paramStatus[4] -= 1;
                else Show_paramStatus[4] = 4;
           }
        }
    }
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS){
        if(Show_boost == 1){
            if(Show_list[Show_status-1] == Page1){
                if(Show_paramStatus[0] == 1) Speed_set+=1;
                else if(Show_paramStatus[0] == 2) Speed_accVar+=1;
                else if(Show_paramStatus[0] == 3){
                    if(Barrier_Judege_Status == 0) Barrier_Judege_Status = 1;
                    else Barrier_Judege_Status = 0;
                }
                else if(Show_paramStatus[0] == 4){
                    if(Circle_multiCircle_Status == 0) Circle_multiCircle_Status = 1;
                    else Circle_multiCircle_Status = 0;
                }
                else if(Show_paramStatus[0] == 5){
                    if(Grage_outWarehous_Status == 0) Grage_outWarehous_Status = 1;
                    else Grage_outWarehous_Status = 0;
                }
            }
            else if(Show_list[Show_status-1] == Page2){
                if(Show_paramStatus[1] == 1) Circle_encoderLeft_Thre+=100;
                else if(Show_paramStatus[1] == 2) Circle_encoderRight_Thre+=100;
                else if(Show_paramStatus[1] == 3){
                    if(Circle_speedAcc_Status == 0) Circle_speedAcc_Status = 1;
                    else Circle_speedAcc_Status = 0;
                }
            }
            else if(Show_list[Show_status-1] == Page3){
                if(Show_paramStatus[2] == 1) Barrier_distance_Thre+=10;
                else if(Show_paramStatus[2] == 2) Barrier_speedVarTurnLeft_Motor1+=1;
                else if(Show_paramStatus[2] == 3) Barrier_speedVarTurnLeft_Motor2+=1;
                else if(Show_paramStatus[2] == 4) Barrier_AngleTurnLeft_Thre += 0.5;
                else if(Show_paramStatus[2] == 5) Barrier_encoderSumGoStraight_Thre += 100;
           }
            else if(Show_list[Show_status-1] == Page4){
                if(Show_paramStatus[3] == 1) Barrier_speedVarTurnRight_Motor1 += 1;
                else if(Show_paramStatus[3] == 2) Barrier_speedVarTurnRight_Motor2 += 1;
                else if(Show_paramStatus[3] == 3) Barrier_AngleTurnRight_Thre += 0.5;
                else if(Show_paramStatus[3] == 4) Barrier_encoderSumGoStraightEnd_Thre += 500;
                else if(Show_paramStatus[3] == 5) Barrier_usefulBarrier += 1;
                else if(Show_paramStatus[3] == 6) Barrier_encoderSumBridge_Thre += 1000;
           }
            else if(Show_list[Show_status-1] == Page5){
                if(Show_paramStatus[4] == 1) Grage_stroageSpeedVar_Motor1_Left += 1;
                else if(Show_paramStatus[4] == 2) Grage_stroageSpeedVar_Motor2_Left += 1;
                else if(Show_paramStatus[4] == 3) Grage_inAngle_Thre += 0.5;
                else if(Show_paramStatus[4] == 4) Grage_inStraight_Thre += 500;
           }
       }
        else if(Show_boost == 0){
            if(Show_list[Show_status-1] == Page1){
                    if(Show_paramStatus[0] == 1) Speed_set-=1;
                    else if(Show_paramStatus[0] == 2) Speed_accVar-=1;
                    else if(Show_paramStatus[0] == 3){
                        if(Barrier_Judege_Status == 0) Barrier_Judege_Status = 1;
                        else Barrier_Judege_Status = 0;
                    }
                    else if(Show_paramStatus[0] == 4){
                        if(Circle_multiCircle_Status == 0) Circle_multiCircle_Status = 1;
                        else Circle_multiCircle_Status = 0;
                    }
                    else if(Show_paramStatus[0] == 5){
                        if(Grage_outWarehous_Status == 0) Grage_outWarehous_Status = 1;
                        else Grage_outWarehous_Status = 0;
                    }
            }
            else if(Show_list[Show_status-1] == Page2){
                if(Show_paramStatus[1] == 1) Circle_encoderLeft_Thre-=100;
                else if(Show_paramStatus[1] == 2) Circle_encoderRight_Thre-=100;
                else if(Show_paramStatus[1] == 3){
                    if(Circle_speedAcc_Status == 0) Circle_speedAcc_Status = 1;
                    else Circle_speedAcc_Status = 0;
                }
            }
            else if(Show_list[Show_status-1] == Page3){
                if(Show_paramStatus[2] == 1) Barrier_distance_Thre-=10;
                else if(Show_paramStatus[2] == 2) Barrier_speedVarTurnLeft_Motor1-=1;
                else if(Show_paramStatus[2] == 3) Barrier_speedVarTurnLeft_Motor2-=1;
                else if(Show_paramStatus[2] == 4) Barrier_AngleTurnLeft_Thre -= 0.5;
                else if(Show_paramStatus[2] == 5) Barrier_encoderSumGoStraight_Thre -= 100;
           }
            else if(Show_list[Show_status-1] == Page4){
                if(Show_paramStatus[3] == 1) Barrier_speedVarTurnRight_Motor1 -= 1;
                else if(Show_paramStatus[3] == 2) Barrier_speedVarTurnRight_Motor2 -= 1;
                else if(Show_paramStatus[3] == 3) Barrier_AngleTurnRight_Thre -= 0.5;
                else if(Show_paramStatus[3] == 4) Barrier_encoderSumGoStraightEnd_Thre -= 500;
                else if(Show_paramStatus[3] == 5) Barrier_usefulBarrier -= 1;
                else if(Show_paramStatus[3] == 6) Barrier_encoderSumBridge_Thre -= 1000;
           }
            else if(Show_list[Show_status-1] == Page5){
                if(Show_paramStatus[4] == 1) Grage_stroageSpeedVar_Motor1_Left -= 1;
                else if(Show_paramStatus[4] == 2) Grage_stroageSpeedVar_Motor2_Left -= 1;
                else if(Show_paramStatus[4] == 3) Grage_inAngle_Thre -= 0.5;
                else if(Show_paramStatus[4] == 4) Grage_inStraight_Thre -= 500;
           }
        }
    }
    Show_direction = gpio_get_level(P33_13);
    Show_boost = gpio_get_level(P33_12);
    switch(Show_list[Show_status-1]){
        case Elec:Show_Elec();break;
        case Motor:Show_Motor();break;
        case Pid:Show_PID();break;
        case Page1:Show_Page1();break;
        case Page2:Show_Page2();break;
        case Page3:Show_Page3();break;
        case Page4:Show_Page4();break;
        case Page5:Show_Page5();break;
        default:break;
    }
    key_clear_all_state();
}
void Show_Page1(){
    if (SHOW_SCREEN == SCREEN_TFT180) {
        X = 5, Y = 10, ZOOMX = 8, ZOOMY = 10;
        if(Show_boost == 1) tft180_show_string(X, Y, "+");
        else tft180_show_string(X, Y, "-");
        if(Show_direction ==  1) tft180_show_string(X+ZOOMX, Y, "G:");
        else if(Show_direction ==  0) tft180_show_string(X+ZOOMX, Y, "B:");
        if(Show_paramStatus[0] == 1) tft180_show_string(X+3*ZOOMX, Y, "Speed       ");
        else if(Show_paramStatus[0] == 2) tft180_show_string(X+3*ZOOMX, Y, "Speed_A");
        else if(Show_paramStatus[0] == 3) tft180_show_string(X+3*ZOOMX, Y, "Bar_Jud");
        else if(Show_paramStatus[0] == 4) tft180_show_string(X+3*ZOOMX, Y, "Mul_Cir");
        else if(Show_paramStatus[0] == 5) tft180_show_string(X+3*ZOOMX, Y, "Gra_out");
        tft180_show_string(X, Y+2*ZOOMY, "Speed:"); tft180_show_float(X+7*ZOOMX, Y+2*ZOOMY, Speed_set, 3,1);
        tft180_show_string(X, Y+4*ZOOMY, "Speed_A:"); tft180_show_float(X+7*ZOOMX, Y+4*ZOOMY, Speed_accVar, 3,1);
        tft180_show_string(X, Y+6*ZOOMY, "Bar_Jud:"); tft180_show_int(X+7*ZOOMX, Y+6*ZOOMY, Barrier_Judege_Status, 1);
        tft180_show_string(X, Y+8*ZOOMY, "Mul_Cir:"); tft180_show_int(X+7*ZOOMX, Y+8*ZOOMY, Circle_multiCircle_Status, 1);
        tft180_show_string(X, Y+10*ZOOMY, "Gra_out:"); tft180_show_int(X+7*ZOOMX, Y+10*ZOOMY, Grage_outWarehous_Status, 1);
        tft180_show_string(X, Y+12*ZOOMY, "L3:");//tft180_show_int(X+3*ZOOMX, Y+12*ZOOMY, Elec_data[5], 4);
        tft180_show_string(X, Y+14*ZOOMY, "L5:");//tft180_show_int(X+3*ZOOMX, Y+14*ZOOMY, Elec_data[6], 4);
    }
    else if (SHOW_SCREEN == SCREEN_IPS200) {
        X = 5; Y = 5, ZOOMX = 16, ZOOMY = 16;
        if(Show_boost == 1) ips200_show_string(X, Y, "+");
        else ips200_show_string(X, Y, "-");
        if(Show_direction ==  1) ips200_show_string(X+ZOOMX, Y, "G:");
        else if(Show_direction ==  0) ips200_show_string(X+ZOOMX, Y, "B:");
        if(Show_paramStatus[0] == 1) ips200_show_string(X+3*ZOOMX, Y, "Speed       ");
        else if(Show_paramStatus[0] == 2) ips200_show_string(X+3*ZOOMX, Y, "Speed_A");
        else if(Show_paramStatus[0] == 3) ips200_show_string(X+3*ZOOMX, Y, "Bar_Jud");
        else if(Show_paramStatus[0] == 4) ips200_show_string(X+3*ZOOMX, Y, "Mul_Cir");
        else if(Show_paramStatus[0] == 5) ips200_show_string(X+3*ZOOMX, Y, "Gra_out");
        ips200_show_string(X, Y+2*ZOOMY, "Speed:"); ips200_show_float(X+7*ZOOMX, Y+2*ZOOMY, Speed_set, 3,1);
        ips200_show_string(X, Y+4*ZOOMY, "Speed_A:"); ips200_show_float(X+7*ZOOMX, Y+4*ZOOMY, Speed_accVar, 3,1);
        ips200_show_string(X, Y+6*ZOOMY, "Bar_Jud:"); ips200_show_int(X+7*ZOOMX, Y+6*ZOOMY, Barrier_Judege_Status, 1);
        ips200_show_string(X, Y+8*ZOOMY, "Mul_Cir:"); ips200_show_int(X+7*ZOOMX, Y+8*ZOOMY, Circle_multiCircle_Status, 1);
        ips200_show_string(X, Y+10*ZOOMY, "Gra_out:"); ips200_show_int(X+7*ZOOMX, Y+10*ZOOMY, Grage_outWarehous_Status, 1);
        ips200_show_string(X, Y+12*ZOOMY, "L3:");//ips200_show_int(X+3*ZOOMX, Y+12*ZOOMY, Elec_data[5], 4);
        ips200_show_string(X, Y+14*ZOOMY, "L5:");//ips200_show_int(X+3*ZOOMX, Y+14*ZOOMY, Elec_data[6], 4);
    }
}
void Show_Page2(){
    if (SHOW_SCREEN == SCREEN_TFT180) {
           X = 5, Y = 10, ZOOMX = 8, ZOOMY = 10;
           if(Show_boost == 1) tft180_show_string(X, Y, "+");
           else tft180_show_string(X, Y, "-");
           if(Show_direction ==  1) tft180_show_string(X+ZOOMX, Y, "G:");
           else if(Show_direction ==  0) tft180_show_string(X+ZOOMX, Y, "B:");
           if(Show_paramStatus[1] == 1)       tft180_show_string(X+3*ZOOMX, Y, "EnLeft_T ");
           else if(Show_paramStatus[1] == 2)  tft180_show_string(X+3*ZOOMX, Y, "EnRight_T");
           else if(Show_paramStatus[1] == 3)  tft180_show_string(X+3*ZOOMX, Y, "Cir_Acc  ");
           tft180_show_string(X+11*ZOOMX, Y, "Cir");
           tft180_show_string(X, Y+2*ZOOMY, "EnLeft_T:"); tft180_show_int(X+8*ZOOMX, Y+2*ZOOMY, Circle_encoderLeft_Thre, 5);
           tft180_show_string(X, Y+4*ZOOMY, "EnRight_T:"); tft180_show_int(X+8*ZOOMX, Y+4*ZOOMY, Circle_encoderRight_Thre, 5);
           tft180_show_string(X, Y+6*ZOOMY, "Cir_Acc:"); tft180_show_int(X+8*ZOOMX, Y+6*ZOOMY, Circle_speedAcc_Status, 1);
    }
    else if (SHOW_SCREEN == SCREEN_IPS200) {
          X = 5; Y = 5, ZOOMX = 16, ZOOMY = 16;
          if(Show_boost == 1) ips200_show_string(X, Y, "+");
          else ips200_show_string(X, Y, "-");
          if(Show_direction ==  1) ips200_show_string(X+ZOOMX, Y, "G:");
          else if(Show_direction ==  0) ips200_show_string(X+ZOOMX, Y, "B:");
          if(Show_paramStatus[1] == 1)       ips200_show_string(X+3*ZOOMX, Y, "EnLeft_T ");
          else if(Show_paramStatus[1] == 2)  ips200_show_string(X+3*ZOOMX, Y, "EnRight_T");
          else if(Show_paramStatus[1] == 3)  ips200_show_string(X+3*ZOOMX, Y, "Cir_Acc");
          ips200_show_string(X+11*ZOOMX, Y, "Cir");
          ips200_show_string(X, Y+2*ZOOMY, "EnLeft_T:"); ips200_show_int(X+8*ZOOMX, Y+2*ZOOMY, Circle_encoderLeft_Thre, 5);
          ips200_show_string(X, Y+4*ZOOMY, "EnRight_T:"); ips200_show_int(X+8*ZOOMX, Y+4*ZOOMY, Circle_encoderRight_Thre, 5);
          ips200_show_string(X, Y+6*ZOOMY, "Cir_Acc:"); ips200_show_int(X+8*ZOOMX, Y+6*ZOOMY, Circle_speedAcc_Status, 1);
    }
}
void Show_Page3(){
    if (SHOW_SCREEN == SCREEN_TFT180) {
           X = 5, Y = 10, ZOOMX = 8, ZOOMY = 10;
           if(Show_boost == 1) tft180_show_string(X, Y, "+");
           else tft180_show_string(X, Y, "-");
           if(Show_direction ==  1) tft180_show_string(X+ZOOMX, Y, "G:");
           else if(Show_direction ==  0) tft180_show_string(X+ZOOMX, Y, "B:");
           if(Show_paramStatus[2] == 1 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS))      {
               tft180_show_string(X+3*ZOOMX, Y, "Dis      ");
               tft180_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_RED);
               tft180_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+10*ZOOMY,X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);

           }
           else if(Show_paramStatus[2] == 2 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
               tft180_show_string(X+3*ZOOMX, Y, "LS_M1    ");
               tft180_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_RED);
               tft180_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+10*ZOOMY,X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
           }
           else if(Show_paramStatus[2] == 3 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
               tft180_show_string(X+3*ZOOMX, Y, "LS_M2    ");
               tft180_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_RED);
               tft180_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+10*ZOOMY,X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
           }
           else if(Show_paramStatus[2] == 4 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
               tft180_show_string(X+3*ZOOMX, Y, "LeftAngle");
               tft180_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_RED);
               tft180_draw_line(X+13*ZOOMX, Y+10*ZOOMY, X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
           }
           else if(Show_paramStatus[2] == 5 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
               tft180_show_string(X+3*ZOOMX, Y, "SumGo    ");
               tft180_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
               tft180_draw_line(X+13*ZOOMX, Y+10*ZOOMY, X+14*ZOOMX,Y+10*ZOOMY,RGB565_RED);
           }
           tft180_show_string(X+11*ZOOMX, Y, "Bar1");
           tft180_show_string(X, Y+2*ZOOMY, "Dis:"); tft180_show_int(X+3*ZOOMX, Y+2*ZOOMY, Barrier_distance_Thre, 4);
           tft180_show_string(X, Y+4*ZOOMY, "LS_M1:"); tft180_show_int(X+5*ZOOMX, Y+4*ZOOMY, Barrier_speedVarTurnLeft_Motor1, 3);
           tft180_show_string(X, Y+6*ZOOMY, "LS_M2:"); tft180_show_int(X+5*ZOOMX, Y+6*ZOOMY, Barrier_speedVarTurnLeft_Motor2, 3);
           tft180_show_string(X, Y+8*ZOOMY, "LeftAngle:"); tft180_show_float(X+8*ZOOMX, Y+8*ZOOMY, Barrier_AngleTurnLeft_Thre, 3,1);
           tft180_show_string(X, Y+10*ZOOMY, "SumGo:"); tft180_show_int(X+5*ZOOMX, Y+10*ZOOMY, Barrier_encoderSumGoStraight_Thre, 5);
    }
    else if (SHOW_SCREEN == SCREEN_IPS200) {
           X = 5; Y = 5, ZOOMX = 16, ZOOMY = 16;
           if(Show_boost == 1) ips200_show_string(X, Y, "+");
           else ips200_show_string(X, Y, "-");
           if(Show_direction ==  1) ips200_show_string(X+ZOOMX, Y, "G:");
           else if(Show_direction ==  0) ips200_show_string(X+ZOOMX, Y, "B:");
           if(Show_paramStatus[2] == 1 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS))      {
               ips200_show_string(X+3*ZOOMX, Y, "Dis      ");
               ips200_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_RED);
               ips200_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+10*ZOOMY,X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);

           }
           else if(Show_paramStatus[2] == 2 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
               ips200_show_string(X+3*ZOOMX, Y, "LS_M1    ");
               ips200_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_RED);
               ips200_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+10*ZOOMY,X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
           }
           else if(Show_paramStatus[2] == 3 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
               ips200_show_string(X+3*ZOOMX, Y, "LS_M2    ");
               ips200_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_RED);
               ips200_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+10*ZOOMY,X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
           }
           else if(Show_paramStatus[2] == 4 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
               ips200_show_string(X+3*ZOOMX, Y, "LeftAngle");
               ips200_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_RED);
               ips200_draw_line(X+13*ZOOMX, Y+10*ZOOMY, X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
           }
           else if(Show_paramStatus[2] == 5 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
               ips200_show_string(X+3*ZOOMX, Y, "SumGo    ");
               ips200_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
               ips200_draw_line(X+13*ZOOMX, Y+10*ZOOMY, X+14*ZOOMX,Y+10*ZOOMY,RGB565_RED);
           }
           ips200_show_string(X+11*ZOOMX, Y, "Bar1");
           ips200_show_string(X, Y+2*ZOOMY, "Dis:"); ips200_show_int(X+3*ZOOMX, Y+2*ZOOMY, Barrier_distance_Thre, 4);
           ips200_show_string(X, Y+4*ZOOMY, "LS_M1:"); ips200_show_int(X+5*ZOOMX, Y+4*ZOOMY, Barrier_speedVarTurnLeft_Motor1, 3);
           ips200_show_string(X, Y+6*ZOOMY, "LS_M2:"); ips200_show_int(X+5*ZOOMX, Y+6*ZOOMY, Barrier_speedVarTurnLeft_Motor2, 3);
           ips200_show_string(X, Y+8*ZOOMY, "LeftAngle:"); ips200_show_float(X+8*ZOOMX, Y+8*ZOOMY, Barrier_AngleTurnLeft_Thre, 3,1);
           ips200_show_string(X, Y+10*ZOOMY, "SumGo:"); ips200_show_int(X+5*ZOOMX, Y+10*ZOOMY, Barrier_encoderSumGoStraight_Thre, 5);

    }
}
void Show_Page4(){
    if (SHOW_SCREEN == SCREEN_TFT180) {
           X = 5, Y = 10, ZOOMX = 8, ZOOMY = 10;
           if(Show_boost == 1) tft180_show_string(X, Y, "+");
           else tft180_show_string(X, Y, "-");
           if(Show_direction ==  1) tft180_show_string(X+ZOOMX, Y, "G:");
           else if(Show_direction ==  0) tft180_show_string(X+ZOOMX, Y, "B:");
           if(Show_paramStatus[3] == 1 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS))      {
              tft180_show_string(X+3*ZOOMX, Y, "RS_M1     ");
              tft180_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_RED);
              tft180_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+10*ZOOMY,X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+12*ZOOMY, X+14*ZOOMX,Y+12*ZOOMY,RGB565_WHITE);
          }
          else if(Show_paramStatus[3] == 2 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
              tft180_show_string(X+3*ZOOMX, Y, "RS_M2     ");
              tft180_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_RED);
              tft180_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+10*ZOOMY,X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+12*ZOOMY, X+14*ZOOMX,Y+12*ZOOMY,RGB565_WHITE);
          }
          else if(Show_paramStatus[3] == 3 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
              tft180_show_string(X+3*ZOOMX, Y, "RightAngle");
              tft180_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_RED);
              tft180_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+10*ZOOMY,X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+12*ZOOMY, X+14*ZOOMX,Y+12*ZOOMY,RGB565_WHITE);
          }
          else if(Show_paramStatus[3] == 4 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
              tft180_show_string(X+3*ZOOMX, Y, "SumGoEnd  ");
              tft180_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_RED);
              tft180_draw_line(X+13*ZOOMX, Y+10*ZOOMY, X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+12*ZOOMY, X+14*ZOOMX,Y+12*ZOOMY,RGB565_WHITE);
          }
          else if(Show_paramStatus[3] == 5 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
              tft180_show_string(X+3*ZOOMX, Y, "UsefulBar ");
              tft180_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+10*ZOOMY, X+14*ZOOMX,Y+10*ZOOMY,RGB565_RED);
              tft180_draw_line(X+13*ZOOMX, Y+12*ZOOMY, X+14*ZOOMX,Y+12*ZOOMY,RGB565_WHITE);
          }
          else if(Show_paramStatus[3] == 6 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
              tft180_show_string(X+3*ZOOMX, Y, "SumBridge ");
              tft180_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+10*ZOOMY, X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
              tft180_draw_line(X+13*ZOOMX, Y+12*ZOOMY, X+14*ZOOMX,Y+12*ZOOMY,RGB565_RED);
          }
           tft180_show_string(X+11*ZOOMX, Y, "Bar2");
           tft180_show_string(X, Y+2*ZOOMY, "RS_M1:"); tft180_show_int(X+5*ZOOMX, Y+2*ZOOMY, Barrier_speedVarTurnRight_Motor1, 3);
           tft180_show_string(X, Y+4*ZOOMY, "RS_M2:"); tft180_show_int(X+5*ZOOMX, Y+4*ZOOMY, Barrier_speedVarTurnRight_Motor2, 3);
           tft180_show_string(X, Y+6*ZOOMY, "RightAngle:"); tft180_show_float(X+8*ZOOMX+3, Y+6*ZOOMY, Barrier_AngleTurnRight_Thre, 3,1);
           tft180_show_string(X, Y+8*ZOOMY, "SumGoEnd:"); tft180_show_uint(X+7*ZOOMX, Y+8*ZOOMY, Barrier_encoderSumGoStraightEnd_Thre, 6);
           tft180_show_string(X, Y+10*ZOOMY, "UsefulBar:"); tft180_show_int(X+8*ZOOMX, Y+10*ZOOMY, Barrier_usefulBarrier, 1);
           tft180_show_string(X, Y+12*ZOOMY, "SumBridge:"); tft180_show_uint(X+8*ZOOMX, Y+12*ZOOMY, Barrier_encoderSumBridge_Thre, 6);
    }
    else if (SHOW_SCREEN == SCREEN_IPS200) {
          X = 5; Y = 5, ZOOMX = 16, ZOOMY = 16;
          if(Show_boost == 1) ips200_show_string(X, Y, "+");
          else ips200_show_string(X, Y, "-");
          if(Show_direction ==  1) ips200_show_string(X+ZOOMX, Y, "G:");
          else if(Show_direction ==  0) ips200_show_string(X+ZOOMX, Y, "B:");
          if(Show_paramStatus[3] == 1 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS) )      {
             ips200_show_string(X+3*ZOOMX, Y, "RS_M1     ");
             ips200_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_RED);
             ips200_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+10*ZOOMY,X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+12*ZOOMY, X+14*ZOOMX,Y+12*ZOOMY,RGB565_WHITE);
         }
         else if(Show_paramStatus[3] == 2 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
             ips200_show_string(X+3*ZOOMX, Y, "RS_M2     ");
             ips200_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_RED);
             ips200_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+10*ZOOMY,X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+12*ZOOMY, X+14*ZOOMX,Y+12*ZOOMY,RGB565_WHITE);
         }
         else if(Show_paramStatus[3] == 3 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
             ips200_show_string(X+3*ZOOMX, Y, "RightAngle");
             ips200_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_RED);
             ips200_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+10*ZOOMY,X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+12*ZOOMY, X+14*ZOOMX,Y+12*ZOOMY,RGB565_WHITE);
         }
         else if(Show_paramStatus[3] == 4 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
             ips200_show_string(X+3*ZOOMX, Y, "SumGoEnd  ");
             ips200_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_RED);
             ips200_draw_line(X+13*ZOOMX, Y+10*ZOOMY, X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+12*ZOOMY, X+14*ZOOMX,Y+12*ZOOMY,RGB565_WHITE);
         }
         else if(Show_paramStatus[3] == 5 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
             ips200_show_string(X+3*ZOOMX, Y, "UsefulBar ");
             ips200_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+10*ZOOMY, X+14*ZOOMX,Y+10*ZOOMY,RGB565_RED);
             ips200_draw_line(X+13*ZOOMX, Y+12*ZOOMY, X+14*ZOOMX,Y+12*ZOOMY,RGB565_WHITE);
         }
         else if(Show_paramStatus[3] == 6 && (key_get_state(KEY_2) == KEY_SHORT_PRESS || key_get_state(KEY_1) == KEY_SHORT_PRESS)) {
             ips200_show_string(X+3*ZOOMX, Y, "SumBridge ");
             ips200_draw_line(X+13*ZOOMX, Y+2*ZOOMY, X+14*ZOOMX, Y+2*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+4*ZOOMY, X+14*ZOOMX, Y+4*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+6*ZOOMY, X+14*ZOOMX, Y+6*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+8*ZOOMY, X+14*ZOOMX, Y+8*ZOOMY, RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+10*ZOOMY, X+14*ZOOMX,Y+10*ZOOMY,RGB565_WHITE);
             ips200_draw_line(X+13*ZOOMX, Y+12*ZOOMY, X+14*ZOOMX,Y+12*ZOOMY,RGB565_RED);
         }
          ips200_show_string(X+11*ZOOMX, Y, "Bar2");
          ips200_show_string(X, Y+2*ZOOMY, "RS_M1:"); ips200_show_int(X+5*ZOOMX, Y+2*ZOOMY, Barrier_speedVarTurnRight_Motor1, 3);
          ips200_show_string(X, Y+4*ZOOMY, "RS_M2:"); ips200_show_int(X+5*ZOOMX, Y+4*ZOOMY, Barrier_speedVarTurnRight_Motor2, 3);
          ips200_show_string(X, Y+6*ZOOMY, "RightAngle:"); ips200_show_float(X+8*ZOOMX+3, Y+6*ZOOMY, Barrier_AngleTurnRight_Thre, 3,1);
          ips200_show_string(X, Y+8*ZOOMY, "SumGoEnd:"); ips200_show_uint(X+7*ZOOMX, Y+8*ZOOMY, Barrier_encoderSumGoStraightEnd_Thre, 6);
          ips200_show_string(X, Y+10*ZOOMY, "UsefulBar:"); ips200_show_int(X+8*ZOOMX, Y+10*ZOOMY, Barrier_usefulBarrier, 1);
          ips200_show_string(X, Y+12*ZOOMY, "SumBridge:"); ips200_show_uint(X+8*ZOOMX, Y+12*ZOOMY, Barrier_encoderSumBridge_Thre, 6);

    }
}
void Show_Page5(){
    if (SHOW_SCREEN == SCREEN_TFT180) {
          X = 5, Y = 10, ZOOMX = 8, ZOOMY = 10;
          if(Show_boost == 1) tft180_show_string(X, Y, "+");
          else tft180_show_string(X, Y, "-");
          if(Show_direction ==  1) tft180_show_string(X+ZOOMX, Y, "G:");
          else if(Show_direction ==  0) tft180_show_string(X+ZOOMX, Y, "B:");
          if(Show_paramStatus[4] == 1)       tft180_show_string(X+3*ZOOMX, Y, "GraLS_M1");
          else if(Show_paramStatus[4] == 2)  tft180_show_string(X+3*ZOOMX, Y, "GraLS_M2");
          else if(Show_paramStatus[4] == 3)  tft180_show_string(X+3*ZOOMX, Y, "InAngle ");
          else if(Show_paramStatus[4] == 4)  tft180_show_string(X+3*ZOOMX, Y, "InST    ");
          tft180_show_string(X+11*ZOOMX, Y, "Gra");
          tft180_show_string(X, Y+2*ZOOMY, "GraLS_M1:"); tft180_show_int(X+8*ZOOMX, Y+2*ZOOMY, Grage_stroageSpeedVar_Motor1_Left, 1);
          tft180_show_string(X, Y+4*ZOOMY, "GraLS_M2:"); tft180_show_int(X+8*ZOOMX, Y+4*ZOOMY, Grage_stroageSpeedVar_Motor2_Left, 2);
          tft180_show_string(X, Y+6*ZOOMY, "InAngle:"); tft180_show_float(X+8*ZOOMX, Y+6*ZOOMY, Grage_inAngle_Thre, 3,1);
          tft180_show_string(X, Y+8*ZOOMY, "InST:"); tft180_show_int(X+8*ZOOMX, Y+8*ZOOMY, Grage_inStraight_Thre, 5);
   }
   else if (SHOW_SCREEN == SCREEN_IPS200) {
         X = 5; Y = 5, ZOOMX = 16, ZOOMY = 16;
         if(Show_boost == 1) ips200_show_string(X, Y, "+");
           else ips200_show_string(X, Y, "-");
           if(Show_direction ==  1) ips200_show_string(X+ZOOMX, Y, "G:");
           else if(Show_direction ==  0) ips200_show_string(X+ZOOMX, Y, "B:");
           if(Show_paramStatus[4] == 1)       ips200_show_string(X+3*ZOOMX, Y, "GraLS_M1");
           else if(Show_paramStatus[4] == 2)  ips200_show_string(X+3*ZOOMX, Y, "GraLS_M2");
           else if(Show_paramStatus[4] == 3)  ips200_show_string(X+3*ZOOMX, Y, "InAngle ");
           else if(Show_paramStatus[4] == 4)  ips200_show_string(X+3*ZOOMX, Y, "InST    ");
           ips200_show_string(X+11*ZOOMX, Y, "Gra");
           ips200_show_string(X, Y+2*ZOOMY, "GraLS_M1:"); ips200_show_int(X+8*ZOOMX, Y+2*ZOOMY, Grage_stroageSpeedVar_Motor1_Left, 1);
           ips200_show_string(X, Y+4*ZOOMY, "GraLS_M2:"); ips200_show_int(X+8*ZOOMX, Y+4*ZOOMY, Grage_stroageSpeedVar_Motor2_Left, 2);
           ips200_show_string(X, Y+6*ZOOMY, "InAngle:"); ips200_show_float(X+8*ZOOMX, Y+6*ZOOMY, Grage_inAngle_Thre, 3,1);
           ips200_show_string(X, Y+8*ZOOMY, "InST:"); ips200_show_int(X+8*ZOOMX, Y+8*ZOOMY, Grage_inStraight_Thre, 5);

   }
}
/**
 * @brief 电感数据显示
 * @return NULL
 */
void Show_Elec(void){
    if (SHOW_SCREEN == SCREEN_TFT180) {
            X = 5, Y = 10, ZOOMX = 8, ZOOMY = 10;
    }
    else if (SHOW_SCREEN == SCREEN_IPS200) {
        X = 5; Y = 5, ZOOMX = 16, ZOOMY = 16;
    }
    P1x=Show_Elec_L(X,Y,ZOOMX,ZOOMY);
    P2x=Show_Elec_Diff(P1x, Y, ZOOMX, ZOOMY);
}
/**
 * @brief               显示电感值
 * @parameter x         坐标x方向的起点
 * @parameter y         坐标y方向的起点
 * @parameter zoomx     x方向间隔
 * @parameter zoomy     y方向间隔
 * @return NULL         下一模块的起点x坐标
 */
uint8 Show_Elec_L(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy){
    if(SHOW_SCREEN == SCREEN_TFT180){
        tft180_show_string(x, y, "L4:"); //tft180_show_int(x+3*zoomx, y, Elec_data[4], 4);
        tft180_show_string(x, y+2*zoomy, "L3:");//tft180_show_int(x+3*zoomx, y+2*zoomy, Elec_data[5], 4);
        tft180_show_string(x, y+4*zoomy, "L5:");//tft180_show_int(x+3*zoomx, y+4*zoomy, Elec_data[6], 4);
        tft180_show_string(x, y+6*zoomy, "L7:");//tft180_show_int(x+3*zoomx, y+6*zoomy, Elec_data[7], 4);
    }
    else if(SHOW_SCREEN == SCREEN_IPS200){
        ips200_show_string(x, y, "A0:"); //ips200_show_int(x + 3 * zoomx, y, Elec_data[0], 4);
        ips200_show_string(x, y + 2 * zoomy, "A1:"); //ips200_show_int(x + 3 * zoomx, y + 2 * zoomy, Elec_data[1], 4);
        ips200_show_string(x, y + 4 * zoomy, "A2:"); //ips200_show_int(x + 3 * zoomx, y + 4 * zoomy, Elec_data[2], 4);
        ips200_show_string(x, y + 6 * zoomy, "A3:"); //ips200_show_int(x + 3 * zoomx, y + 6 * zoomy, Elec_data[3], 4);
        ips200_show_string(x, y + 8 * zoomy, "A4:"); //ips200_show_int(x + 3 * zoomx, y + 8 * zoomy, Elec_data[4], 4);
        ips200_show_string(x, y + 10 * zoomy, "A5:"); //ips200_show_int(x + 3 * zoomx, y + 10 * zoomy, Elec_data[5], 4);
        ips200_show_string(x, y + 12 * zoomy, "A6:"); //ips200_show_int(x + 3 * zoomx, y + 12 * zoomy, Elec_data[6], 4);
        ips200_show_string(x, y + 14 * zoomy, "A7:"); //ips200_show_int(x + 3 * zoomx, y + 14 * zoomy, Elec_data[7], 4);
    }
    return (4+3)*zoomx+x;
}
/**
 * @brief               显示电感差值相关
 * @parameter x         坐标x方向的起点
 * @parameter y         坐标y方向的起点
 * @parameter zoomx     x方向间隔
 * @parameter zoomy     y方向间隔
 * @return uin8         下一模块的起点x坐标
 */
uint8 Show_Elec_Diff(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy){
    if(SHOW_SCREEN == SCREEN_TFT180){
        tft180_show_string(x, y, "E53:"); //tft180_show_int(x + 3 * zoomx, y, Elec_diff53, 4);
        tft180_show_string(x, y + 2 * zoomy, "E62:"); //tft180_show_int(x + 3 * zoomx, y + 2 * zoomy, Elec_diff62, 4);
    }
    else if(SHOW_SCREEN == SCREEN_IPS200){
        ips200_show_string(x, y, "E53:"); //ips200_show_int(x + 3 * zoomx, y,Elec_diff53, 4);
        ips200_show_string(x, y + 2 * zoomy, "E:"); //ips200_show_float(x + 2 * zoomx, y + 2 * zoomy, Elec_cur, 3,3);
        ips200_show_string(x, y + 4 * zoomy, "X:"); //ips200_show_float(x + 2 * zoomx, y + 4 * zoomy, Gyro_corrX, 3,3);
    }
    return (4+3)*zoomx+x;
}
/**
 * @brief 后轮数据显示
 * @return NULL
 */
void Show_Motor(void){
    if(SHOW_SCREEN == SCREEN_TFT180){
       X=5,Y=10,ZOOMX=7,ZOOMY=8;
    }
    else if(SHOW_SCREEN == SCREEN_IPS200){
        X=5,Y=10,ZOOMX=12,ZOOMY=16;
    }
    P1y = Show_Motor_Speed(X, Y, ZOOMX, ZOOMY);
    P2y = Show_Motor_Motor1(X,P1y,ZOOMX,ZOOMY);
    P3y = Show_Motor_Motor2(X,P2y,ZOOMX,ZOOMY);
}
/**
 * @brief               显示Speed
 * @parameter x         坐标x方向的起点
 * @parameter y         坐标y方向的起点
 * @parameter zoomx     x方向间隔
 * @parameter zoomy     y方向间隔
 * @return uint8        下一模块的起点y坐标
 */
uint8 Show_Motor_Speed(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy){
    if(SHOW_SCREEN == SCREEN_TFT180){
        tft180_show_string(x, y, "Speed:"); tft180_show_int(x + 7 * zoomx, y, Speed, 4);
        tft180_show_string(x, y + 4 * zoomy, "S_Set:"); tft180_show_int(x + 7 * zoomx, y + 4 * zoomy, Speed_set,4);
        return (2+4)*zoomy+y;
    }
    else if(SHOW_SCREEN == SCREEN_IPS200){
        ips200_show_string(x, y, "Speed:"); ips200_show_int(x + 7 * zoomx, y, Speed, 4);
        ips200_show_string(x, y + 2 * zoomy, "S_Set:"); ips200_show_int(x + 7 * zoomx, y + 2 * zoomy, Speed_set,4);
        return (2+2)*zoomy+y;
    }

}
/**
 * @brief               显示Motor1相关数据
 * @parameter x         坐标x方向的起点
 * @parameter y         坐标y方向的起点
 * @parameter zoomx     x方向间隔
 * @parameter zoomy     y方向间隔
 * @return uint8        下一模块的起点y坐标
 */
uint8 Show_Motor_Motor1(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy){
    if(SHOW_SCREEN == SCREEN_TFT180){
        tft180_show_string(x + 5 * zoomx, y, "MOTOR1");
        tft180_show_string(x, y + 2 * zoomy, "Target:"); tft180_show_int(x + 7 * zoomx, y + 2 * zoomy, Motor_1Target, 4);
        tft180_show_string(x, y + 4 * zoomy, "Encode:"); tft180_show_int(x + 7 * zoomx, y + 4 * zoomy, Encoder_1Data, 4);
    }
    else if(SHOW_SCREEN == SCREEN_IPS200){
        ips200_show_string(x + 7 * zoomx, y, "MOTOR1");
        ips200_show_string(x, y + 2 * zoomy, "Target:"); ips200_show_int(x + 7 * zoomx, y + 2 * zoomy, Motor_1Target, 4);
        ips200_show_string(x, y + 4 * zoomy, "Encode:"); ips200_show_int(x + 7 * zoomx, y + 4 * zoomy, Encoder_1Data, 4);
    }
    return (2+4)*zoomy+y;
}
/**
 * @brief               显示Motor2相关数据
 * @parameter x         坐标x方向的起点
 * @parameter y         坐标y方向的起点
 * @parameter zoomx     x方向间隔
 * @parameter zoomy     y方向间隔
 * @return uint8        下一模块的起点y坐标
 */
uint8 Show_Motor_Motor2(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy){
    if(SHOW_SCREEN == SCREEN_TFT180){
        tft180_show_string(x + 5 * zoomx, y, "MOTOR2");
        tft180_show_string(x, y + 2 * zoomy, "Target:"); tft180_show_int(x + 7 * zoomx, y + 2 * zoomy, Motor_2Target, 4);
        tft180_show_string(x, y + 4 * zoomy, "Encode:"); tft180_show_int(x + 7 * zoomx, y + 4 * zoomy, Encoder_2Data, 4);
    }
    else if(SHOW_SCREEN == SCREEN_IPS200){
        ips200_show_string(x + 7 * zoomx, y, "MOTOR2");
        ips200_show_string(x, y + 2 * zoomy, "Target:"); ips200_show_int(x + 7 * zoomx, y + 2 * zoomy, Motor_2Target, 4);
        ips200_show_string(x, y + 4 * zoomy, "Encode:"); ips200_show_int(x + 7 * zoomx, y + 4 * zoomy, Encoder_2Data, 4);
    }
    return (2+4)*zoomy+y;
}
/**
 * @brief PID数据显示
 * @return NULL
 */
void Show_PID(void){
    if(SHOW_SCREEN == SCREEN_TFT180){
       X=5,Y=10,ZOOMX=7,ZOOMY=8;
    }
    else if(SHOW_SCREEN == SCREEN_IPS200){
        X=2,Y=10,ZOOMX=16,ZOOMY=14;
    }
    P1y=Show_PID_Elec(X, Y, ZOOMX, ZOOMY);
    P2y=Show_PID_Motor1(X, P1y, ZOOMX, ZOOMY);
    P3y=Show_PID_Motor2(X, P2y, ZOOMX, ZOOMY);
}

/**
 * @brief               显示电磁PID相关数据
 * @parameter x         坐标x方向的起点
 * @parameter y         坐标y方向的起点
 * @parameter zoomx     x方向间隔
 * @parameter zoomy     y方向间隔
 * @return uint8        下一模块的起点y坐标
 */
uint8 Show_PID_Elec(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy){
    if(SHOW_SCREEN == SCREEN_TFT180){
       tft180_show_string(x + 5 * zoomx, y, "Elec");
       if(Elec_pidStatus == 1)         tft180_show_string(x + 9 * zoomx, y, "(ON)");
       else if(Elec_pidStatus == 0)    tft180_show_string(x + 9 * zoomx, y, "(OFF)");
       tft180_show_string(x, y + 2 * zoomy, "P:"); //tft180_show_float(x + 2 * zoomx, y + 2 * zoomy, Elec_P, 3, 3);
       tft180_show_string(x + 8 * zoomx, y + 2 * zoomy, "D:"); //tft180_show_float(x + 10 * zoomx, y + 2 * zoomy, Elec_D, 5, 2);
       tft180_show_string(x, y + 4 * zoomy, "E:"); //tft180_show_float(x + 2 * zoomx, y + 4 * zoomy, Elec_cur, 3,1);
       tft180_show_string(x + 7 * zoomx, y + 4 * zoomy, "COR:"); tft180_show_float(x + 11 * zoomx, y + 4 * zoomy, Elec_cor, 4, 1);
     }
    else if(SHOW_SCREEN == SCREEN_IPS200){
        ips200_show_string(x + 5 * zoomx, y, "Elec");
        if(Elec_pidStatus == 1)         ips200_show_string(x + 8 * zoomx, y, "(ON)");
        else if(Elec_pidStatus == 0)    ips200_show_string(x + 8 * zoomx, y, "(OFF)");
        ips200_show_string(x, y + 2 * zoomy, "P:"); //ips200_show_float(x + 2 * zoomx, y + 2 * zoomy, Elec_P, 3, 3);
        ips200_show_string(x + 8 * zoomx, y + 2 * zoomy, "D:"); //ips200_show_float(x + 10 * zoomx, y + 2 * zoomy, Elec_D, 5, 1);
        ips200_show_string(x, y + 4 * zoomy, "E:"); //ips200_show_float(x + 2 * zoomx, y + 4 * zoomy, Elec_cur, 3,3);
        ips200_show_string(x + 7 * zoomx, y + 4 * zoomy, "COR:"); //ips200_show_float(x + 11 * zoomx, y + 4 * zoomy, Elec_cor, 2, 3);
    }
    return(2+4)*zoomy+y;
}
/**
 * @brief               显示电机1PID相关数据
 * @parameter x         坐标x方向的起点
 * @parameter y         坐标y方向的起点
 * @parameter zoomx     x方向间隔
 * @parameter zoomy     y方向间隔
 * @return uint8        下一模块的起点y坐标
 */
uint8 Show_PID_Motor1(uint8 x,uint8 y,uint8 zoomx,uint8 zoomy){
    if(SHOW_SCREEN == SCREEN_TFT180){
        tft180_show_string(x + 5 * zoomx, y, "Motor1");
        if (Motor_pidStatus == 1)        tft180_show_string(x + 11 * zoomx, y, "(ON)");
        else if (Motor_pidStatus == 0)   tft180_show_string(x + 11 * zoomx, y, "(OFF)");
        tft180_show_string(x, y + 2 * zoomy, "P:"); tft180_show_float(x + 2 * zoomx, y + 2 * zoomy, Motor_1Puse, 3, 3);
        tft180_show_string(x + 8 * zoomx, y + 2 * zoomy, "I:"); tft180_show_float(x + 10 * zoomx, y + 2 * zoomy, Motor_1I, 3, 3);
        tft180_show_string(x, y + 4 * zoomy, "D:"); tft180_show_float(x + 2 * zoomx, y + 4 * zoomy, Motor_1D, 3, 3);
        tft180_show_string(x + 7 * zoomx, y + 4 * zoomy, "COR:"); tft180_show_float(x + 11 * zoomx, y + 4 * zoomy, Motor_1cor, 4, 1);
        return (2+4)*zoomy+y;
    }
    else if(SHOW_SCREEN == SCREEN_IPS200){
        ips200_show_string(x + 5 * zoomx, y , "Motor1");
        if(Motor_pidStatus == 1)        ips200_show_string(x + 9 * zoomx, y , "(ON)");
        else if(Motor_pidStatus == 0)   ips200_show_string(x + 9 * zoomx, y , "(OFF)");
        ips200_show_string(x, y + 2 * zoomy, "P:"); ips200_show_float(x + 2 * zoomx, y + 2 * zoomy, Motor_1P, 3, 3);
        ips200_show_string(x + 8 * zoomx, y + 2 * zoomy, "I:"); ips200_show_float(x + 10 * zoomx, y + 2 * zoomy, Motor_1I, 3, 3);
        ips200_show_string(x, y + 4 * zoomy, "D:"); ips200_show_float(x + 2 * zoomx, y + 4 * zoomy, Motor_1D, 3, 3);
        ips200_show_string(x + 7 * zoomx, y + 4 * zoomy, "COR:"); ips200_show_float(x + 10 * zoomx, y + 4 * zoomy, Motor_1cor, 5, 1);
        ips200_show_string(x, y + 6 * zoomy, "P_U:"); ips200_show_float(x + 4 * zoomx, y + 6 * zoomy, Motor_1Puse, 3, 3);
        return (2+6)*zoomy+y;
    }

}
/**
 * @brief               显示电机2PID相关数据
 * @parameter x         坐标x方向的起点
 * @parameter y         坐标y方向的起点
 * @parameter zoomx     x方向间隔
 * @parameter zoomy     y方向间隔
 * @return uint8        下一模块的起点y坐标
 */
uint8 Show_PID_Motor2(uint8 x, uint8 y, uint8 zoomx, uint8 zoomy) {
    if (SHOW_SCREEN == SCREEN_TFT180) {
        tft180_show_string(x + 5 * zoomx, y, "Motor2");
        if (Motor_pidStatus == 1)        tft180_show_string(x + 11 * zoomx, y, "(ON)");
        else if (Motor_pidStatus == 0)   tft180_show_string(x + 11 * zoomx, y, "(OFF)");
        tft180_show_string(x, y + 2 * zoomy, "P:"); tft180_show_float(x + 2 * zoomx, y + 2 * zoomy, Motor_2Puse, 3, 3);
        tft180_show_string(x + 8 * zoomx, y + 2 * zoomy, "I:"); tft180_show_float(x + 10 * zoomx, y + 2 * zoomy, Motor_2I, 3, 3);
        tft180_show_string(x, y + 4 * zoomy, "D:"); tft180_show_float(x + 2 * zoomx, y + 4 * zoomy, Motor_2D, 3, 3);
        tft180_show_string(x + 7 * zoomx, y + 4 * zoomy, "COR:"); tft180_show_float(x + 11 * zoomx, y + 4 * zoomy, Motor_2cor, 4, 1);
        return (2+4)*zoomy+y;
    }
    else if (SHOW_SCREEN == SCREEN_IPS200) {
        ips200_show_string(x + 5 * zoomx, y, "Motor2");
        if (Motor_pidStatus == 1)        ips200_show_string(x + 9 * zoomx, y, "(ON)");
        else if (Motor_pidStatus == 0)   ips200_show_string(x + 9 * zoomx, y, "(OFF)");
        ips200_show_string(x, y + 2 * zoomy, "P:"); ips200_show_float(x + 2 * zoomx, y + 2 * zoomy, Motor_2P, 3, 3);
        ips200_show_string(x + 8 * zoomx, y + 2 * zoomy, "I:"); ips200_show_float(x + 10 * zoomx, y + 2 * zoomy, Motor_2I, 3, 3);
        ips200_show_string(x, y + 4 * zoomy, "D:"); ips200_show_float(x + 2 * zoomx, y + 4 * zoomy, Motor_2D, 3, 3);
        ips200_show_string(x + 7 * zoomx, y + 4 * zoomy, "COR:"); ips200_show_float(x + 10 * zoomx, y + 4 * zoomy, Motor_2cor, 5, 1);
        ips200_show_string(x, y + 6 * zoomy, "P_U:"); ips200_show_float(x + 4 * zoomx, y + 6 * zoomy, Motor_2Puse, 3, 3);
        return (2 + 6) * zoomy + y;
    }
}
