#ifndef _EM_H_
#define _EM_H_

extern float ADC_Final[8]; //ADC归一化值
extern int16 adc[8];    //ADC滤波值
//extern float ADC_ALL,Gap_Val_Across,Sum_Val_Across,Gap_val_Vertical,Sum_Val_Vertical,Gap_val_Vertical_absolute;
extern float ADC_ALL;
extern float ADC_L,ADC_LM,ADC_M,ADC_RM,ADC_R,ADC_LD,ADC_RD,ADC_MD;
extern double  AD_M_Right,AD_M_Lift;
extern double  RightAverage,LeftAverage;
extern double  distance,distance_L,distance_R;
extern int16   RightADC[20],LeftADC[20]; //电压采集

void EM_adc(void);
void ADC_Normalization1(int16 AD_Sensor[]);//AD值归一化
void infrared_detection();
#endif