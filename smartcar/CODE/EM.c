#include "headfile.h"
#include <math.h>
int16 adc[8];    //ADC滤波值
float ADC_Final[8]; //ADC归一化值
//float ADC_ALL,Gap_Val_Across,Sum_Val_Across,Gap_val_Vertical,Sum_Val_Vertical,Gap_val_Vertical_absolute;
float ADC_ALL=0;
int AD_MinValue[8] = {150,150,300,150,300,300,150 ,150};//七路电感最小值
int AD_MaxValue[8] = {2000,2000,3800,2000,4400,3800,2000,2000}; //七路电感最大值
float ADC_L,ADC_LM,ADC_M,ADC_RM,ADC_R,ADC_LD,ADC_RD,ADC_MD;

void EM_adc()
{
	 
	  uint16 i,j;
    int filter_temp;
	  int Left[10],Left_M[10],Middle_L[10],Right[10],Right_M[10],Middle_R[10],Middle[10], Middle_H[10];
	  static int adc_last[8]={0};
		
		
		
    for(i=0;i<10;i++)
    {      
   			Left[i]=adc_once(ADC_P11, ADC_12BIT);//采集10次求平均  分辨率12位
			  Left_M[i]=adc_once(ADC_P13, ADC_12BIT);//采集10次求平均  分辨率12位
        Middle_L[i]=adc_once(ADC_P01, ADC_12BIT);	//采集10次求平均  分辨率12位
			  Middle_H[i]=adc_once(ADC_P00, ADC_12BIT);//采集10次求平均  分辨率12位
			  //Middle[i]=adc_once(ADC_P15, ADC_12BIT);//采集10次求平均  分辨率12位
        Middle_R[i]=adc_once(ADC_P14, ADC_12BIT);//采集10次求平均  分辨率12位
			  Right_M[i]=adc_once(ADC_P17, ADC_12BIT);//采集10次求平均  分辨率12位
			  Right[i]=adc_once(ADC_P16, ADC_12BIT);	//采集10次求平均  分辨率12位
    }

    // 采样值从小到大排列（冒泡法）
		
		
    //左横
    for(j = 0; j < 10 - 1; j++)
    {
        for(i = 0; i < 10 - 1 - j; i++)
        {
            if(Left[i] > Left[i + 1])
            {
                filter_temp = Left[i];
                Left[i] = Left[i + 1];
                Left[i + 1] = filter_temp;
            }
        }
    }
    //左竖
    for(j = 0; j < 10 - 1; j++)
    {
        for(i = 0; i < 10 - 1 - j; i++)
        {
            if(Left_M[i] > Left_M[i + 1])
            {
                filter_temp = Left_M[i];
                Left_M[i] = Left_M[i + 1];
                Left_M[i + 1] = filter_temp;
            }
        }
    }
    //左中
    for(j = 0; j < 10 - 1; j++)
    {
        for(i = 0; i < 10 - 1 - j; i++)
        {
            if(Middle_L[i] > Middle_L[i + 1])
            {
                filter_temp = Middle_L[i];
                Middle_L[i] = Middle_L[i + 1];
                Middle_L[i + 1] = filter_temp;
            }
        }
    }
		//中
//		 for(j = 0; j < 10 - 1; j++)
//     {
//        for(i = 0; i < 10 - 1 - j; i++)
//        {
//            if(Middle[i] > Middle[i + 1])
//            {
//                filter_temp = Middle[i];
//                Middle[i] = Middle[i + 1];
//                Middle[i + 1] = filter_temp;
//            }
//        }
//    }
			//中
		 for(j = 0; j < 10 - 1; j++)
     {
        for(i = 0; i < 10 - 1 - j; i++)
        {
            if(Middle_H[i] > Middle_H[i + 1])
            {
                filter_temp = Middle_H[i];
                Middle_H[i] = Middle_H[i + 1];
                Middle_H[i + 1] = filter_temp;
            }
        }
    }
    //右中
    for(j = 0; j < 10 - 1; j++)
    {
        for(i = 0; i < 10 - 1 - j; i++)
        {
            if(Middle_R[i] > Middle_R[i + 1])
            {
                filter_temp = Middle_R[i];
                Middle_R[i] = Middle_R[i + 1];
                Middle_R[i + 1] = filter_temp;
            }
        }
    }
    //右竖
    for(j = 0; j < 10 - 1; j++)
    {
        for(i = 0; i < 10 - 1 - j; i++)
        {
            if(Right_M[i] > Right_M[i + 1])
            {
                filter_temp = Right_M[i];
                Right_M[i] = Right_M[i + 1];
                Right_M[i + 1] = filter_temp;
            }
        }
    }
    //右横
    for(j = 0; j < 10 - 1; j++)
    {
        for(i = 0; i < 10 - 1 - j; i++)
        {
            if(Right[i] > Right[i + 1])
            {
                filter_temp = Right[i];
                Right[i] = Right[i + 1];
                Right[i + 1] = filter_temp;
            }
        }
    }
	
   /////////////////////////////冒泡排序完成///////////////////////////////////////////////

    // 去除最大最小极值后求平均
    for(i = 1; i < 10 - 1; i++)
    {
      adc[0] += Left[i];
      adc[1] += Left_M[i];
      adc[2] += Middle_L[i];
			adc[3] += Middle_H[i];
		  adc[4] += Middle[i];
      adc[5] += Middle_R[i];
      adc[6] += Right_M[i];
      adc[7] += Right[i];
    }
    adc[0] = adc[0] /8;
    adc[1] = adc[1] /8;
    adc[2] = adc[2] /8;
    adc[3] = adc[3] /8;
    adc[4] = adc[4] /8;
    adc[5] = adc[5] /8;
		adc[6] = adc[6] /8;
		adc[7] = adc[7] /8;
		if(adc[2]>4000)adc[2]=4000;
		if(adc[5]>4000)adc[5]=4000;
		if(adc[2]<0)adc[2]=adc_last[2];
		if(adc[5]<0)adc[5]=adc_last[5];
		adc_last[2]=adc[2];
		adc_last[5]=adc[5];
}
	void ADC_Normalization1(int16 AD_Sensor[])
{
    float AD_Normalized[8];
		uint8 i;
    for(i=0;i<8;i++)
    {      
			AD_Normalized[i]=(float)(AD_Sensor[i]-AD_MinValue[i])/(float)(AD_MaxValue[i]-AD_MinValue[i]);
			if(AD_Normalized[i]<=0.0)
			{
				AD_Normalized[i]=0.001;
			}
			if(AD_Normalized[i]>1.0)
			{
				AD_Normalized[i]=1.0;
			}
			AD_Normalized[i]=100 * AD_Normalized[i];
			ADC_Final[i]=AD_Normalized[i];
    }		
		ADC_L=ADC_Final[0];
		ADC_LM=ADC_Final[1];
		ADC_LD=ADC_Final[2];
		ADC_M=ADC_Final[3];
		ADC_MD=ADC_Final[4];
		ADC_RD=ADC_Final[5];
		ADC_RM=ADC_Final[6];
		ADC_R=ADC_Final[7];
		
//		Gap_Val_Across=ADC_L-ADC_R;
//		Sum_Val_Across=ADC_L+ADC_R;
//		Gap_val_Vertical=ADC_LM-ADC_RM;
//		Sum_Val_Vertical=ADC_LM+ADC_RM;
//		Gap_val_Vertical_absolute=abs(Gap_val_Vertical);
		ADC_ALL=ADC_L+ADC_R+ADC_M;
			
}
double  AD_M_Right=0,AD_M_Lift=0;
double  RightAverage=0,LeftAverage=0;
double  distance,distance_L,distance_R;
int16   RightADC[20],LeftADC[20]; 

void infrared_detection()
{  
   int i;
   int Right_Sum=0,Left_Sum=0;
   for(i=0;i<20;i++)
    {   
        LeftADC[i]=adc_once(ADC_P06, ADC_12BIT);   
        Left_Sum+=LeftADC[i];
        RightADC[i]=adc_once(ADC_P05, ADC_12BIT);  
        Right_Sum+=RightADC[i];
    }
		LeftAverage=Left_Sum/20;
    RightAverage=Right_Sum/20; 
		AD_M_Lift =LeftAverage*3.30/4096;
		AD_M_Right=RightAverage*3.30/4096;
 
 distance_L=26.481*pow(AD_M_Lift,(-1.05));   
 distance_R=26.481*pow(AD_M_Right,(-1.05));   
 
 if(distance_L>150.00) distance_L=150.00;        
 if(distance_L<20.00) distance_L=20.00;
 if(distance_R>150.00) distance_R=150.00;        
 if(distance_R<20.00) distance_R=20.00;
		
		distance=(distance_L+distance_R)*0.5;
//			if(distance_L-distance_R>5)distance=distance_R;
//				if(distance_R-distance_L>5)distance=distance_L;
				

}