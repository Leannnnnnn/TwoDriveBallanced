/*
 * pid.c
 *
 *  Created on: 2021年7月2日
 *      Author: 12249
 */

#include "pid.h"


PIDs PID_L,PID_R;           //定义左右速度pid结构体

/******************** PID 控制设计 ***************************/
/**
  * 函数功能: PID参数初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void PID_ParamInit(PIDs *pid)
{
    pid->LastError = 0.0;               // Error[-1]
    pid->PrevError = 0.0;               // Error[-2]
    pid->Proportion = SPD_P_DATA; // 比例常数 Proportional Const
    pid->Integral = SPD_I_DATA;   // 积分常数  Integral Const
    pid->Derivative = SPD_D_DATA; // 微分常数 Derivative Const
    pid->SetPoint = TARGET_SPEED;     // 设定目标Desired Value
    pid->lastCCR=0;

}


void pid_init()
{
    PID_ParamInit(&PID_L);
    PID_ParamInit(&PID_R);
}

void PID_speed_setL(int speedrpm)   //设定目标转速（转/秒）
{
    PID_L.SetPoint=speedrpm;
    PID_L.lastCCR=speedrpm*50;
    PID_L.limit=PID_L.SetPoint*80;
}

void PID_speed_setR(int speedrpm)   //设定目标转速（转/秒）
{
    PID_R.SetPoint=speedrpm;
    PID_R.lastCCR=speedrpm*50;
    PID_R.limit=PID_R.SetPoint*80;
}

/**
  * 函数名称：速度闭环PID控制设计
  * 输入参数：当前控制量
  * 返 回 值：目标控制量
  * 说    明：无
  */
//浮点数运算
float SpdPIDCalc(PIDs *pid,float NextPoint)    //速度闭环PID控制设计--增量式pid
{
    register float iError,InteError,iIncpid;
    iError = (float)pid->SetPoint - NextPoint; //偏差

    if((iError<2.0f )&&(iError>-2.0f))           //偏差滤波
    iError = 0.0f;

    pid->SumError+=iError;
    InteError=pid->Integral * iError;
    if((pid->SumError>LIMIT)||(pid->SumError<-300))           //积分项限幅，减小超调
    {    InteError=0;
        pid->SumError-=iError;
    }

    //send_pidpoint("add 2,1,",(int)pid->SumError);

    //if(InteError>15.0) InteError=0.0;
    iIncpid = (pid->Proportion * (iError - pid->LastError) )                //E[k]项
              + (InteError)     //E[k-1]项
              + (pid->Derivative * (iError-2*pid->LastError+pid->PrevError));  //E[k-2]项

    pid->PrevError = pid->LastError;
    pid->LastError = iError;
    return iIncpid;
}



//整数运算版pid--加快处理速度
/*
long SpdPIDCalc(PIDs *pid,uint32_t NextPoint)    //速度闭环PID控制设计--增量式pid
{
    register uint32_t iError,InteError,iIncpid;
    iError = pid->SetPoint - NextPoint; //偏差

    if((iError<3 )&&(iError>-3))           //偏差滤波
    iError = 0;

    pid->SumError+=iError;
    InteError=pid->Integral * iError;
    if((pid->SumError>LIMIT)||(pid->SumError<-300))           //积分项限幅，减小超调
    {    InteError=0;
        pid->SumError-=iError;
    }

    //send_pidpoint("add 2,1,",(int)pid->SumError);

    //if(InteError>15.0) InteError=0.0;
    iIncpid = (pid->Proportion * (iError - pid->LastError) )                //E[k]项
              + (InteError)     //E[k-1]项
              + (pid->Derivative * (iError-2*pid->LastError+pid->PrevError));  //E[k-2]项

    pid->PrevError = pid->LastError;
    pid->LastError = iError;
    return iIncpid;
}
*/

/*
int32_t PosPIDCalc(PIDs *pid,float NextPoint)    //速度闭环PID控制设计--位置式pid
{
    register float iError,iIncpid;
    iError = (float)pid->SetPoint - NextPoint; //偏差

    if((iError<1.0f )&&(iError>-1.0f))           //偏差滤波
    iError = 0.0f;

    pid->SumError+=iError;
    //InteError=pid->Integral * iError;
    if((pid->SumError>1200)){           //积分项限幅，减小超调
        pid->SumError=1200;
    }
    if(pid->SumError<-1200){
        pid->SumError=-1200;
    }

    //if(InteError>15.0) InteError=0.0;
    iIncpid = (pid->Proportion * (iError) )                //E[k]项
              + (pid->Integral*pid->SumError)     //E[k-1]项
              + (pid->Derivative * (iError-pid->LastError));  //E[k-2]项

    pid->LastError = iError;
    return iIncpid;
}
*/

uint32_t flag_bios=0;
float PID_speed_prosL()//测量左侧轮速度并调用pid处理函数
{
    int32_t bios=0;
    long CCR=0;
    float speed=speed_filterL();
    bios=SpdPIDCalc(&PID_L,speed);
    flag_bios=bios;
    CCR=PID_L.lastCCR+bios;
    /*if(bios>2500){
        CCR=PID_L.lastCCR;
    }*/
    if(CCR<0){
        CCR=0;
    }
    /*if(CCR>6000){           //速度波动限制
        CCR=PID_L.limit;
    }*/
    if(PID_L.SetPoint<5)CCR=0;
    //TA0CCR3=CCR;
    PID_L.lastCCR=CCR;
    return speed;
    
}

float PID_speed_prosR()
{
    int32_t bios=0;
    long CCR=0;
    float speed;
    speed=speed_filterR();
    //speed=speed_getR();
    bios=SpdPIDCalc(&PID_R,speed);
    CCR=PID_R.lastCCR+bios;
    if(CCR<0){
        CCR=0;
    }
    /*
    if(CCR>6000){           //速度波动限制
        CCR=PID_R.limit;
    }*/
    if(PID_R.SetPoint<5)CCR=0;
    //TA0CCR4=CCR;
    PID_R.lastCCR=CCR;
    return speed;
}

/*short PID_Pos_PosCalc(short NextPoint)        //位置式pid
{
  register float  iError,dError;
    iError = pos_Set_Pos - NextPoint;                       // 偏差
    pos_SumError += iError*0.2;                                 // 积分       0.2表示积分时间常数的倒数，求时间平均
    if(pos_SumError > 1000.0)                               //积分限幅1000
        pos_SumError = 1000.0;
    if(pos_SumError < -1000.0)
        pos_SumError = -1000.0;
    dError = iError - pos_LastError;                        // 当前微分
    pos_LastError = iError;

    return(short)(pos_p * iError*0.8 + pos_i * pos_SumError + pos_d * dError);  //返回计算值
}

short PID_Ang_PosCalc(short NextPoint)
{
  register float  iError,dError;
    iError = ang_Set_Pos - NextPoint;                       // 偏差
    ang_SumError += iError;                                 // 积分
    if(ang_SumError > 1000.0)                               //积分限幅1000
        ang_SumError = 1000.0;
    if(ang_SumError < -1000.0)
        ang_SumError = -1000.0;
    dError = iError - ang_LastError;                        // 当前微分
    ang_LastError = iError;

    return(short)(ang_p * iError + ang_i * ang_SumError + ang_d * dError);  //返回计算值
}
*/
