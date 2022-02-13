/*
 * pid.h
 *功能：pid控制函数，各个参数版本供参考
 *  Created on: 2021年7月2日
 *      Author: 12249
 */

#ifndef PID_H_
#define PID_H_

#define uint32_t unsigned int
#define int32_t int
#define uint16_t unsigned short

/**************************/
//PID参数设置
/*
//bios系数为52，适合低速状态
#define SPD_P_DATA 1.08f // 比例常数 Proportional Const
#define SPD_I_DATA 0.4f  // 积分常数  Integral Const    --动态响应
#define SPD_D_DATA 0.0f// 微分常数 Derivative Const    --预测作用，增大阻尼
#define TARGET_SPEED 0
#define LIMIT   1.2f      //与预计速度的调节最大值限制
*/

//bios系数为1--平地行驶
#define SPD_P_DATA 100.0f // 比例常数 Proportional Const
#define SPD_I_DATA 35.0f  // 积分常数  Integral Const    --动态响应
#define SPD_D_DATA 2.0f // 微分常数 Derivative Const    --预测作用，增大阻尼
#define TARGET_SPEED 0
#define LIMIT   800      //积分限幅

/***************************/

typedef struct
{
    double SumError; //误差累计
    double Proportion; //比例常数Proportional Const
    double Integral; //积分常数 IntegralConst
    double Derivative; //微分常数Derivative Const
    float SetPoint; //设定目标 DesiredValue
    float LastError; //Error[-1]
    float PrevError; //Error[-2]
    int lastCCR;
    int limit;      //调节CCR限幅

}PIDs;


/*
//整数运算版pid
//bios系数为1--平地行驶
#define SPD_P_DATA 100 // 比例常数 Proportional Const
#define SPD_I_DATA 35  // 积分常数  Integral Const    --动态响应
#define SPD_D_DATA 2 // 微分常数 Derivative Const    --预测作用，增大阻尼
#define TARGET_SPEED 0
#define LIMIT   800      //积分限幅

typedef struct
{
    long SumError; //误差累计
    uint32_t Proportion; //比例常数Proportional Const
    uint32_t Integral; //积分常数 IntegralConst
    uint32_t Derivative; //微分常数Derivative Const
    uint32_t SetPoint; //设定目标 DesiredValue
    int LastError; //Error[-1]
    int PrevError; //Error[-2]
    int lastCCR;
    int limit;      //调节CCR限幅

}PIDs;
*/


void pid_init();                //左右速度环初始化

void PID_speed_setL(int speedrpm);   //设定目标转速（转/秒）
void PID_speed_setR(int speedrpm);   //设定目标转速（转/秒）

float SpdPIDCalc(PIDs *pid,float NextPoint);    //速度闭环PID控制设计，

float PID_speed_prosL();
float PID_speed_prosR();


/*short PID_Pos_PosCalc(short NextPoint);

short PID_Ang_PosCalc(short NextPoint);
*/

#endif /* PID_H_ */
