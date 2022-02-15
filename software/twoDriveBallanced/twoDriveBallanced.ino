// Open loop motor control example
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <String.h>
#include <Ticker.h>

MPU6050 mpu6050(Wire);

/***************** 定时中断参数 *****************/
Ticker timer1;  // 中断函数
int interrupt_time = 10; // 中断时间
int timer_flag=0;               //定时器标志；
 
/***************** 编码器引脚及参数 *****************/
volatile long Rcounter=0; // 右轮脉冲计数  该变量用于存储编码器的值，所以用类型修饰符volatile；
volatile long Lcounter=0; // 左轮脉冲计数  该变量用于存储编码器的值，所以用类型修饰符volatile；
long  Rcountbuff=0;
long  Lcountbuff=0;

int direcL=1;   //电机方向标志
int direcR=1;   //电机方向标志

/*************蓝牙参数配置**********************/
BluetoothSerial SerialBT;
#define Master 0    //主从机模式选择 1主机 0从机
void Bluetooth_Event(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);  //蓝牙事件回调函数
void sendFloatBT(float data);   //蓝牙发送浮点数
void sendStringBT(char* sdata);  //蓝牙发送字符串
void btReceive(int rec);  //蓝牙接收信息处理
uint8_t address[6]={0x78,0x21,0x84,0x7D,0xA6,0xEA}; //从机MAC地址 不同的蓝牙地址不同 需要自己修改

char recBuffBT_M[100];
int i_buff_M=0;
int i_rec=0;
char receive_A1[20];

uint16_t control_flag=0;

/************电机参数宏**************/
#define PWML1 13
#define PWML2 14
#define PWMR1 33
#define PWMR2 32
#define RCODE 26
#define LCODE 25
/**********************************/

/***********控制信息宏定义*************/
#define STOP        0x00
#define START       0x01
#define LEFT        0x02
#define RIGHT       0x03
#define SPEEDUP     0x04
#define SPEEDDOWN   0x05
#define FORWARD     0x06
#define BACK        0x07
#define BALLANCE    0x08
#define CHANGE      0x09
#define NEXTSTAT    0xff 
/**********************************/


/************控制函数宏定义**************/
//#define MOTOSTOPR() {digitalWrite(PWMR1,LOW);digitalWrite(PWMR2,LOW);}

/**************************************/


/************PID参数****************/
//角度环
float ANG_P_DATA = 500.00f; // 比例常数 Proportional Const
float ANG_I_DATA = 5.00f;  // 积分常数  Integral Const    --动态响应
float ANG_D_DATA = 0.00f; // 微分常数 Derivative Const    --预测作用，增大阻尼
#define TARGET_ANGLE  -1.50   //需测试角度机械零点
#define LIMIT         1000      //积分限幅

//速度环
float SPD_P_DATA = 20.00f; // 比例常数 Proportional Const
float SPD_I_DATA = 2.00f;  // 积分常数  Integral Const    --动态响应
float SPD_D_DATA = 0.00f; // 微分常数 Derivative Const    --预测作用，增大阻尼
#define TARGET_SPEED  0   //设定目标点初值

/***************************/

typedef struct
{
    float SumError; //误差累计
    float Proportion; //比例常数Proportional Const
    float Integral; //积分常数 IntegralConst
    float Derivative; //微分常数Derivative Const
    float SetPoint; //设定目标 DesiredValue
    float LastError; //Error[-1]
    float PrevError; //Error[-2]
    int lastCCR;
    int limit;      //调节CCR限幅

}PIDs;

PIDs pid_ang, pid_spdL, pid_spdR;

void PID_ANG_ParamInit(PIDs *pid)
{
    pid->LastError = 0.0;               // Error[-1]
    pid->PrevError = 0.0;               // Error[-2]
    pid->Proportion = ANG_P_DATA; // 比例常数 Proportional Const
    pid->Integral = ANG_I_DATA;   // 积分常数  Integral Const
    pid->Derivative = ANG_D_DATA; // 微分常数 Derivative Const
    pid->SetPoint = TARGET_ANGLE;     // 设定目标Desired Value
    pid->lastCCR=0;
}

void PID_SPD_ParamInit(PIDs *pid)
{
    pid->LastError = 0.0;               // Error[-1]
    pid->PrevError = 0.0;               // Error[-2]
    pid->Proportion = SPD_P_DATA; // 比例常数 Proportional Const
    pid->Integral = SPD_I_DATA;   // 积分常数  Integral Const
    pid->Derivative = SPD_D_DATA; // 微分常数 Derivative Const
    pid->SetPoint = TARGET_SPEED;     // 设定目标Desired Value
    pid->lastCCR=0;

}

void PID_SetSpeed(PIDs *pid,int speed)  //设置速度，实际为编码数量（0--20）
{
  pid->SetPoint = speed;
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

    /*
    if((iError<0.1f )&&(iError>-0.1f))           //偏差滤波
      iError = 0.0f;
    */

    pid->SumError+=iError;
    InteError=pid->Integral * iError;
    if((pid->SumError>LIMIT)||(pid->SumError<-LIMIT))           //积分项限幅，减小超调
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


void moto_init(){  //方向测试用，电机初始化
  pinMode(PWMR1,OUTPUT);
  pinMode(PWMR2,OUTPUT);
  pinMode(PWML1,OUTPUT);
  pinMode(PWML2,OUTPUT);
  //digitalWrite(PWML1,LOW);
  //digitalWrite(PWML2,HIGH);
  //digitalWrite(PWMR1,HIGH);
  //digitalWrite(PWMR2,LOW);
}

void moto_pwm_init(){
  ledcSetup(8, 1000, 10);  //设置LEDC通道8频率为1000，分辨率为10位，即占空比可选0~1023
  ledcAttachPin(PWMR1, 8); //设置LEDC通道8在IO上输出
  ledcSetup(9, 1000, 10);  
  ledcAttachPin(PWMR2, 9); 

  ledcSetup(10, 1000, 10);  
  ledcAttachPin(PWML1, 10); 
  ledcSetup(11, 1000, 10);  
  ledcAttachPin(PWML2, 11); 

}

void moto_pwm_set(uint8_t moto, int pwm){
  if(moto==LEFT){
    if(pwm<0){
      direcL=-1;
      ledcWrite(10, -pwm); //设置输出PWM占空比
      ledcWrite(11, 0);
    }
    else{
      direcL=1;
      ledcWrite(11, pwm); //设置输出PWM占空比
      ledcWrite(10, 0);
    }
  }
  else{
    if(pwm<0){
      direcR=-1;
      ledcWrite(9, -pwm); //设置输出PWM占空比
      ledcWrite(8, 0);
    }
    else{
      direcR=1;
      ledcWrite(9, 0);
      ledcWrite(8, pwm); //设置输出PWM占空比
    }
  }
}


void right_counter_encoder();
void left_counter_encoder();
void readEncoder();
void timerIsr();


void setup() {
  Serial.begin(115200);
  SerialBT.register_callback(Bluetooth_Event); //设置事件回调函数 连接 断开 发送 接收
  SerialBT.begin("Leansbot"); //Bluetooth device name    
  Serial.println("The device started, now you can pair it with bluetooth!");

  //mpu6050
  Wire.begin(18,19 ,400000);// (sda,scl)Set I2C frequency to 400kHz
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  moto_pwm_init();
  //moto_init();
  PID_ANG_ParamInit(&pid_ang);
  PID_SPD_ParamInit(&pid_spdL);
  PID_SPD_ParamInit(&pid_spdR);
  moto_pwm_set(RIGHT,0);
  moto_pwm_set(LEFT,0);
  PID_SetSpeed(&pid_spdL,0);
  PID_SetSpeed(&pid_spdR,0);

  /***************** 编码器初始化 *****************/
  pinMode(RCODE, INPUT);    pinMode(LCODE, INPUT);   
  attachInterrupt(RCODE, right_counter_encoder, RISING);//设置编码器R上升沿中断
  attachInterrupt(LCODE, left_counter_encoder, RISING);//设置编码器L上升沿中断      
    
  /***************** 定时中断 *****************/   
  timer1.attach_ms(interrupt_time, timerIsr);  // 打开定时器中断
  interrupts();                      //打开外部中断  
}

float getAngle=0.0;
float setSpeed=0.0;
float setSpeedL=0.0;
float setSpeedR=0.0;
int tnow=0;


/********************     LOOP   *****************/
void loop() {

  if (Serial.available()) { //串口接收控制
    switch(Serial.read()){
      case 'S': control_flag=STOP;break;
      case 'F': control_flag=FORWARD;break;
      case 'B': control_flag=BACK;break;
      case 'M': control_flag=BALLANCE;break;
      default:break;
    }
  }
  
  switch(control_flag){
    case STOP:
      moto_pwm_set(RIGHT,0);
      moto_pwm_set(LEFT,0);
      break;
    case FORWARD:
      moto_pwm_set(RIGHT,1000);
      moto_pwm_set(LEFT,1000);
      break;
    case BACK:
      moto_pwm_set(RIGHT,-400);
      moto_pwm_set(LEFT,-400);
      break;
    case BALLANCE:
      if(timer_flag==1){
        timer_flag=0;
        /*
        Serial.print("\r\n");
        Serial.print("R:");
        Serial.println(Rcountbuff);
        Serial.print("\r\n");
        Serial.print("L:");
        Serial.println(Lcountbuff);
        */
        mpu6050.update();
        getAngle=mpu6050.getAngleX();
        
        
        //setSpeedL = SpdPIDCalc(&pid_spdL, (direcL*Lcountbuff)+(direcR*Rcountbuff));
        //pid_ang.SetPoint=SpdPIDCalc(&pid_spdL, (direcL*Lcountbuff)+(direcR*Rcountbuff));
        //Serial.println(direcL*Lcountbuff);
        //Serial.println(direcR*Rcountbuff);
        //setSpeedR += SpdPIDCalc(&pid_spdR, (direcR*Rcountbuff));
        setSpeed+=SpdPIDCalc(&pid_ang,getAngle);
        moto_pwm_set(RIGHT,(int)(setSpeed));
        moto_pwm_set(LEFT,(int)(setSpeed));

      }
      break;
    case CHANGE:
      PID_ANG_ParamInit(&pid_ang);
      setSpeed=0.0;
      sendStringBT("\r\n");
      SerialBT.write('P');
      sendFloatBT(pid_ang.Proportion);
      sendStringBT("\r\n");

      SerialBT.write('I');
      sendFloatBT(pid_ang.Integral);
      sendStringBT("\r\n");

      SerialBT.write('D');
      sendFloatBT(pid_ang.Derivative);
      sendStringBT("\r\n");
      control_flag=STOP;
      break;
    default: break;
  }
}


/*******************    蓝牙事件回调函数   ******************/

void Bluetooth_Event(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)  //蓝牙事件回调函数
{
    if(event == ESP_SPP_OPEN_EVT || event == ESP_SPP_SRV_OPEN_EVT) //蓝牙连接成功标志 
    {                                                              //蓝牙主机和从机模式对应的标志不同，前面的是主机模式的，后面是从机模式
        sendStringBT("connection successful!\r\n");
    }
    else if(event == ESP_SPP_CLOSE_EVT)     //蓝牙断开连接标志
    {
        sendStringBT("disconnect successful!\r\n");
    }
    else if(event == ESP_SPP_DATA_IND_EVT)  //数据接收标志
    {
        while(SerialBT.available())
        {
            recBuffBT_M[i_buff_M]=SerialBT.read();

            switch(recBuffBT_M[i_buff_M]){
                  case 'S': control_flag=STOP;break;
                  case 'F': control_flag=FORWARD;break;
                  case 'B': control_flag=BACK;break;
                  case 'M': control_flag=BALLANCE;break;
                  default:break;
                }

            if((i_buff_M>=7)&&((recBuffBT_M[i_buff_M-7]=='P')||(recBuffBT_M[i_buff_M-7]=='I')||(recBuffBT_M[i_buff_M-7]=='D'))){
              switch(recBuffBT_M[i_buff_M-7]){    //pid参数传递,"P1000.00"
                case 'P':
                  ANG_P_DATA = (float)((recBuffBT_M[i_buff_M-6]-'0')*1000.0+(recBuffBT_M[i_buff_M-5]-'0')*100.0+(recBuffBT_M[i_buff_M-4]-'0')*10.0+(recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  break;
                case 'I':
                  ANG_I_DATA = (float)((recBuffBT_M[i_buff_M-6]-'0')*1000.0+(recBuffBT_M[i_buff_M-5]-'0')*100.0+(recBuffBT_M[i_buff_M-4]-'0')*10.0+(recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  break;
                case 'D':
                  ANG_D_DATA = (float)((recBuffBT_M[i_buff_M-6]-'0')*1000.0+(recBuffBT_M[i_buff_M-5]-'0')*100.0+(recBuffBT_M[i_buff_M-4]-'0')*10.0+(recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  break;
              }
              sendStringBT("Set successful!\r\n");
              sendStringBT(recBuffBT_M);
              sendStringBT("\r\n");

              i_buff_M=0;
              control_flag=CHANGE;
          }
          else ++i_buff_M;
        }
    }
    else if(event == ESP_SPP_WRITE_EVT)     //数据发送标志
    {
        Serial.write("  send complete! \r\n");
    }
}

void sendFloatBT(float data){
	int i;
	char dstr[10];
	sprintf(dstr, "%.2f", data);
	for(i=0;i<strlen(dstr);i++){  
	SerialBT.write(dstr[i]);
  }
}

void sendStringBT(char *sdata){
	int i;
	for(i=0;i<strlen(sdata);i++){  
	SerialBT.write(sdata[i]);
  }
}

void btReceive(int rec){  //蓝牙接收信息处理
    receive_A1[i_rec]=rec;
    if((i_rec>5)&&(receive_A1[i_rec-6]==0x55)&&(receive_A1[i_rec]==0xff)&&(receive_A1[i_rec-1]==0xff)){
      switch(receive_A1[i_rec-5]){
        case 0x00:          
            //servo_num=receive_A1[i_rec-4];  
            //digitalWrite(2,HIGH);
            i_rec=0;
            break;
        case 0x01:          //角度控制开关
            //servo_change=receive_A1[i_rec-4];
            //Serial.write(servo_change);
            i_rec=0;
        default:break;
    }
  }
  else i_rec++;
}


//定时器中断处理函数,其功能主要为了输出编码器得到的数据
void timerIsr(){
   timer_flag=1;  //定时时间达到标志      
   readEncoder();   // 编码器
}

//编码器输出  
void readEncoder(){
  Rcountbuff=Rcounter;
  Lcountbuff=Lcounter;
//数值清零，重新计数
  Rcounter = 0;
  Lcounter = 0;
}


// 编码器计数，中断回调函数
void right_counter_encoder(){
    Rcounter++;
}
void left_counter_encoder(){  // //左轮计数
    Lcounter++;
}

