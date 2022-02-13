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

/*************蓝牙参数配置**********************/
BluetoothSerial SerialBT;
#define Master 1    //主从机模式选择 1主机 0从机
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
#define PWML1 35
#define PWML2 34
#define PWMR1 33
#define PWMR2 32
#define MOTOR 26
#define MOTOL 25
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
#define NEXTSTAT    0xff 
/**********************************/


/************控制函数宏定义**************/
//#define MOTOSTOPR() {digitalWrite(PWMR1,LOW);digitalWrite(PWMR2,LOW);}

/**************************************/


/************PID参数****************/
#define SPD_P_DATA    50.0f // 比例常数 Proportional Const
#define SPD_I_DATA    0.00f  // 积分常数  Integral Const    --动态响应
#define SPD_D_DATA    0.00f // 微分常数 Derivative Const    --预测作用，增大阻尼
#define TARGET_SPEED  0
#define LIMIT         200      //积分限幅
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

PIDs pid_ang;

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

    if((iError<0.1f )&&(iError>-0.1f))           //偏差滤波
    iError = 0.0f;

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


void moto_init(){
  pinMode(PWMR1,OUTPUT);
  pinMode(PWMR2,OUTPUT);
}

void moto_pwm_init(){
  ledcSetup(8, 1000, 10);  //设置LEDC通道8频率为1000，分辨率为10位，即占空比可选0~1023
  ledcAttachPin(PWMR1, 8); //设置LEDC通道8在IO上输出
  //ledcWrite(8, 77); //设置输出PWM占空比,90°
  ledcSetup(7, 1000, 10);  
  ledcAttachPin(PWMR2, 7); 

  ledcSetup(6, 1000, 10);  
  ledcAttachPin(PWML1, 6); 
  ledcSetup(5, 1000, 10);  
  ledcAttachPin(PWML2, 5); 

}

void moto_pwm_set(uint8_t moto, int pwm){
  if(moto==LEFT){
    if(pwm<0){
      ledcWrite(5, -pwm); //设置输出PWM占空比
      ledcWrite(6, 0);
    }
    else{
      ledcWrite(6, pwm); //设置输出PWM占空比
      ledcWrite(5, 0);
    }
  }
  else{
    if(pwm<0){
      ledcWrite(7, -pwm); //设置输出PWM占空比
      ledcWrite(8, 0);
    }
    else{
      ledcWrite(7, 0);
      ledcWrite(8, pwm); //设置输出PWM占空比
    }
  }
}



void setup() {
  Serial.begin(115200);
  moto_pwm_init();
  //mpu6050
  Wire.begin(13,14 ,400000);// (sda,scl)Set I2C frequency to 400kHz
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  /*
  //bluetooth initiate
  SerialBT.register_callback(Bluetooth_Event); //设置事件回调函数 连接 断开 发送 接收
  SerialBT.begin("ESP32_MASTER",true); //开启蓝牙 名称为:"ESP32_MASTER" 主机
  Serial.printf("Init Successful - Master\r\n");
  SerialBT.connect(address);
  Serial.printf("Connect Successful\r\n");
  */

  PID_ParamInit(&pid_ang);
  moto_pwm_set(RIGHT,0);

/***************** 编码器初始化 *****************/
  pinMode(MOTOR, INPUT);    pinMode(MOTOL, INPUT);   
  attachInterrupt(MOTOR, right_counter_encoder, RISING);//设置编码器R上升沿中断
  attachInterrupt(MOTOL, left_counter_encoder, RISING);//设置编码器L上升沿中断      
    
/***************** 定时中断 *****************/   
  timer1.attach_ms(interrupt_time, timerIsr);  // 打开定时器中断
  interrupts();                      //打开外部中断  
}

float getAngle=0.0;
float setSpeed=0.0;

/********************     LOOP   *****************/
void loop() {

  if (SerialBT.available()) { //蓝牙接收，每次收一个数据
    btReceive(SerialBT.read());
  }

  if(Serial.available()){
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
      break;
    case FORWARD:
      moto_pwm_set(RIGHT,200);
      break;
    case BACK:
      moto_pwm_set(RIGHT,-200);
      break;
    case BALLANCE:
      mpu6050.update();
      getAngle=mpu6050.getAngleY();
      setSpeed+=SpdPIDCalc(&pid_ang,getAngle);
      //moto_pwm_set(MOTOR,(int)setSpeed);
      if(timer_flag==1){
        timer_flag=0;
        Serial.println(Rcountbuff);
      }
      break;
    default: break;
  }


  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  //motor.move(target_velocity);
  //mpu6050.update();
  //getAngle=mpu6050.getAngleX();
  //sendFloatBT(getAngle);
  //setSpeed+=SpdPIDCalc(&pid_ang,getAngle);  //增量式pid

}


/*******************    LOOP   ******************/

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
            if((i_buff_M>=4)&&((recBuffBT_M[i_buff_M-4]=='P')||(recBuffBT_M[i_buff_M-4]=='I')||(recBuffBT_M[i_buff_M-4]=='D'))){
              switch(recBuffBT_M[i_buff_M-4]){    //pid参数传递
                case 'P':
                  pid_ang.Proportion = (float)((recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  break;
                case 'I':
                  pid_ang.Integral = (float)((recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  break;
                case 'D':
                  pid_ang.Derivative = (float)((recBuffBT_M[i_buff_M-3]-'0')+(recBuffBT_M[i_buff_M-1]-'0')*0.1+(recBuffBT_M[i_buff_M]-'0')*0.01);
                  break;
              }
              sendStringBT("Set successful!\r\n");
              sendStringBT(recBuffBT_M);
                i_buff_M=0;
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
        case 0x00:          //选择舵机序号
            //servo_num=receive_A1[i_rec-4];  
            //digitalWrite(2,HIGH);
            //Serial.write(servo_num);
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



