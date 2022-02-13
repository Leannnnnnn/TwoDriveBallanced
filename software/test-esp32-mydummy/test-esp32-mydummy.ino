/*六轴机械臂蓝牙远程控制
 * 
 * 说明文档：https://shimo.im/docs/VMAPVQvzMgCrXBqg/ 《ESP系列物联网项目筹备》
 * 见该文档后半部分
 * 
 * Leannnus
 * 
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <dummy.h>
#include <BluetoothSerial.h>
#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

BluetoothSerial SerialBT;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
int servo_num=0;
int servo_change=0;
int i_A1=0;
char receive_A1[20];
int angArray_const[10]={90,45,0,0,0,0,20,90,90,90}; //初始状态角度 序号0，1，2，4，5，6
int angArray[10];

void servo_test(){
  ledcSetup(8, 50, 10);  //设置LEDC通道8频率为50，分辨率为10位，即占空比可选0~1023
  ledcAttachPin(2, 8); //设置LEDC通道8在IO12上输出
  ledcWrite(8, 1024); //设置输出PWM占空比
}

void pwm16_setup(){
  Wire.begin(18,19); //sda,scl
  pwm.reset();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

void btReceive(int rec){  //蓝牙接收信息处理
    receive_A1[i_A1]=rec;
    if((i_A1>5)&&(receive_A1[i_A1-6]==0x55)&&(receive_A1[i_A1]==0xff)&&(receive_A1[i_A1-1]==0xff)){
      switch(receive_A1[i_A1-5]){
        case 0x00:          //选择舵机序号
            servo_num=receive_A1[i_A1-4];  
            //digitalWrite(2,HIGH);
            //Serial.write(servo_num);
            i_A1=0;
            break;
        case 0x01:          //角度控制开关
            servo_change=receive_A1[i_A1-4];
            //Serial.write(servo_change);
            i_A1=0;
        default:break;
    }
  }
  else i_A1++;
}

void setAngle(int n,int ang){
  pwm.setPWM(n,0,(int)2.275*ang+102.4);
}

void servo_init(){
  int i=0;
  for(i=0;i<10;i++){
    angArray[i]=angArray_const[i];
  }
  setAngle(0,angArray[0]);
  setAngle(1,angArray[1]);
  setAngle(2,angArray[2]);
  setAngle(4,angArray[4]);
  setAngle(5,angArray[5]);
  setAngle(6,angArray[6]);
}

void selectServo(int n){
  if((angArray[n]<180)&&(servo_change==1)){
      angArray[n]+=4;
      setAngle(n,angArray[n]);
      servo_change=0;
  }
  if((angArray[n]>0)&&(servo_change==3)){ 
      angArray[n]-=4;
      setAngle(n,angArray[n]);
      servo_change=0;
  }
  if(servo_change==2){
    servo_init();
  }
}


void setup() {
  Serial.begin(115200);
  SerialBT.begin("Leansdummy"); //Bluetooth device name    
  Serial.println("The device started, now you can pair it with bluetooth!");
  // run high latency network code in 2nd core instead in loop()
  pinMode(2,OUTPUT);
  //servo_test();
  pwm16_setup();
  servo_init();
  /*
  pwm.setPWM(0,0,307);//90°
  pwm.setPWM(1,0,307);//90°
  pwm.setPWM(2,0,307);//90°
  pwm.setPWM(4,0,307);//90°
  pwm.setPWM(5,0,307);//90°
  pwm.setPWM(6,0,200);//90°*/

}

void loop() {
  
  /*
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  */
  if (SerialBT.available()) { //蓝牙接收，每次收一个数据
    btReceive(SerialBT.read());
  }
  if(servo_change) 
    selectServo(servo_num);
}
