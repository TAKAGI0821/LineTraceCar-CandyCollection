#include <Servo.h>

//アナログピン
const int leftSensorPin = A7;
const int rightSensorPin = A1;
const int middleSensorPin = A2;
const int R = A4; //横の線読み取るよう
const int L = A5; //横の線読み取るよう
const int RS = A6;//赤外線センサー

//センサーの変数初期化
int leftSensorValue = 0;
int rightSensorValue = 0;
int middleSensorValue = 0;
int Right = 0;
int Left = 0;
int Rsensor;
float Rsensor1;

//サーボモータピン
const int servoPin = 7; //車輪向き変更
const int armServoPin_buttom = 3; 
const int armServoPin_above = 4; 
Servo servo,servoButtom,servoAbove;//サーボモータのオブジェクト
int servoAngle;

//モータの制御ピン
const int IN1 = 5;
const int IN2 = 6;

//フラグ
int box_flag = 0;
int keep_left = 0;

// 比例ゲインパラメータ
float Kp = 1.8;  
float error = 0;  

//サーボモータの範囲
const int servoMin = 90;
const int servoMax = 180;

//センサーの閾値
int white = 600; //白線を読み取ったときの値
int white_box = 550; //白線を読み取ったときの値(箱取るとき)

//その他変数の初期化
int speed;
int InitialAngle = 141;//車輪用
int stoptime = 200; //停止した時
int times0 = 3;//(左と右を読む間) 
int times = 14;//外のループ(右の線を読んだとき)
int times1 = 80; //中のループ
float controlSignal=0;//変更角度
int angle = 130;


void threshold(){
  if(middleSensorValue > white && rightSensorValue > white && leftSensorValue > white){
    error = 0; //真ん中
  }else if(rightSensorValue > white && middleSensorValue > white){
    error = 3.5; //真ん中と右
  }else if(rightSensorValue > white){
    error = 10; //右
  }else if(leftSensorValue > white && middleSensorValue > white){
    error = -3.5; //真ん中と左
  }else if(leftSensorValue > white){
    error = -10; //左
  }else if(error > 0 && middleSensorValue < white && rightSensorValue < white && leftSensorValue < white) {
    error = 15;
  }else if(error < 0 && middleSensorValue < white && rightSensorValue < white && leftSensorValue < white) {
    error = -15;
  }
}

void sensor(){
  leftSensorValue = analogRead(leftSensorPin);
  rightSensorValue = analogRead(rightSensorPin);
  middleSensorValue = analogRead(middleSensorPin);
  Right = analogRead(R);
  Left = analogRead(L);
  Rsensor = analogRead(RS);
  Rsensor1 = 5.0 * Rsensor/1024;
}

void get(){ //箱をつかむ
  if(box_flag == 0 && Left > white_box && Rsensor1 > 1.8){
    analogWrite(IN1,0);
    delay(500);
    servoAbove.write(108);
    delay(3000);
    for(int angle = 130; angle > 0; angle--){
      servoButtom.write(angle);
      delay(10);
    }
    speed = 200; //速度を少し上げる
    box_flag = 1; //箱を持っている状態にする
  }
}

void left_loop(){
 if(box_flag == 1 && Left > white_box){
  analogWrite(IN1,0);
  delay(500);
  keep_left = 1;
  for(int i = 0; i <= times0; i++){  //1.8s(3回)の間白線(左)読み取りを維持
    analogWrite(IN1,speed);
    for(int j = 0; j <= times1; j++){
      sensor();
      threshold();

      //比例制御
      controlSignal = Kp * error;
      servoAngle = InitialAngle + controlSignal;
      if(servoAngle < servoMin) servoAngle = servoMin;
      if(servoAngle > servoMax) servoAngle = servoMax;
      servo.write(servoAngle);

      remove(); //右の白線を読んだら放す
      if(keep_left == 0) break;
      delay(1);
    }
    if(keep_left==0) break;
    analogWrite(IN1,0);
    delay(stoptime);
  }
  keep_left = 0;
 }
}


void remove(){  //右の白線を読んだら放す
  if(Right > white_box && keep_left == 1){
    analogWrite(IN1,0);
    delay(500);
    for(int i = 0; i <= times; i++){  //1.8s(3回)の間少し進ませる
      analogWrite(IN1,speed);
      for(int j = 0; j <= times1; j++){
        sensor();
        threshold();
        //比例制御
        controlSignal = Kp * error;
        servoAngle = InitialAngle + controlSignal;
        if(servoAngle < servoMin) servoAngle = servoMin;
        if(servoAngle > servoMax) servoAngle = servoMax;
        servo.write(servoAngle);

        delay(1);
      }
      analogWrite(IN1,0);
      delay(stoptime);
    }
    analogWrite(IN1,0);  //止まって箱を放す
    servoAbove.write(0);
    delay(3000);
    servoButtom.write(130);
    box_flag = 0;
    keep_left = 0;
    speed = 185;  //スピードを下げる
    reset(); //少しの間白線に反応しないようにする
  }
}

void reset(){   //少しの間白線に反応しないようにする
  for(int i = 0; i < times; i++){
    analogWrite(IN1,speed);
    for(int j = 0; j < times1; j++){
      sensor();//センサーの値読み取り
      threshold();//ライントレース用の閾値設定

      //比例制御
      controlSignal = Kp * error;
      servoAngle = InitialAngle + controlSignal;
      if(servoAngle < servoMin) servoAngle = servoMin;
      if(servoAngle > servoMax) servoAngle = servoMax;
      servo.write(servoAngle);

      delay(1);
    }
    analogWrite(IN1,0);
    delay(stoptime);
  }
}





void setup() {

  //サーボモータの初期化
  servo.attach(servoPin);
  servoButtom.attach(armServoPin_buttom);
  servoAbove.attach(armServoPin_above);
  servo.write(InitialAngle); //車輪用
  servoButtom.write(angle);  //110(アーム下用)
  servoAbove.write(0);  //0(アーム上用)
  
  //モータピン設定
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  //センサーの初期化
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  pinMode(middleSensorPin, INPUT);
  pinMode(R, INPUT);
  pinMode(L, INPUT);

  // シリアルモニタの初期化（デバッグ用）
  Serial.begin(9600);

  speed = 200;
}

void loop(){
  analogWrite(IN1,speed);
  for(int i = 0; i <= times1; i++){
    sensor();//センサーの値読み取り
    threshold();//ライントレース用の閾値設定
    get();//箱を取る
    left_loop();//左のセンサーを読んだとき（ただし、箱を持っているときのみ動作）

    //比例制御
    controlSignal = Kp * error;
    servoAngle = InitialAngle + controlSignal;
    if(servoAngle < servoMin) servoAngle = servoMin;
    if(servoAngle > servoMax) servoAngle = servoMax;
    servo.write(servoAngle);

    delay(1);
  }
  analogWrite(IN1,0);
  delay(stoptime);



}






