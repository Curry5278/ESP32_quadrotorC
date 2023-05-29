#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include "esp32-hal-cpu.h"
#include <Wire.h>
#include <WiFi.h>

// 超音波設置
int const trigPin= 12;//13
int const echoPin= 14;//12
double Duration;
double Distance;
// --------------------------------------------------
// const char* ssid = "5B";
// const char* password = "44396726";
// const char* ssid = "林威廷的iphone";
// const char* password = "12345678";
const char* ssid = "lulu";
const char* password = "89110423";//1104

int value ;
int WifiTryCount =0;
int state = 0 ; //記錄收到訊息後的狀態
WiFiServer server(80);
int Position[9];
double PID_var[9];
double Sliding_var[9];
String NewString ;
String data , dataMat ;
int number = 0;
bool clientConnect = false ;
Servo esc1,esc2,esc3,esc4;

unsigned long t1 = 0;
unsigned long t2 = 0;                  
float hz = 0;                                                            
Adafruit_MPU6050 mpu;
// #include <MPU6050_light.h>
// MPU6050 mpu1(Wire);
int TCPLED = 0 ; //確認收TCP訊息
int throttleInputPin3 = 19; // 接收油門值19
int throttleInputPin1 = 18; // 接收滾轉值
int throttleInputPin2 = 23; // 接收俯仰值
int throttleInputPin4 = 15; // 接收偏航值

int motor_lf_throttle = 5; //12
int motor_rf_throttle = 17; //14
int motor_lb_throttle = 16; //23 27
int motor_rb_throttle = 4; //15 26

int throttleEmergencypin1 = 13;
int throttleEmergency ;
int throttleInputE;
int throttleMin = 1000; // 最小油門值
int throttleMax = 2000; // 最大油門值

int throttleInput3; // 接收到的3油門输入值
int throttleOutput3; // 输出到3電變的油門值

int throttleInput1; // 接收到的3油門输入值
int throttleOutput1; // 输出到3電變的油門值
int throttleInput2; // 接收到的3油門输入值
int throttleOutput2; // 输出到3電變的油門值
int throttleInput4; // 接收到的3油門输入值
int throttleOutput4; // 输出到3電變的油門值
int elapsedTime, timePrev; 
unsigned long prevTime ;     
unsigned long curTime ;

unsigned long start1,start2 ;
float time_HZ ;

float roll_angle, pitch_angle , yaw_angle;
float roll_desired_angle , pitch_desired_angle ,yaw_desired_angle , altitude_desired;             
float roll_error , roll_previous_error, pitch_error , pitch_previous_error , yaw_error , yaw_previous_error , altitude_error , altitude_previous_error ; 

double altitude_kp = 10; 
double altitude_kd = 1.0; 
double altitude_pid_p , altitude_pid_d = 0;

float roll_previous_angle , pitch_previous_angle , yaw_previous_angle ;
float Ix = 0.05;
float Iy = Ix ;
float Iz = 0.24 ; 
float Sign_C = 20 ; //符號函數可調參數

double S_roll , S_pitch , S_yaw ;
double roll_error_dot , pitch_error_dot , yaw_error_dot ;

double roll_C = 5; //6
double roll_E = 1; //1
double roll_r = 1; //1.5
double roll_Sliding = 0;
 
double pitch_C = roll_C; 
double pitch_E = roll_E; 
double pitch_r = roll_r; 
double pitch_Sliding = 0;

double yaw_C = 5; //6
double yaw_E = 1; //2
double yaw_r = 1; //2
double yaw_Sliding = 0;

volatile bool stopState = false ;
volatile unsigned long lastTime1,lastTime2,lastTime3,lastTime4,lastTimeE;
boolean interruptState1,interruptState2,interruptState3,interruptState4,interruptStateE;

void IRAM_ATTR rc3Interrupt(){
    interruptState3 = digitalRead(throttleInputPin3);
    if (interruptState3){
        lastTime3 = micros();
    }else{
        throttleInput3 = micros() - lastTime3;
    }
}
void IRAM_ATTR rc1Interrupt(){
    interruptState1 = digitalRead(throttleInputPin1);
    if (interruptState1){
      lastTime1 = micros();
    }else{
      throttleInput1 = micros() - lastTime1;
    }
}
void IRAM_ATTR rc2Interrupt(){
    interruptState2 = digitalRead(throttleInputPin2);
    if (interruptState2){
      lastTime2 = micros();
    }else{
      throttleInput2 = micros() - lastTime2;
    }  
}
void IRAM_ATTR rc4Interrupt(){
    interruptState4 = digitalRead(throttleInputPin4);
    if (interruptState4){
      lastTime4 = micros();
    }else{
      throttleInput4 = micros() - lastTime4;
    }  
}

void IRAM_ATTR emergencyLand(){
  interruptStateE = digitalRead(throttleEmergencypin1);
  if (interruptStateE){
    lastTimeE = micros();
  }
  else{
    throttleEmergency = micros() - lastTimeE;
  }
  
  if(throttleEmergency < 1120)
  {
    stopState = true ;
    detachInterrupt(throttleInputPin3);
    motor_lf_throttle = 0; 
    motor_rf_throttle = 0; 
    motor_lb_throttle = 0; 
    motor_rb_throttle = 0; 
    esc1.writeMicroseconds(0);
    esc2.writeMicroseconds(0);
    esc3.writeMicroseconds(0);
    esc4.writeMicroseconds(0);  
  }
  else
  {
    attachInterrupt(digitalPinToInterrupt(throttleInputPin3), rc3Interrupt, CHANGE);
    stopState = false ;
  }
}

void setup(void) {
  Serial.begin(115200); // 不要print那麼多值試試看loop Hz
  setCpuFrequencyMhz(240); // 80 16~27ms , 240 13~24
  Serial.print("CpuFrequency: ");Serial.println(getCpuFrequencyMhz());
  WIFI_Connect();
  IMU();

  Serial.println("");
  delay(100);

  esc1.attach(motor_lf_throttle, 1000, 2000);  
  esc2.attach(motor_rf_throttle, 1000, 2000);
  esc3.attach(motor_lb_throttle, 1000, 2000);
  esc4.attach(motor_rb_throttle, 1000, 2000);
  esc1.writeMicroseconds(1000);  //將PWM信號設置為最小值
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(100);  
  esc1.writeMicroseconds(2000);  //將PWM信號設置為最大值
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  esc4.writeMicroseconds(2000);
  delay(100);
  esc1.writeMicroseconds(1500);  //將PWM信號設置為中間值
  esc2.writeMicroseconds(1500);
  esc3.writeMicroseconds(1500);
  esc4.writeMicroseconds(1500);
  delay(100);

  pinMode(throttleInputPin3, INPUT);
  attachInterrupt(digitalPinToInterrupt(throttleInputPin3), rc3Interrupt, CHANGE);

  pinMode(throttleInputPin1, INPUT);
  attachInterrupt(digitalPinToInterrupt(throttleInputPin1), rc1Interrupt, CHANGE);
  pinMode(throttleInputPin2, INPUT);
  attachInterrupt(digitalPinToInterrupt(throttleInputPin2), rc2Interrupt, CHANGE);
  pinMode(throttleInputPin4, INPUT);
  attachInterrupt(digitalPinToInterrupt(throttleInputPin4), rc4Interrupt, CHANGE);
  pinMode(throttleEmergencypin1, INPUT);
  attachInterrupt(digitalPinToInterrupt(throttleEmergencypin1), emergencyLand, CHANGE);
}

void loop() {
  // 333 500 HZ
  // start1 = start2;  
  // start2 = millis();  
  // time_HZ = (start2 - start1) ;
  WiFiClient client = server.available();  // 等待客戶端連接
  if (client) {
    Serial.println("Client connected.");
    // clientConnect = true ;
    while (client.connected()) {
      // String data = "Hello from server.";  // 欲傳送的訊息
      clientConnect = true ;
      Serial.println("new client");
      if(client.available()) {
        state = 1;
        data = client.readStringUntil('\n');
        Serial.println(data);
        client.write("Message received.");
    }
      angle();
      Slidingcontroller();
      
      SlidingRead_TCP();
      Sliding_Change();
      // a=String(roll_angle);b=String(pitch_angle) ;c=String(yaw_angle);
      number++;
      dataMat = String(number)+","+String(start2)+","+String(throttleInput1)+","+String(throttleInput2)+","+String(throttleInput3)+","+String(throttleInput4)\
      +","+String(roll_desired_angle)+","+String(pitch_desired_angle)+","+String(yaw_desired_angle)+","+String(roll_angle)+","+String(pitch_angle)+","+String(yaw_angle)\
      +","+String(roll_error)+","+String(pitch_error)+","+String(yaw_error);

      Serial.println(dataMat); //電腦觀看
      client.println(dataMat);  // 傳送訊息給客戶端
      if(stopState == false){
      motor_lf_throttle = constrain(motor_lf_throttle, 1000, 2000);
      motor_rf_throttle = constrain(motor_rf_throttle, 1000, 2000);
      motor_lb_throttle = constrain(motor_lb_throttle, 1000, 2000);
      motor_rb_throttle = constrain(motor_rb_throttle, 1000, 2000);

      esc1.writeMicroseconds(motor_lf_throttle);
      esc2.writeMicroseconds(motor_rf_throttle);
      esc3.writeMicroseconds(motor_lb_throttle);
      esc4.writeMicroseconds(motor_rb_throttle);
  }
      if(!client.connected()){
        clientConnect = false ;
        break;
      }
    }
  }  
  if (clientConnect == false){
    Serial.println(stopState);
    angle();
    Slidingcontroller();
    Sliding_Change();
  }

  Serial.print("滾動值: ");Serial.print(throttleInput1);
  Serial.print("俯仰值: ");Serial.print(throttleInput2);
  Serial.print("油門值: ");Serial.print(throttleInput3);
  Serial.print("偏航值: ");Serial.print(throttleInput4);
  Serial.print("Rotation X: ");
  Serial.print(roll_angle);
  Serial.print("Rotation Y: ");
  Serial.print(pitch_angle);
  Serial.print("Rotation Z: ");
  Serial.print(yaw_angle);
  Serial.print("time: ");Serial.println(time_HZ);

  if(stopState == false){
    motor_lf_throttle = constrain(motor_lf_throttle, 1000, 2000);
    motor_rf_throttle = constrain(motor_rf_throttle, 1000, 2000);
    motor_lb_throttle = constrain(motor_lb_throttle, 1000, 2000);
    motor_rb_throttle = constrain(motor_rb_throttle, 1000, 2000);

    esc1.writeMicroseconds(motor_lf_throttle);
    esc2.writeMicroseconds(motor_rf_throttle);
    esc3.writeMicroseconds(motor_lb_throttle);
    esc4.writeMicroseconds(motor_rb_throttle);
  }
}


void angle()
{
  start2 = millis();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  roll_angle = g.gyro.x*180/(3.14);
  pitch_angle = g.gyro.y*180/(3.14);
  yaw_angle = g.gyro.z*180/(3.14);
}

float mapValue(int value, int input_min, int input_max, int output_min, int output_max) { 
  return (value - input_min) * (output_max - output_min) / (input_max - input_min) + output_min; 
} 

void Slidingcontroller()
{
  // -------------- roll --------------------------
  if (throttleInput1 > 1479 && throttleInput1 < 1484) {
    roll_desired_angle = 0 ;
  }
  else {
    if(throttleInput1 > 1480) // 右滾
    {
    roll_desired_angle = throttleInput1 - 1480;
    }
    else if(throttleInput1 < 1480) // 左滾
    {
      roll_desired_angle = throttleInput1 - 1480;
    }
  roll_desired_angle /= 3.0;
  }
  roll_error = roll_desired_angle - roll_angle ;
  curTime = millis();
  elapsedTime = curTime - prevTime;

// (roll_error - roll_previous_error)/elapsedTime);
  roll_error_dot = ((roll_error - roll_previous_error)/elapsedTime) ;
  S_roll = (roll_C*roll_error) + roll_error_dot ;
  
  roll_Sliding = Ix*(-roll_C * ((roll_angle - roll_previous_angle) / elapsedTime) + roll_E*SIGN(S_roll) + roll_r * S_roll) + \
  ((pitch_angle - pitch_previous_angle) / elapsedTime) * ((yaw_angle - yaw_previous_angle) / elapsedTime) * (Iz-Iy) ;
  // Serial.print("roll_desir:");Serial.println(roll_desired_angle);Serial.print(" ");
  // Serial.print("roll_error:");Serial.println(roll_error);Serial.print(" ");
  Serial.print("roll_Sliding:");Serial.println(roll_Sliding);Serial.print(" ");
  if(roll_Sliding < -400){roll_Sliding=-400; }
  if(roll_Sliding > 400) {roll_Sliding= 400; }

  // -------------- pitch --------------------------
  if (throttleInput2 > 1472 && throttleInput2 < 1480) {
    pitch_desired_angle = 0 ;
  }
  else {
    if(throttleInput2 < 1472) // 
    {
      pitch_desired_angle = 1472 - throttleInput2;
    }
    else if(throttleInput1 > 1480) // 
    {
      pitch_desired_angle = 1480 - throttleInput2;
    }
  pitch_desired_angle /= 3.0;
  }

  pitch_error = pitch_desired_angle - pitch_angle ;
  curTime = millis();
  elapsedTime = curTime - prevTime;

  pitch_error_dot = ((pitch_error - pitch_previous_error)/elapsedTime) ;
  S_pitch = (pitch_C*pitch_error) + pitch_error_dot ;
  
  pitch_Sliding = Iy*(-pitch_C * ((pitch_angle - pitch_previous_angle) / elapsedTime) + pitch_E*SIGN(S_pitch) + pitch_r * S_pitch) + \
  ((roll_angle - roll_previous_angle) / elapsedTime) * ((yaw_angle - yaw_previous_angle) / elapsedTime) * (Ix-Iz) ;

  // Serial.print("pitch_desir:");Serial.println(pitch_desired_angle);Serial.print(" ");
  // Serial.print("pitch_error:");Serial.println(pitch_error);Serial.print(" ");
  Serial.print("pitch_Sliding:");Serial.println(pitch_Sliding);Serial.print(" ");
  if(pitch_Sliding < -400){pitch_Sliding=-400; }
  if(pitch_Sliding > 400) {pitch_Sliding= 400; }
  // -------------- yaw --------------------------
  if (throttleInput4 > 1492 && throttleInput4 < 1502) {
    yaw_desired_angle = 0 ;
  }
  else {
    if(throttleInput4 > 1500)
    {
      yaw_desired_angle = 1500-throttleInput4;
    }
    else if(throttleInput4 < 1500)
    {
      yaw_desired_angle = 1500-throttleInput4;
    }
  yaw_desired_angle /= 2.0;
  }
  yaw_error = yaw_desired_angle - yaw_angle ;
  curTime = millis();
  elapsedTime = curTime - prevTime;

  yaw_error_dot = ((yaw_error - yaw_previous_error)/elapsedTime) ;
  S_yaw = (yaw_C*yaw_error) + yaw_error_dot ;
  
  yaw_Sliding = Iy*(-yaw_C * ((yaw_angle - yaw_previous_angle) / elapsedTime) + yaw_E*SIGN(S_yaw) + yaw_r * S_yaw) + \
  ((roll_angle - roll_previous_angle) / elapsedTime) * ((pitch_angle - pitch_previous_angle) / elapsedTime) * (Iy-Ix) ;

  // Serial.print("yaw_desir:");Serial.println(yaw_desired_angle);Serial.print(" ");
  // Serial.print("yaw_error:");Serial.println(yaw_error);Serial.print(" ");
  Serial.print("yaw_Sliding:");Serial.println(yaw_Sliding);Serial.print(" ");
  if(yaw_Sliding < -400){yaw_Sliding=-400; }
  if(yaw_Sliding > 400) {yaw_Sliding= 400; }


  if(throttleInput3 < 1110){
    roll_Sliding=0;
    pitch_Sliding=0;
    yaw_Sliding=0;
  }
  
  motor_lf_throttle = throttleInput3 + roll_Sliding - pitch_Sliding + yaw_Sliding ;
  motor_rf_throttle = throttleInput3 - roll_Sliding - pitch_Sliding - yaw_Sliding ;
  motor_lb_throttle = throttleInput3 + roll_Sliding + pitch_Sliding - yaw_Sliding ;
  motor_rb_throttle = throttleInput3 - roll_Sliding + pitch_Sliding + yaw_Sliding ;
  
  prevTime = millis();

  roll_previous_error = roll_error ; 
  pitch_previous_error = pitch_error;
  yaw_previous_error = yaw_error;

  roll_previous_angle = roll_angle ;
  pitch_previous_angle = pitch_angle ;
  yaw_previous_angle = yaw_angle ;

}

void altitudeController(){
  digitalWrite(trigPin,HIGH); //發射超音波
  delayMicroseconds(3);
  digitalWrite(trigPin,LOW);
  Duration = pulseIn(echoPin,HIGH); //超音波發射到接收的時間
  // Distance = Duration*0.034/2; //計算距離(cm) 在常溫 20 度時, 音速是 331.5+0.6*20=343.5 m/s
  Distance = Duration*0.034*0.01/2 ; //計算距離(m)
  // Serial.println(Distance);
  altitude_desired = mapValue(throttleInput3,1100,2000,0,10) ;
  altitude_error = altitude_desired - Distance ;

  curTime = millis();
  elapsedTime = curTime - prevTime;

  altitude_pid_p = altitude_kp * altitude_error ;
  altitude_pid_d = altitude_kd * ((altitude_error - altitude_previous_error)/elapsedTime);
  
  throttleInput3 = throttleInput3 + altitude_pid_p + altitude_pid_d ;
  // Serial.print("altitude_error:");Serial.println(altitude_error);Serial.print(" ");
  // Serial.print("throttleInput3:");Serial.println(throttleInput3);Serial.print(" ");
  // motor_lf_throttle = throttleInput3 ;
  // motor_rf_throttle = throttleInput3 ;
  // motor_lb_throttle = throttleInput3 ;
  // motor_rb_throttle = throttleInput3 ;
  // esc1.writeMicroseconds(motor_lf_throttle);
  // esc2.writeMicroseconds(motor_rf_throttle);
  // esc3.writeMicroseconds(motor_lb_throttle);
  // esc4.writeMicroseconds(motor_rb_throttle);
}

void WIFI_Connect(){
  // 連接到 Wi-Fi 網路
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    WifiTryCount++;
    // 判斷連接10次若無wifi就退出
    if (WifiTryCount > 5){
      break ;
    }
  }

  // 啟動 TCP/IP 伺服器Server端
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  server.begin();
  Serial.println("Server started");
}

void TCP_MatlabClient(){
  WiFiClient client = server.available(); // 監聽客戶端連線
  if (client) {
    Serial.println("New client connected");
    
    // 傳送訊息給MATLAB client
    // if (client.connected()) {
      // client.println("1,2");
    // }
  }
  value++;
  client.println(value);
}

void SlidingRead_TCP(){
    // WiFiClient client = server.available();
  // if (!client) {
  //   return;
  // }

  // 讀取客戶端傳送的資料
  // while (client.connected()) {
    // client.println("1,2"); // 傳送訊息給 client
    // Serial.println("new client");
    // // if(client.available() ) {
    //   state = 1;
    //   data = client.readStringUntil('\n');
    //   Serial.println(data);
      digitalWrite(TCPLED, HIGH);
      Position[0]=data.indexOf(',');//找出第一個逗點的位置並存放在Position1
    for(int n=0;n<8;n++)
    {
      Position[n+1] = data.indexOf(',',Position[n]+1);
    }
     NewString=data.substring(0,Position[0]); //0~1可顯示0的數字
     Sliding_var[0]=NewString.toDouble();
     NewString=data.substring(Position[0]+1,Position[1]); //2~3可顯示1的數字
     Sliding_var[1]=NewString.toDouble();
     NewString=data.substring(Position[1]+1,Position[2]);
     Sliding_var[2]=NewString.toDouble();
     NewString=data.substring(Position[2]+1,Position[3]);
     Sliding_var[3]=NewString.toDouble();
     NewString=data.substring(Position[3]+1,Position[4]);
     Sliding_var[4]=NewString.toDouble();
     NewString=data.substring(Position[4]+1,Position[5]);
     Sliding_var[5]=NewString.toDouble();
     NewString=data.substring(Position[5]+1,Position[6]);
     Sliding_var[6]=NewString.toDouble();
     NewString=data.substring(Position[6]+1,Position[7]);
     Sliding_var[7]=NewString.toDouble();
     NewString=data.substring(Position[7]+1,Position[8]);
     Sliding_var[8]=NewString.toDouble();
    
    //  for(int i =0 ; i<9 ; i++){
    //    Serial.println(Sliding_var[i]);
    //  }
    //  Serial.println(Sliding_var[8]);
  //     break;
  //   }
  // }
  // 接收完訊息斷開客戶端連接
  // client.println("1,2"); // 傳送訊息給 client
  digitalWrite(TCPLED, LOW);
  // client.stop();
  
}

void Sliding_Change(){
  // for(int i =0 ; i<9 ; i++){
  //   Serial.println(PID_var[i]);
  // }
  if(state == 1){
    roll_C = Sliding_var[0] ;
    roll_E = Sliding_var[1] ;
    roll_r = Sliding_var[2] ;
    pitch_C = roll_C; 
    pitch_E = roll_E; 
    pitch_r = roll_r; 
    yaw_C = Sliding_var[6] ;
    yaw_E = Sliding_var[7] ;
    yaw_r = Sliding_var[8] ;
    state=0;
  }
  else{
    roll_C = roll_C ;
    roll_E = roll_E ;
    roll_r = roll_r ;
    pitch_C = roll_C; 
    pitch_E = roll_E; 
    pitch_r = roll_r; 
    yaw_C = yaw_C ;
    yaw_E = yaw_E ;
    yaw_r = yaw_r ;
  }
  Serial.println(state);
  Serial.println(roll_C,3);
  Serial.println(roll_E,6);
  Serial.println(roll_r,3);
  Serial.println(pitch_C,3);
  Serial.println(pitch_E,6);
  Serial.println(pitch_r,3);
  Serial.println(yaw_C,3);
  Serial.println(yaw_E,3);
  Serial.println(yaw_r,3);
}

float SIGN(float X)
{
  return  ( X / (abs(X)+ Sign_C)); // 係數調低有過大震顫現象 ??
}

void IMU(){
  // while (!Serial)
  //   delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  // pinMode(TCPLED, OUTPUT); // 將 ledPin 設置為輸出模式
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}