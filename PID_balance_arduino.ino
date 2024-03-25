#include "Wire.h"


//Motor board connections and PWM
int enA = 3, in1 = 2, in2 = 4;
int enB = 5, in3 = 7, in4 = 8;
int enC = 6, in5 = 10, in6 = 11;
int enD = 9, in7 = 12, in8 = 13;
int gyro_error = 0, acc_error = 0, i = 0, j = 0;

const int MPU = 0x68; //Address for the gyroscope

float Total_angleX, Total_angleY;
float rad_to_deg = 180/3.141592654;
float elapsedtime, timer, timePrev;
float gyro_rawX, gyro_rawY, rawXg_error, rawYg_error;
float gyroX_angle, gyroY_angle, accX_angle, accY_angle;
float acc_rawX, acc_rawY, acc_rawZ, rawXa_error, rawYa_error, rawZa_error;
float Xangle_error, Yangle_error;
float LF, LR, RF, RR;

float roll_desired_angle = 0, roll_PID, roll_error, roll_previous_error, roll_pid_p = 0;
float roll_pid_i = 0, roll_pid_d = 0;
float roll_kp = 0.7, roll_ki = 0.006, roll_kd = 0.2;
float pitch_desired_angle = 0, pitch_PID, pitch_error, pitch_previous_error, pitch_pid_p = 0;
float pitch_pid_i = 0, pitch_pid_d = 0; 
float pitch_kp = 0.7, pitch_ki = 0.006, pitch_kd = 0.2;  

/*char tmp_str[7];
char* int16_to_str(int16_t i){
  sprintf(tmp_str, "%d", i);
  return tmp_str;
}*/

void setup() {
  
  Serial.begin(9600); //Begins the serial for the arduino
  Wire.begin(); //Begins the transmission line for the Gyroscope
  Wire.beginTransmission(MPU); //Connects the gyroscope to the slave
  Wire.write(0x6B); //PWR_MGMT_1 register
  Wire.write(0); //Wakes up the gyroscope
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);
  
  pinMode(enA, OUTPUT); 
  pinMode(in1, OUTPUT); 
  pinMode(in2, OUTPUT); //Sets all of the motor pins to outputs
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT); 
  pinMode(in4, OUTPUT);
  pinMode(enC, OUTPUT); 
  pinMode(in5, OUTPUT); 
  pinMode(in6, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);
  
  //Clockwise direction for dc motor this allows the motors to go up and down allowing double speed
  digitalWrite(in1, HIGH); // Front left DOWN
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); // Back left DOWN
  digitalWrite(in4, HIGH);
  analogWrite(enA, 255); 
  analogWrite(enB, 255);
  digitalWrite(in5, HIGH); // Front right DOWN
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH); // Back right DOWN
  digitalWrite(in8, LOW);
  analogWrite(enC, 255);
  analogWrite(enD, 255);
  delay(12000); //Due to only being a motor this is a value that is completely deterministic
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); 
  digitalWrite(in4, LOW);
  analogWrite(enA, 255); 
  analogWrite(enB, 255);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(enC, 255);
  analogWrite(enD, 255);
  delay(6000); //Due to only being a motor this is a value that is completely deterministic
  digitalWrite(in1, LOW); 
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); 
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW); 
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW); 
  digitalWrite(in8, LOW);
  
  if(gyro_error == 0){
    for(i = 0; i < 200; i++){
      Wire.beginTransmission(MPU);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 4, true);

      gyro_rawX = Wire.read() <<8 | Wire.read();
      gyro_rawY = Wire.read() <<8 | Wire.read();

      rawXg_error = rawXg_error + (gyro_rawX/32.8);
      rawYg_error = rawYg_error + (gyro_rawY/32.8);

      if(i == 199){
        rawXg_error = rawXg_error / 200;
        rawYg_error = rawYg_error / 200;

        gyro_error = 1;
      }
    }
  }

  if(acc_error == 0){
    for(j = 0; j < 200; j++){
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true);

      acc_rawX=(Wire.read() <<8 | Wire.read()); //each value needs two registres
      acc_rawY=(Wire.read() <<8 | Wire.read());
      acc_rawZ=(Wire.read() <<8 | Wire.read());
      acc_rawX = map(acc_rawX, -17000, 17000, -125, 125);
      acc_rawY = map(acc_rawY, -17000, 17000, -125, 125);
      acc_rawZ = map(acc_rawZ, -17000, 17000, -125, 125);

      /*---X---*/
      Xangle_error = Xangle_error + ((atan((acc_rawY)/sqrt(pow((acc_rawX),2) + pow((acc_rawZ),2)))*rad_to_deg));
      /*---Y---*/
      Yangle_error = Yangle_error + ((atan(-1*(acc_rawX)/sqrt(pow((acc_rawY),2) + pow((acc_rawZ),2)))*rad_to_deg));

      if(j == 199){
        Xangle_error = Xangle_error / 200;
        Yangle_error = Yangle_error / 200;

        acc_error = 1;
      }
    }
  }
}

void loop() {
 
  
  analogWrite(enA, 255), (enB, 255), (enC, 255), (enD, 255);

  timePrev = timer;
  timer = millis();
  elapsedtime = (timer - timePrev) / 1000;
  
  Wire.beginTransmission(MPU);
  Wire.write(0x43); //Register for ACCEL_XOUT_H from the MPU6050 datasheet
  Wire.endTransmission(false); //keeps the connection active
  Wire.requestFrom(MPU, 4, true); // requests a total of 14 registers

  gyro_rawX = Wire.read()<<8 | Wire.read(); // Reading from GYRO_XOUT_H and GYRO_XOUT_L
  gyro_rawX = Wire.read()<<8 | Wire.read(); // Reading from GYRO_YOUT_H and 

  gyro_rawX = (gyro_rawX / 32.8) - rawXg_error;
  gyro_rawY = (gyro_rawY / 32.8) - rawYg_error;

  gyroX_angle = gyro_rawX * elapsedtime;
  gyroY_angle = gyro_rawY * elapsedtime;

  Wire.beginTransmission(MPU);
  Wire.write(0X3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  acc_rawX = Wire.read() <<8 | Wire.read();
  acc_rawY = Wire.read() <<8 | Wire.read();
  acc_rawZ = Wire.read() <<8 | Wire.read();
  
  acc_rawX = map(acc_rawX, -17000, 17000, -125, 125);
  acc_rawY = map(acc_rawY, -17000, 17000, -125, 125);
  acc_rawZ = map(acc_rawZ, -17000, 17000, -125, 125);

  
  accX_angle = (atan((acc_rawY)/sqrt(pow((acc_rawX),2) + pow((acc_rawZ),2)))*rad_to_deg) - Xangle_error;
  accY_angle = (atan(-1*(acc_rawX)/sqrt(pow((acc_rawY),2) + pow((acc_rawZ),2)))*rad_to_deg) - Yangle_error;
  Total_angleX = accX_angle;
  Total_angleY = accY_angle;
  if((-5 < Total_angleX) && (Total_angleX < 5)){
    Total_angleX = 0;
  }
  if((-5 < Total_angleY) && (Total_angleY < 5)){
    Total_angleY = 0;
  }
  Serial.println(Total_angleX);
  Serial.println(Total_angleY);
  roll_desired_angle = 0;
  pitch_desired_angle = 0;

  roll_error = Total_angleY - roll_desired_angle;
  pitch_error = Total_angleX - pitch_desired_angle;

  roll_pid_p = roll_kp * roll_error;
  pitch_pid_p = pitch_kp * pitch_error;

  if(-3 < roll_error < 3){
    roll_pid_i = roll_pid_i + (roll_ki * roll_error);
  }
  if(-3 < pitch_error < 3){
    pitch_pid_i = pitch_pid_i + (pitch_ki * pitch_error);
  }

  roll_pid_d = roll_kd * ((roll_error - roll_previous_error) / elapsedtime);
  pitch_pid_d = pitch_kd * ((pitch_error - pitch_previous_error) / elapsedtime);

  roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
  pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;

  if(roll_PID < -150){
    roll_PID = -150;
  }
  if(roll_PID > 150){
    roll_PID = 150;
  }
  if(pitch_PID < -150){
    pitch_PID = -150;
  }
  if(pitch_PID > 150){
    pitch_PID = 150;
  }

  RF = - roll_PID - pitch_PID;
  LF = roll_PID + pitch_PID;
  RR = roll_PID - pitch_PID;
  LR = - roll_PID + pitch_PID;
   
  if(LF > 255){
    LF = 255;
  }
  if(LF < -255){
    LF = -255;
  }
  if(LR > 255){
    LR = 255;
  }
  if(LR < -255){
    LR = -255;
  }
  if(LF > 255){
    LF = 255;
  }
  if(RF < -255){
    RF = -255;
  }
  if(RR > 255){
    RR = 255;
  }
  if(RR < -255){
    RR = -255;
  }
  
  if((Total_angleX == 0) &&(Total_angleY == 0)){
    LF = 0;
    LR = 0;
    RR = 0;
    RF = 0;
    analogWrite(enA,0);
    analogWrite(enB,0);
    analogWrite(enC,0);
    analogWrite(enD,0);
    digitalWrite(in1, LOW); 
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); 
    digitalWrite(in4, LOW);
    digitalWrite(in5, LOW); 
    digitalWrite(in6, LOW);
    digitalWrite(in7, LOW); 
    digitalWrite(in8, LOW);
  }
  Serial.print("LF = "); Serial.println(LF);
  Serial.print("LR = "); Serial.println(LR);
  Serial.print("RF = "); Serial.println(RF);
  Serial.print("RR = "); Serial.println(RR);
  Serial.println();
  delay(100);
  if(RF < 0){
    digitalWrite(in5, HIGH);
    digitalWrite(in6, LOW);
    analogWrite(enC, 255);
  }
  if(RF > 0){
    digitalWrite(in5, LOW);
    digitalWrite(in6, HIGH);
    analogWrite(enC, 255);
  }
  if(RR < 0){
    digitalWrite(in7, HIGH);
    digitalWrite(in8, LOW);
    analogWrite(enD, 255);
  }
  if(RR > 0){
    digitalWrite(in7, LOW);
    digitalWrite(in8, HIGH);
    analogWrite(enD, 255);
  }
  if(LF < 0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, 255);
  }
  if(LF > 0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 255);
  }
  if(LR > 0){
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, 255);
  }
  if(LR < 0){
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, 255);
  }
}
