#include "Wire.h"

//MPU 6050 uses 16 bit data to comunicate

int16_t Acc_X, Acc_Y, Acc_Z,Gyro_X, Gyro_Y, Gyro_Z;
 

float Acceleration_angle[2], Total_angle[2], Gyro_angle[2];

int i, enA = 3, in1 = 2, in2 = 4, enB = 5, in3 = 7, in4 = 8, enC = 6, in5 = 10, in6 = 11, enD = 9, in7 = 12, in8 = 13;
int gyro_error, acc_error;
float elapsedTime, time, timePrev;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0, pid_i=0, pid_d=0;

double kp=0.311; //PID CONSTANTS
double ki=0.083;
double kd=1.744;


float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady


void setup() {
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  time = millis(); //Start counting time in milliseconds
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
}//end of setup void

void loop() {

/////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 
  
  /*The tiemStep is the time that elapsed since the previous loop. 
   * This is the value that we will use in the formulas as "elapsedTime" 
   * in seconds. We work in ms so we haveto divide the value by 1000 
   to obtain seconds*/

  /*Reed the values that the accelerometre gives.
   * We know that the slave adress for this IMU is 0x68 in
   * hexadecimal. For that in the RequestFrom and the 
   * begin functions we have to put this value.*/
   
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
   
   /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them  
    * and just make then sum of each pair. For that we shift to the left the high values 
    * register (<<) and make an or (|) operation to add the low values.*/
    
     Acc_X=Wire.read()<<8 | Wire.read(); //each value needs two registres
     Acc_Y=Wire.read()<<8 | Wire.read();
     Acc_Z=Wire.read()<<8 | Wire.read();

 
    /*///This is the part where you need to calculate the angles using Euler equations///*/
    
    /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
     * values that we have just read by 16384.0 because that is the value that the MPU6050 
     * datasheet gives us.*/
    /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
    * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
    * to calculate this value in each loop we have done that just once before the setup void.
    */

    /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
     *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
     *  will calculate the rooth square.*/
     /*---X---*/
     Acceleration_angle[0] = atan((Acc_Y/16384.0)/sqrt(pow((Acc_X/16384.0),2) + pow((Acc_Z/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_X/16384.0)/sqrt(pow((Acc_Y/16384.0),2) + pow((Acc_Z/16384.0),2)))*rad_to_deg;
 
   /*Now we read the Gyro data in the same way as the Acc data. The adress for the
    * gyro data starts at 0x43. We can see this adresses if we look at the register map
    * of the MPU6050. In this case we request just 4 values. W don't want the gyro for 
    * the Z axis (YAW).*/
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
   Gyro_X = Wire.read()<<8 | Wire.read(); //Once again we shift and sum
   Gyro_Y = Wire.read()<<8 | Wire.read();
 
   /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
   the raw value by 131 because that's the value that the datasheet gives us*/

   /*---X---*/
   Gyro_angle[0] = Gyro_X/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyro_Y/131.0;

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
   /*Now we have our angles in degree and values from -100º to 100º aprox*/
    //Serial.println(Total_angle[1]);

   
  
/*///////////////////////////P I D///////////////////////////////////*/
/*Remember that for the balance we will use just one axis. I've choose the x angle
to implement the PID with. That means that the x axis of the IMU has to be paralel to
the balance*/

/*First calculate the error between the desired angle and 
*the real measured angle*/
error = Total_angle[1] - desired_angle;
    
/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/

pid_p = kp*error;

/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if(-3 <error <3)
{
  pid_i = pid_i+(ki*error);  
}

/*The last part is the derivate. The derivate acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For that we will use a variable called previous_error.
We substract that value from the actual error and divide all by the elapsed time. 
Finnaly we multiply the result by the derivate constant*/

pid_d = kd*((error - previous_error)/elapsedTime);

/*The final PID values is the sum of each of this 3 parts*/
PID = pid_p + pid_i + pid_d;

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwmLeft = 0 +PID;
pwmRight = 0 - PID;

//Right
if(pwmRight < -255)
{
  pwmRight= -255;
}
if(pwmRight > 255)
{
  pwmRight=255;
}
//Left
if(pwmLeft < -255)
{
  pwmLeft= -255;
}
if(pwmLeft > 255)
{
  pwmLeft=255;
}

//Using the pwm output from the motor detemins which motor needs to go up or down using the calculated pwm
if(pwmRight > 0){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, pwmRight);
}
if(pwmRight < 0){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, -pwmRight);
}
if(pwmLeft > 0){
  //write for both motors when pins worked out
  analogWrite(enA, pwmLeft);
}
if(pwmLeft < 0){
  //Same here
  analogWrite(enA, -pwmLeft);
}
if(pwmLeft == 0 && pwmRight == 0){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

previous_error = error; //Remember to store the previous error.

}
//end of loop void
