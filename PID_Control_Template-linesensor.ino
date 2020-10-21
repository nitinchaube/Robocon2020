
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
int yaw = 0;
Madgwick filter;
unsigned long microsPerReading, microsPrevious,microsNow;
float accelScale, gyroScale;

int pwm1=3;
int pwm2=5;
const byte dir1 = 2;
const byte dir2 = 4;
const byte dir3 = 8;
const byte dir4 = 9;





double sensed_output, control_signal;
double setpoint=180;
double Kp=1.0; //proportional gain
double Ki=0; //integral gain
double Kd=0; //derivative gain
int T=10; //sample time in milliseconds (ms)
unsigned long last_time;
double total_error, last_error;
int max_control=255;
int min_control=0; 

void setup(){ 

  Serial.begin(9600); 
  pinMode(pwm1,OUTPUT);
  pinMode(pwm2,OUTPUT);
  pinMode(dir1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(dir3,OUTPUT);
  pinMode(dir4,OUTPUT);

  


  filter.begin(25);

  // initialize variables to pace updates to correct rate
  microsPerReading = 600000 / 25;
  microsPrevious = micros();

 

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  
}

void loop(){

timer = millis();
   microsNow = micros();
   if (microsNow - microsPrevious >= microsPerReading) {

    // Output raw
    Vector ng = mpu.readNormalizeGyro();


  Vector na= mpu.readNormalizeAccel();

    // update the filter, which computes orientation
    filter.updateIMU(ng.XAxis,ng.YAxis,ng.ZAxis,na.XAxis,na.YAxis,na.ZAxis);

    // print the heading, pitch and roll
    
   yaw = filter.getYaw();
   // Serial.print("yaw ");
    //Serial.println(yaw);
   yaw=yaw-180;
   
    
   
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;

  // Read normalized values
   }
  sensed_output=yaw;


  
 PID_Control(); //calls the PID function every T interval and outputs a control signal  
 
}

void PID_Control(){

  unsigned long current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
 
  int delta_time = current_time - last_time; //delta time interval 
  
  if (delta_time >= T){

    double error = setpoint - sensed_output;
   
    total_error += error; //accumalates the error - integral term
    if (total_error >= max_control) total_error = max_control;
    else if (total_error <= min_control) total_error = min_control;
    
    double delta_error = error - last_error; //difference of error for derivative term

    control_signal = Kp*error + (Ki*T)*total_error + (Kd/T)*delta_error;; //PID control compute
   
    if (control_signal >= max_control) control_signal = max_control;
    else if (control_signal <= min_control) control_signal = min_control;

if(yaw<0)
{      Serial.print(yaw);
       Serial.println("R");

       analogWrite(pwm1,control_signal);
      analogWrite(pwm2,0);
      
      digitalWrite(dir1,LOW);
      digitalWrite(dir2,HIGH);
      digitalWrite(dir3,LOW);
      digitalWrite(dir4,LOW);

  }
else if(yaw>0)
{ 
    Serial.print(yaw);
    Serial.println("L");

    analogWrite(pwm2,control_signal); 
    analogWrite(pwm1,0);
     
      digitalWrite(dir1,LOW);
      digitalWrite(dir2,LOW);
      digitalWrite(dir3,HIGH);
      digitalWrite(dir4,LOW);
  }
/*else
{   Serial.print(yaw);
    Serial.println("F");

   
    analogWrite(pwm1,50);
     analogWrite(pwm2,50);

     digitalWrite(dir1,LOW);
    digitalWrite(dir2,HIGH);
    digitalWrite(dir3,HIGH);
    digitalWrite(dir4,LOW);
}*/



    
    last_error = error;
    last_time = current_time;
    }  
}
