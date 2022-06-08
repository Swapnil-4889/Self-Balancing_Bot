// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>   
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define PI 3.141592   
#define dela 500

Adafruit_MPU6050 mpu;
float i=0;
double d=0;
float alpha=0;
const int PinR1 = 5;    //  arduino  pin 5 to l293d  pin IN4
const int PinR2 = 6;    //  arduino  pin 6 to l293d  pin IN3
const int PinL1 = 7;    //  arduino  pin 7 to l293d  pin IN1
const int PinL2 = 8;    //  arduino  pin 8 to l293d  pin IN2
const int PwmR  = 9;    //  arduino  pin 9 to l293d  pin ENB for PWM to Motor 1
const int PwmL  = 10;  //   arduino  pin 10 to l293d  pin ENA for PWM to Motor 2

// PID control parameters
const float kp=4000;   
const float ki=50;
const float kd=800;

//variables to control motor direction
int aa=0;         
int bb=0;

float pwm=0; // variable that is proportional to the motor output


// setup function contains code for arduino pin initialisation and data transfer ditrection
// here the sensor MPU6050 is initiated as well

void setup(void) {
  pinMode(PinR1,OUTPUT);
  pinMode(PinR2,OUTPUT);
  pinMode(PinL1,OUTPUT);
  pinMode(PinL2,OUTPUT);
  
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
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

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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

  Serial.println("");
  delay(100);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */ 
  float e=a.acceleration.y;  // acceleration value along vertical axis of robot
  float theta = asin(e/10);  // angle between vertical axis of robot and the vertical axis in ground frame  
  theta=theta*180/PI;        // angle conversion from radians to degree   
  d=(theta-alpha)/dela;      // numerical differentiation of theta   
  d=d*1000;                  // multiply by 1000 as delay is in milliseconds
  alpha=theta;               // variable to store thet value from previous iteration of loop function
  Serial.print("theta ");
  Serial.print(theta);
  Serial.println("");
  i=i+(theta*dela/1000);   // numerical integration of theta
  Serial.print("d :");
  Serial.print(d);
  Serial.println("");
  Serial.print("i ");
  Serial.print(i);
  Serial.println("");
  pwm = ( theta * kp ) + ( i*ki  ) + ( d * kd ) ; // PID pulse width modulated input to motor driver which will further supply power to motors
  if(pwm < 0){    
            aa = 1;   
            bb = 0;
          }
          if(pwm > 0){    
            aa = 0;
            bb= 1;                                           
          }
   pwm  = abs(pwm);                  
          if ( pwm < 0) pwm = 0;
          if ( pwm > 255) pwm = 255;

 if(abs(theta) > 0.5){  // if theta becomes more than 0.5 degree then motors have to be supplied power 
              analogWrite(PwmR, pwm);
              digitalWrite(PinR1, aa);
              digitalWrite(PinR2 ,bb);
             
              analogWrite(PwmL ,pwm);
              digitalWrite(PinL1 ,aa);
              digitalWrite(PinL2 ,bb);
              }
           else{
              analogWrite(PwmR , 0);
              analogWrite(PwmL , 0);
              i = 0;
              delay(dela);
           }
         

  
}
