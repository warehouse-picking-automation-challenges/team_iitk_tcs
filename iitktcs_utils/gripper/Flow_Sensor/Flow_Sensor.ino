//#define USE_USBCON

#include <ros.h>
#include <Servo.h> 
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <rosserial_arduino/Adc.h>

//#define Suction_Down         55
//#define Suction_Up           78
//#define Gripper_Back         55
//#define Gripper_Forward      100
//#define Gripper_Hlf_Close    115
//#define Gripper_Close        120
#define Valve_Back          5 // Valve in open condition
#define Valve_Close         20 // Valve in close condition

#define Valve_Motor         10 // Motor Pin number
#define Vacuum_0            9
#define Vacuum_1            8

//#define Motor_Suction        9
//#define Motor_Gripper        10

ros::NodeHandle  nh;


//Servo Suction;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
Servo Gripper;
Servo Valve;

int sensor_val = 0;
/*void messageCb( const std_msgs::Int16& msg){
  if(msg.data == 10)
  {
    //digitalWrite(13, HIGH);
    Gripper.write(Gripper_Back);
  }
  if(msg.data == 11) 
  {
    //digitalWrite(13, LOW);
    Gripper.write(Gripper_Forward);
  }
  if(msg.data == 12) 
  {
    //digitalWrite(13, LOW);
    Gripper.write(Gripper_Hlf_Close);
  }
  if(msg.data == 13) 
  {
    //digitalWrite(13, LOW);
    Gripper.write(Gripper_Close);
  }
  //myservo.write(msg.data);
}*/

void messageCb1( const std_msgs::Int16& msg){
  if(msg.data == 0)
  {
    //Suction.write(55);
    Valve.write(Valve_Back);
    digitalWrite(Vacuum_0, HIGH);
    digitalWrite(Vacuum_1, HIGH);
  }
  if(msg.data == 1) 
  {
    //Suction.write(80);
    Valve.write(Valve_Close);
    digitalWrite(Vacuum_0, LOW);
    digitalWrite(Vacuum_1, LOW);
  }   
}

 
//int pos = 0;    // variable to store the servo position 
const int Left_Sensor = A5;  // Analog input pin that the potentiometer is attached to
int L_Sensor_Val = 0;        // value read from the pot
const int Right_Sensor = A1;  // Analog input pin that the potentiometer is attached to
int R_Sensor_Val = 0;        // value read from the pot
std_msgs::Int16 sensor_data;
rosserial_arduino::Adc adc_msg;
ros::Publisher pub("/iitktcs/arduino/flow_Sensor", &adc_msg);

//ros::Subscriber<std_msgs::Int16> sub("/iitktcs/arduino/gripper_pos", &messageCb );
ros::Subscriber<std_msgs::Int16> sub1("/iitktcs/arduino/vacuum_ctrl", &messageCb1 );

void setup() 
{ 
  nh.initNode();
  //nh.subscribe(sub);
  nh.subscribe(sub1);
  nh.advertise(pub);
  //Suction.attach(Motor_Suction);  // attaches the servo on pin 9 to the servo object 
  //Suction.attach(Valve_Motor);
  //Gripper.attach(Motor_Gripper);
  pinMode(Vacuum_0, OUTPUT); 
  pinMode(Vacuum_1, OUTPUT);
  pinMode(Valve_Motor, OUTPUT); 
  //Suction.write(Suction_Down); // default pos
  //Gripper.write(Gripper_Back); // default pos
} 
 
 
void loop() 
{ 
  adc_msg.adc0 = analogRead(0);
  pub.publish(&adc_msg);
  nh.spinOnce();
  delay(100);
} 
