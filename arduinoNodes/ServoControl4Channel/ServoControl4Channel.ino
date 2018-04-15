/*
 * Based on the rosserial Servo Control Example
 * This version controls upto four RC Servos
 * Servo 0 is on pin 9,
 * Servo 1 is on pin 6,
 * Servo 2 is on pin 5,
 * Servo 3 is on pin 3.
 * The node subscribes to the servo topic and acts on a UInt16 standard message.
 * The lower 8 bits are the angle and the upper 8 bits are the servo which is to be addressed.
 * Example of data to send
 * 0x00B4 will set servo 0 to 180 degrees
 * 0x01B4 will set servo 1 to 180 degress
 * Note: I would like to use my own message that contains the servo address and the angle but
 * this caused an Error Creation of subscriber failed: need more than 1 value to unpack. I hope
 * to fix this in the future.
 *
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  std_msgs::UInt16 servoIndex = cmd_msg;
  servoIndex.data = (servoIndex.data & 0xFF00);
  
  /* Which servo to drive */
  switch(servoIndex.data)
  {
    case 0x0000:
      nh.logdebug("Servo 0 ");
      servo0.write(cmd_msg.data & 0x00FF); //set servo 0 angle, should be from 0-180
      break;

    case 0x0100:
      nh.logdebug("Servo 1 ");
      servo1.write(cmd_msg.data & 0x00FF); //set servo 1 angle, should be from 0-180
      break;

    case 0x0200:
      nh.logdebug("Servo 2 ");
      servo2.write(cmd_msg.data & 0x00FF); //set servo 2 angle, should be from 0-180
      break;

    case 0x0300:
      nh.logdebug("Servo 3 ");
      servo3.write(cmd_msg.data & 0x00FF); //set servo 3 angle, should be from 0-180
      break;
      
    default:
      nh.logdebug("No Servo");
      break;
  }
    
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo0.attach(9); //attach it to pin 9 (Servo 0)
  servo1.attach(6); //attach it to pin 6 (Servo 1)
  servo2.attach(5); //attach it to pin 5 (Servo 2)
  servo3.attach(3); //attach it to pin 3 (Servo 3)
}

void loop(){
  nh.spinOnce();
  delay(1);
}
