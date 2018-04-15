The Arduino sketche ServoControl4Channel runs using rosserial and is a ros node which controls upto four RC Servo's.
It is based on the rosserial Servo Control Example
This version controls upto four RC Servos
Servo 0 is on pin 9,
Servo 1 is on pin 6,
Servo 2 is on pin 5,
Servo 3 is on pin 3.
The node subscribes to the servo topic and acts on a UInt16 standard message.
The lower 8 bits are the angle and the upper 8 bits are the servo which is to be addressed.
Example of data to send
0x00B4 will set servo 0 to 180 degrees
0x01B4 will set servo 1 to 180 degress
Note: I would like to use my own message that contains the servo address and the angle but this caused an Error Creation of subscriber failed: need more than 1 value to unpack. I hope to fix this in the future.
