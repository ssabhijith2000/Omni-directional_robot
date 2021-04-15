#include <Arduino.h>
//These are the pins to control the relays that control the two motors on diagonals
#define diag_motor1_relay1 3 
#define diag_motor1_relay2 4
#define diag_motor2_relay1 5
#define diag_motor2_relay2 6

//encoder in wheel1
#define wheel1_encoderpin1 7
#define wheel1_encoderpin2 8
#define wheel1_encoderpin3 9
#define wheel1_encoderpin4 10
//encoder in wheel2
#define wheel2_encoderpin1 7
#define wheel2_encoderpin2 8
#define wheel2_encoderpin3 9
#define wheel2_encoderpin4 10
//encoder in wheel3
#define wheel3_encoderpin1 7
#define wheel3_encoderpin2 8
#define wheel3_encoderpin3 9
#define wheel3_encoderpin4 10
//encoder in wheel2
#define wheel4_encoderpin1 7
#define wheel4_encoderpin2 8
#define wheel4_encoderpin3 9
#define wheel4_encoderpin4 10

//These are the pins to control the relays that control the two motors on diagonals
#define steering_motor1_relay1 11 
#define steering_motor1_relay2 12

#define steering_motor2_relay1 13
#define steering_motor2_relay2 14

#define steering_motor3_relay1 15 
#define steering_motor3_relay2 16

#define steering_motor4_relay1 17
#define steering_motor4_relay2 18
int wheelposition = 0, wheel2position=0, wheel3position=0, wheel4position=0 ;

/* encoder data 
-----------------
1001: -90 degree 
0110: -45 degree
0101: 0 degree
1001: 90 degree
*/

//function to move the robot forward
void forward()
{
  digitalWrite(diag_motor1_relay1, HIGH);
  digitalWrite(diag_motor2_relay1, HIGH);
  digitalWrite(diag_motor1_relay2, LOW);
  digitalWrite(diag_motor2_relay2, LOW);
}
//function to move the robot backward
void backward()
{
  digitalWrite(diag_motor1_relay1, LOW);
  digitalWrite(diag_motor2_relay1, LOW);
  digitalWrite(diag_motor1_relay2, HIGH);
  digitalWrite(diag_motor2_relay2, HIGH);
}
//function to brake the vehicle when no command is given
void brake()
{
  digitalWrite(diag_motor1_relay1, LOW);
  digitalWrite(diag_motor2_relay1, LOW);
  digitalWrite(diag_motor1_relay2, LOW);
  digitalWrite(diag_motor2_relay2, LOW);
}

/*function to rotate the steering motor in wheels clockwise
parameters: pins where the relay for controlling motors are connected
*/
void rotate_clockwise(int steerrelay1, int steerrelay2)
{
  digitalWrite(steerrelay1, HIGH);
  digitalWrite(steerrelay2, LOW);
}
/*function to rotate the steering motor in wheels anti-clockwise
parameters: pins where the relay for controlling motors are connected
*/
void rotate_anticlockwise(int steerrelay1, int steerrelay2)
{
  digitalWrite(steerrelay1, LOW);
  digitalWrite(steerrelay2, HIGH);
}
/*function to stop rotating the steering motor in wheels
parameters: pins where the relay for controlling motors are connected
*/
void stop_rotate(int steerrelay1, int steerrelay2)
{
  digitalWrite(steerrelay1, LOW);
  digitalWrite(steerrelay2, LOW);
}

/*function to find the position in degrees(parameters: pinnumbers where enoder is connected)
procedure: 
1. Senses the data from encoder using 4 pins
2. returns the position in degrees by checking the bit pattern
Parameters: encoder pins
*/
int find_degree(int encoder_pin1,int encoder_pin2,int encoder_pin3, int encoder_pin4) 
{
  int encoderdata1, encoderdata2, encoderdata3, encoderdata4;
  encoderdata1 = digitalRead(encoder_pin1);
  encoderdata2 = digitalRead(encoder_pin2);
  encoderdata3 = digitalRead(encoder_pin3);
  encoderdata4 = digitalRead(encoder_pin4);
  if (encoderdata1 == 1 && encoderdata2 == 0 && encoderdata3 == 0 && encoderdata4 == 1)
  {
    return 90;
  }
  if (encoderdata1 == 0 && encoderdata2 == 1 && encoderdata3 == 1 && encoderdata4 == 0)
  {
    return -45;
  }
  if (encoderdata1 == 0 && encoderdata2 == 1 && encoderdata3 == 0 && encoderdata4 == 1)
  {
    return 0;
  }
  if (encoderdata1 == 1 && encoderdata2 == 0 && encoderdata3 == 1 && encoderdata4 == 0)
  {
    return 45;
  }
  if (encoderdata1 == 1 && encoderdata2 == 0 && encoderdata3 == 1 && encoderdata4 == 1)
  {
    return -90;
  }
}
/*Function to set the wheel at desired position
Procedures:
1. Senses the position using find_degree function in degrees
2. Rotates the motor clockwise if position is lesser than given angle till it reaches desired position
3. Rotates the motor anti-clockwise if position is greater than given angle till it reaches desired position
Parameters: required angle, encoder pins, steering pins
*/
void set_motor_at_position(int data, int wheel_encoder1, int wheel_encoder2, int wheel_encoder3, int wheel_encoder4, int steer_rl1, int steer_rl2)
{
      int wheelposition = find_degree(wheel_encoder1,wheel_encoder2,wheel_encoder3,wheel_encoder4);
     
      if (wheelposition < data)
      {
        while(1)
        {
          wheelposition = find_degree(wheel_encoder1,wheel_encoder2,wheel_encoder3,wheel_encoder4);
          rotate_clockwise(steer_rl1,steer_rl2);
          if (wheelposition==data)
          {
            stop_rotate(steer_rl1,steer_rl2);
            break;
          }
        }
      }
      else if (wheelposition > data)
      {
      while(1)
        {
          wheelposition = find_degree(wheel_encoder1,wheel_encoder2,wheel_encoder3,wheel_encoder4);
          rotate_anticlockwise(steer_rl1,steer_rl2);
          if (wheelposition==data)
          {
            stop_rotate(steer_rl1,steer_rl2);
            break;
          }
        }
      }
}


void setup()
{
  Serial.begin(19200);
  pinMode(diag_motor1_relay1, OUTPUT);
  pinMode(diag_motor1_relay2, OUTPUT);
  pinMode(diag_motor2_relay1, OUTPUT);
  pinMode(diag_motor2_relay2, OUTPUT);
  pinMode(wheel1_encoderpin1, INPUT);
  pinMode(wheel1_encoderpin2, INPUT);
  pinMode(wheel1_encoderpin3, INPUT);
  pinMode(wheel1_encoderpin4, INPUT);
}
void loop()
{
  if (Serial.available())
  // if text arrived in from BT serial
  {
    int data = (Serial.read()); //it will be read and
    if (data == 'f')
    {
      forward();
    }
    else if (data == 'b')
    {
      backward();
    }
    else 
    {
      brake();
    }
    if (data == '90' || data == '45' || data == '-45' || data == '0' || data == '-90')
    {
      //Below functionscalls sets the positions of wheels one by one to the given value of degree
      set_motor_at_position(data,wheel1_encoderpin1,wheel1_encoderpin2,wheel1_encoderpin3,wheel1_encoderpin4,steering_motor1_relay2,steering_motor1_relay2);
      set_motor_at_position(data,wheel2_encoderpin1,wheel2_encoderpin2,wheel2_encoderpin3,wheel2_encoderpin4,steering_motor2_relay2,steering_motor2_relay2);
      set_motor_at_position(data,wheel3_encoderpin1,wheel3_encoderpin2,wheel3_encoderpin3,wheel3_encoderpin4,steering_motor3_relay2,steering_motor3_relay2);
      set_motor_at_position(data,wheel4_encoderpin1,wheel4_encoderpin2,wheel4_encoderpin3,wheel4_encoderpin4,steering_motor4_relay2,steering_motor4_relay2);
    }
  }
}
