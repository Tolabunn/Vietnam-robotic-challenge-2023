#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>
  

#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14
#define SERVO 2
#define SERVO_FREQ_MIN 200
#define SERVO_FREQ_MAX 550
#define SERVO_MIN_DEGREE 0
#define SERVO_MAX_DEGREE 180

#define ENA
#define ENB 

#define pressures false
#define rumble false

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PS2X ps2x;
bool state = false;
short state1 = 0;

void setup() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);

  pwm.setPWM(SERVO,0,0);

  Wire.setClock(400000);
  Serial.begin(115200);
  Serial.print("Ket noi voi tay cam PS2)");

  int error = -1;
  for (int i = 0; i < 10; i++) {
    delay(200);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
  }
  switch (error) {
    case (0):
      Serial.println(" Ket noi tay cam PS2 thanh cong");
      break;
    case (1):
      Serial.println(" LOI) Khong tim thay tay cam, hay kiem tra day ket noi vơi tay cam ");
      break;
    case (2):
      Serial.println(" LOI) khong gui duoc lenh");
      break;
    case (3):
      Serial.println(" LOI) Khong vao duoc Pressures mode ");
      break;
  }

}
// servo setup
void servo_clockwise()
{
  pwm.setPWM(SERVO,0,285);
}
void servo_anticlockwise()
{
  pwm.setPWM(SERVO, 90, 680);
}
void stop_servo()
{
  pwm.setPWM(SERVO, 0, 0);
}
// motor setup
void forwarddc(uint16_t forward, uint16_t forward2)
{
  pwm.setPWM(10,0,forward);
  pwm.setPWM(11,0,forward2);
}
void forward2dc(uint16_t fur, uint16_t fur2)
{
  pwm.setPWM(12,0,fur);
  pwm.setPWM(13,0,fur2);
}
void rightdc(uint16_t right, uint16_t right2)
{
  pwm.setPWM(10,0,right);
  pwm.setPWM(11,0,right2);
}
void right2dc(uint16_t right0, uint16_t right1)
{
  pwm.setPWM(12,0,right0);
  pwm.setPWM(13,0,right1);
}
void leftdc(uint16_t left, uint16_t left2)
{
  pwm.setPWM(10,0,left);
  pwm.setPWM(11,0,left2);
}
void left2dc(uint16_t left0, uint16_t left1)
{
  pwm.setPWM(12,0,left0);
  pwm.setPWM(13,0,left1);
}
void backdc(uint16_t back, uint16_t back2)
{
  pwm.setPWM(10,0,back);
  pwm.setPWM(11,0,back2);
}
void back2dc(uint16_t back0, uint16_t back1)
{
  pwm.setPWM(12,0,back0);
  pwm.setPWM(13,0,back1);
}
void eco()
{
  forward2dc(0,1000);
  forwarddc(0,1000);
  back2dc(1000,0);
  backdc(1000,0);
  left2dc(0,1000);
  leftdc(1000,0);
  right2dc(1000,0);
  rightdc(0,1000);
}
void sport()
{
  forward2dc(0,3000);
  forwarddc(0,3000);
  back2dc(3000,0);
  backdc(3000,0);
  left2dc(0,3000);
  leftdc(3000,0);
  right2dc(3000,0);
  rightdc(0,3000);
}
// shooter and collector setup
void collector()
{
  pwm.setPWM(8,0,0);
  pwm.setPWM(9,0,4095);
}
void collector_stop()
{
  pwm.setPWM(8,0,0);
  pwm.setPWM(9,0,0);
}
void shooter()
{
  pwm.setPWM(14,0,0);
  pwm.setPWM(15,0,3000);
}
void shooter_stop()
{
  pwm.setPWM(14,0,0);
  pwm.setPWM(15,0,0);
}
// control setup
void ps2Control() 
{
  ps2x.read_gamepad(false, false);
  delay(50);
  // control motor with joystick
  // if(ps2x.Analog(PSS_LX) == 0)
  // {
  //   left2dc(0,1000);
  //   leftdc(1000,0);
  // }
  // else if(ps2x.Analog(PSS_LX) == 255)
  // {
  //   right2dc(1000,0);
  //   rightdc(0,1000);
  // }
  // else if(ps2x.Analog(PSS_LY) == 0)
  // {
  //   forward2dc(0,1000);
  //   forwarddc(0,1000);
  // }
  // else if(ps2x.Analog(PSS_LY) == 255)
  // {
  //   back2dc(1000,0);
  //   backdc(1000,0);
  // }
  // else if(ps2x.Analog(PSS_RX) == 0)
  // {
  //   left2dc(0,3000);
  //   leftdc(3000,0);
  // }
  // else if(ps2x.Analog(PSS_RX) == 255)
  // {
  //   right2dc(3000,0);
  //   rightdc(0,3000);
  // }
  // else if(ps2x.Analog(PSS_RY) == 0)
  // {
  //   forward2dc(0,3000);
  //   forwarddc(0,3000);
  // }
  // else if(ps2x.Analog(PSS_RY) == 255)
  // {
  //   back2dc(3000,0);
  //   backdc(3000,0);
  // }
  // else
  // {
  //   left2dc(0,0);
  //   leftdc(0,0);
  //   right2dc(0,0);
  //   rightdc(0,0);
  // }
  if(ps2x.Analog(PSS_LY) == 0)
  {
    forwarddc(0,1000);
  }
  else if(ps2x.Analog(PSS_RY) == 0)
  {
    forward2dc(0,1000);
  }
  else if(ps2x.Analog(PSS_LY) == 255)
  {
    backdc(1000,0);
  else if(ps2x.Analog(PSS_RY) == 255)
  {
    back2dc(1000,0);
  }
  else
  {
    forwarddc(0,0);
    forward2dc(0,0);
    backdc(0,0);
    back2dc(0,0);
  }
//control servo clockwise and anticlockwise
  if (ps2x.Button(PSB_GREEN))
  {
    servo_clockwise();
    Serial.println("triangle");
  }
  if (ps2x.Button(PSB_BLUE))
  {
    servo_anticlockwise();
    Serial.println("blue");
  }
  if(ps2x.ButtonReleased(PSB_BLUE))
  {
    stop_servo();
  }
//shooting motor
  if(ps2x.Button(PSB_R1))
  {
    shooter();
  }
  if(ps2x.Button(PSB_R2))
  {
    shooter_stop();
  }
//ball collector motor
  if(ps2x.Button(PSB_L1))
  {
    collector();
  }
  if(ps2x.Button(PSB_L2))
  {
    collector_stop();
  }
  
  if (ps2x.Button(PSB_PAD_UP))
  {
    state = !state;
  }
  if (state)
  {
    shooter();
  }
  else
  {
    shooter_stop();
  }
  // if(ps2x.Button(PSB_CIRCLE))
  // {
  //   switch(state1 % 3 + 1)
  //   {
  //     case (0):
  //       shooter();
  //     case (1):
  //       shooter_stop();
  //     case (2):
  //       collector();
  //       break;
  //   }
  // }

}
void loop() {
  ps2Control();
}
