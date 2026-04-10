
#include "HC_SR04.h"
#include "Motor.h"
#include<Servo.h>

#define A IRMR
#define B IRML
#define C IRR
#define D IRL
enum STATUS_TYPE
{
  NO_ACTION,
  STOP_DETECTED_FORWARD,
  STOP_DETECTED_RIGHT,
  STOP_DETECTED_LEFT,
  RIGHT_DISTANCE_MEASURED,
  LEFT_DISTABCE_MEASURED,
};

enum VECHICAL_STATE_TYPE
{
  TURNING_RIGHT = 1,
  TURNING_LEFT,
  FORWARD,
  BACKWARD
};


enum CAR_DIRECTIONS_TYPE
{
  NO_ACTION_DIR = 0,
  FORWARD_CAR_DIR,
  BACKWARD_CAR_DIR,
  LEFT_CAR_DIR,
  RIGHT_CAR_DIR,
  STOP_CAR_DIR
};

enum SERVO_STATE_TYPE
{
  SERVO_NO_ACTION,
  SERVO_MIDDLE,
  SERVO_TURN_RIGHT,
  SERVO_TURN_LEFT,
};

enum IR_STATE_TYPE
{
  LINE_FOLLOWING,
  FREE_MOVE,
  AUTOMATIC,
  BULLTOOTH
};
#define SET   20
#define RIGHT 0
#define LEFT  180
#define MIDDLE 90
#define SPEED 150
#define TRIG_PIN 4
#define ECHO_PIN 2
#define ECHO_INT 0
#define IRR_PIN A0
#define IRMR_PIN A1
#define IRML_PIN A2
#define IRL_PIN A3
#define IN1  5
#define IN2  6
#define IN3  3
#define IN4  12
#define T2        50
#define SERVO_PIN 11
#define PWM 100
int T =  500;
Motor M1(IN1,IN2);
Motor M2(IN3,IN4);
char x;
byte Reading;
int ULTRA_Reading;
int ULTRA_Reading_R;
int ULTRA_Reading_L;
bool detected = false;
Servo S;
HC_SR04 sensor(TRIG_PIN, ECHO_PIN, ECHO_INT);
STATUS_TYPE state = NO_ACTION;
CAR_DIRECTIONS_TYPE DIR = NO_ACTION_DIR;
CAR_DIRECTIONS_TYPE DIR_B = NO_ACTION_DIR;
CAR_DIRECTIONS_TYPE DIR_IR = NO_ACTION_DIR;
CAR_DIRECTIONS_TYPE DIR_US = NO_ACTION_DIR;
SERVO_STATE_TYPE SERVO_STATE = SERVO_NO_ACTION;
IR_STATE_TYPE IR_STATE = FREE_MOVE;
IR_STATE_TYPE FOLLOWING_STATE = AUTOMATIC;
VECHICAL_STATE_TYPE vechical_state = 0;
long LoopTimer1;
long LoopTimer2;
long LoopTimer3;
long delta;
long error;
long dLT = 500; //Desired Loop Timer
long T1 = 498;
int Kp = 1,Ki = 1,Kd;
float Coutput;
int prverror;
//int state = 0;
bool isFinished = false;
bool IRMR;  // IR middle Right
bool IRML;  // IR middle left
bool IRR;   // IR Right
bool IRL;   // IR Left
byte s = 0;
volatile bool updated = false;
int temp_Reading = 0;
void setup(){
  M1.init();
  M2.init();  
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(IRR_PIN,INPUT);
  pinMode(IRL_PIN,INPUT);
  pinMode(IRMR_PIN,INPUT);
  pinMode(IRML_PIN,INPUT);
  S.attach(SERVO_PIN);
  sensor.begin();
  Serial.begin(9600); 
  while(!Serial) continue;
  Serial.println("HAllO");
  sensor.start();
  LoopTimer1 = millis();
  LoopTimer2 = millis();
  
  S.write(MIDDLE);   // Servo 90 Degree at the intilization
  SERVO_STATE = SERVO_MIDDLE;
  //S.write(0);
  
  //M1.run(100, 2);
  /*analogWrite(IN3,255-100);
  digitalWrite(IN4,HIGH);*/
  //right(100);
  

}
volatile int Reading_State = 0;
volatile bool Wall_Detected = false;
volatile bool excute_once_flag = false;
volatile bool d = false;
bool read = false;
int m = 0;
char c;
byte F,R,L,N;
int val;
int val2 = 0;
bool move_F = false;
bool move_B = false;
void loop()
{
  IRR = digitalRead(IRR_PIN);
  IRL = digitalRead(IRL_PIN);
  IRMR = digitalRead(IRMR_PIN);
  IRML = digitalRead(IRML_PIN);
  val2 = IRR + IRMR*10 + IRML*100 + IRL*1000;
  //Serial.println(val2);
  switch(m)
  {
    case 0:
      if(Serial.available()>0 && IR_STATE == FREE_MOVE)
      {
        c = Serial.read();
        Serial.println(c);
        switch(c)
        {
          case 'F':
            forward(PWM);
            //Serial.println("Forward");
            c = 0;
          break;
          case 'B':
            backward(PWM);
            //Serial.println("Backward");
            c = 0;
          break;
          case 'R':
            right(PWM);
            //Serial.println("RIGHT");
            c = 0;
          break;
          case 'L':
            left(PWM);
            //Serial.println("LEFT");
            c = 0;
          break;
          case 'W':
            FOLLOWING_STATE = AUTOMATIC;
            Serial.println("AUTOMATIC FOLLOWING STATE");
            c = 0;
          break;
          case 'w':
            FOLLOWING_STATE = BULLTOOTH;
            Serial.println("BULLTOOTH FOLLOWING STATE");
            c = 0;
          break;
          case 'S':
            //Serial.println("STOP");
            stop();
          break;         
        }
      }
      sensor.start();
      while(!sensor.isFinished()) continue;
      //Serial.print(sensor.getRange());
      //Serial.println("cm");
      if(IRMR == 1 || IRML == 1 || IRL == 1 || IRR == 1)
      {
        IR_STATE = LINE_FOLLOWING;
        Serial.println("Line Following Mode");
        m = 8;
      }else
      {
        IR_STATE = FREE_MOVE;
      }
      if(sensor.getRange()<= 20 && SERVO_STATE == SERVO_MIDDLE)
      {
        //Serial.println("Backward");
        //backward(PWM);
        //delay(500);
        //Serial.println("STOP");
        stop();
        Serial.println("Comapring Sides Routine");
        m = 1;
      }
      delay(50);
    break;
    case 1:
      S.write(RIGHT);
      delay(500);
      SERVO_STATE == SERVO_TURN_RIGHT;
      sensor.start();
      while(!sensor.isFinished()) continue;
      ULTRA_Reading_R = sensor.getRange();
      Serial.print("Right Reading is = ");
      Serial.print(ULTRA_Reading_R);
      Serial.println("cm");
      delay(500);
      m = 2;
    break;
    case 2:
      S.write(LEFT);
      delay(500);
      SERVO_STATE == SERVO_TURN_LEFT;
      sensor.start();
      while(!sensor.isFinished()) continue;
      ULTRA_Reading_L = sensor.getRange();
      Serial.print("Left Reading is = ");
      Serial.print(ULTRA_Reading_L);
      Serial.println("cm");
      delay(500);
      S.write(MIDDLE);
      SERVO_STATE == SERVO_MIDDLE;
      delay(500);
      m = 3;
    break;
    case 3:
      if(ULTRA_Reading_L >= ULTRA_Reading_R)
      {
        //S.write(RIGHT);
        //delay(500);
        sensor.start();
        m = 4;
        
      }else
      {
        //S.write(LEFT);
        //delay(500);
        sensor.start();
        m = 5;
      }
    break;
    case 4:
        if(sensor.isFinished())
        {
          Serial.println("Inside IF");
          temp_Reading = sensor.getRange();
          Serial.print("Temp Reading is = ");
          Serial.println(temp_Reading);
          if(temp_Reading <= SET)
          {
              //left(150);
              m = 6;
          }else
          {
            Serial.println("Stop");
            stop();
            S.write(MIDDLE);
            SERVO_STATE == SERVO_MIDDLE;
            delay(200);
            m = 0;
          }
        }
        sensor.start();
        delay(50);
    break;
    case 5:
        if(sensor.isFinished() == true)
        {
          Serial.println("Inside IF");
          temp_Reading = sensor.getRange();
          Serial.print("Temp Reading is = ");
          Serial.println(temp_Reading);
          if(temp_Reading <= SET)
          {
              //left(150);
            //Serial.println("Turning RIGHT");
            m = 7;
          }else
          {
            Serial.println("Stop");
            stop();
            S.write(MIDDLE);
            SERVO_STATE == SERVO_MIDDLE;
            delay(200);
            m = 0;
          }
        }
        sensor.start();
        delay(50);
    break;
    case 6:
      //Serial.println("Turning LEFT");
      left(PWM);
      delay(500);
      stop();
      sensor.start();
      m = 4;
    break;
    case 7:
      //Serial.println("Turning RIGHT");
      right(PWM);
      delay(500);
      stop();
      sensor.start();
      m = 5;
    break;
    case 8:
      F = (A)&(B)&(!C) | (A)&(B)&(!D);
      R = (!B)&(C) | (A)&(!B);
      L = (B)&(!A) | (!A)&(!C)&(D);
      N = (!A)&(!B)&(!C)&(!D);
      val = N + L*10 + R*100 + F*1000;
      /*Serial.println("Inside Case 8");
      Serial.print("F is ");
      Serial.println(F);
      Serial.print("R is ");
      Serial.println(R);
      Serial.print("L is ");
      Serial.println(L);
      Serial.print("N is ");
      Serial.println(N);
      Serial.print("Val is");
      Serial.println(val);*/
      if(FOLLOWING_STATE == AUTOMATIC)
      {
          switch (val)
          {
            case 1000:
              //Serial.println("Forward");
              forward(PWM);
            break;
            case 100:
              //Serial.println("RIGHT");
              right(PWM);
            break;
            case 10:
              //Serial.println("LEFT");
              left(PWM);
            break;
            case 1:
              //Serial.println("STOP");
              stop();
              m = 0;
              //IR_STATE = FREE_MOVE;
            break;
          }
          if(Serial.available() > 0)  // Set Following Mode 
          {
            c = Serial.read();
            switch(c)
            {
              case 'W':
                FOLLOWING_STATE = AUTOMATIC;
                Serial.println("Automatic Following STATE");
                c = 0;
              break;
              case 'w':
                FOLLOWING_STATE = BULLTOOTH;
                Serial.println("BULLTOOTH FOLLOWING STATE");
                c = 0;
              break;
              case 'S':
                //Serial.println("STOP");
                stop();
              break;         
            }
          }
      }else if(FOLLOWING_STATE = BULLTOOTH)
      {
          if(Serial.available() > 0)
          {
            c = Serial.read();
            switch(c)
            {
              case 'F':
                //forward(PWM);
                Serial.println("Moving Forward Enable");
                move_F = true;
                move_B = false;
                c = 0;
              break;
              case 'B':
                //backward(PWM);
                //Serial.println("Backward");
                Serial.println("Moving Backward Enable");
                move_B = true;
                move_F = false;
                c = 0;
              break;
              case 'W':
                FOLLOWING_STATE = AUTOMATIC;
                Serial.println("AUTOMATIC FOLLOWING STATE");
                c = 0;
                break;
              case 'w':
                FOLLOWING_STATE = BULLTOOTH;
                Serial.println("BULLTOOTH FOLLOWING STATE");
                c = 0;
              break;
              case 'S':
                //Serial.println("STOP");
                move_F = false;
                move_B = false;
                stop();
              break;         
            }
          }
          if(move_F == true)
          {
            switch (val)
            {
              case 1000:
                //Serial.println("Forward");
                forward(PWM);
              break;
              case 100:
                //Serial.println("RIGHT");
                right(PWM);
              break;
              case 10:
                //Serial.println("LEFT");
                left(PWM);
              break;
              case 1:
                //Serial.println("STOP");
                stop();
                m = 0;
                IR_STATE = FREE_MOVE;
              break;
            }
          }
          if(move_B == true)
          {
            switch (val)
            {
              case 1000:
                //Serial.println("Backward");
                backward(PWM);
              break;
              case 100:
                //Serial.println("LEFT");
                right(PWM);
              break;
              case 10:
                //Serial.println("RIGHT");
                left(PWM);
              break;
              case 1:
                //Serial.println("STOP");
                stop();
                IR_STATE = FREE_MOVE;
                m = 0;
              break;
            }
          } 
      }
      m = 0;  
    break;
  }
}









void left(int pwm)
{
  M1.run(pwm,2);
  M2.run(pwm,2);
}
void right(int pwm)
{
  M1.run(pwm,1);
  M2.run(pwm,1);
}
void backward(int pwm)
{
  M1.run(pwm,1);
  M2.run(pwm,2);
}
void forward(int pwm)
{
  M1.run(pwm,2);
  M2.run(pwm,1);
}
void stop()
{
  M1.stop();
  M2.stop();
}

/*void serialEvent()
{
  
}*/


bool updata_left(int *Reading)
{
  static byte state = 0;
  static long LoopTimer = millis();
  int temp;
  
  S.write(180);
  static bool isFinished = false;
  if(isFinished == true)
  {
    LoopTimer = millis();
    isFinished == false;
  }
  
  if(millis()-LoopTimer >= 1000 )
  {
    LoopTimer = millis();
    state = 1;
  }
  switch (state)
  {
    case 1:
      LoopTimer = millis();
      state = 2;
    break;
    case 2:
      sensor.start();
      if(millis() - LoopTimer >= 50)
      {
        state = 3;
        LoopTimer = millis();
      }
      
    break;
    case 3:
      if(sensor.isFinished())
      {
        *Reading = sensor.getRange();
        
        state = 0;
        isFinished == true;
        return true;
      }
    break;
  }
}

bool updata_right(int *Reading)
{
  static byte state = 0;
  static long LoopTimer = millis();
  static bool isFinished = false;
  if(isFinished == true)
  {
    LoopTimer = millis();
    isFinished == false;
  }
  if(millis()-LoopTimer >= 1000)
  {
    LoopTimer = millis();
    state = 1;
  }
  S.write(0);
  
  switch (state)
  {
    case 1:
      LoopTimer = millis();
      state = 2;
    break;
    case 2:
      sensor.start();
      if(millis() - LoopTimer >= 50)
      {
        state = 3;
        LoopTimer = millis();
      }
      
    break;
    case 3:
      if(sensor.isFinished())
      {
        *Reading = sensor.getRange();
        state = 0;
        isFinished == true;
        return true;
      }
    break;
  }
}

bool updata_center(int *Reading)
{
  static byte state = 0;
  static long LoopTimer = millis();
  static bool isFinished = false;
  if(isFinished == true)
  {
    LoopTimer = millis();
    isFinished == false;
  }
  if(millis()-LoopTimer >= 1000)
  {
    LoopTimer = millis();
    state = 1;
  }
  S.write(90);
  switch (state)
  {
    case 1:
      Serial.println("Inside Case 0 of Ceneter");
      LoopTimer = millis();
      state = 2;
    break;
    case 2:
      sensor.start();
      Serial.println("Inside Case 1 of Ceneter");
      if(millis() - LoopTimer >= 50)
      {
        state = 3;
        LoopTimer = millis();
      }
      
    break;
    case 3:
      Serial.println("Inside Case 2 of Ceneter");
      if(sensor.isFinished())
      {
        *Reading = sensor.getRange();
        state = 0;
        isFinished == true;
        return true;
      }
    break;
  }
}


bool updata_sensor(int *Reading , bool Stop)
{
  static byte state = 0;
  static long LoopTimer = millis();
  static bool isFinished = false;
  //Serial.println("Inside Updata Sensor ");
  if(isFinished == true)
  {
    LoopTimer = millis();
    isFinished == false;
  }
  switch (state)
  {
    case 0:
      //Serial.println("Inside Case 0 of Ceneter");
      LoopTimer = millis();
      if(Stop == true)
      {
        return true;
      }else
      {
        state = 1;
      }
      

    break;
    case 1:
      sensor.start();
      //Serial.println("Inside Case 1 of Ceneter");
      if(millis() - LoopTimer >= 50)
      {
        state = 2;
        LoopTimer = millis();
      }
      
    break;
    case 2:
      //Serial.println("Inside Case 2 of Ceneter");
      if(sensor.isFinished())
      {
        *Reading = sensor.getRange();
        state = 0;
        isFinished == true;
        return true;
      }
    break;
  }
}

bool updata_sensor_L(int *Reading , bool Stop)
{
  static byte state = 0;
  static long LoopTimer = millis();
  static bool isFinished = false;
  //Serial.println("Inside Updata Sensor ");
  if(isFinished == true)
  {
    LoopTimer = millis();
    isFinished == false;
  }
  switch (state)
  {
    case 0:
      //Serial.println("Inside Case 0 of Ceneter");
      LoopTimer = millis();
      if(Stop == true)
      {
        return true;
      }else
      {
        state = 1;
      }
      

    break;
    case 1:
      sensor.start();
      //Serial.println("Inside Case 1 of Ceneter");
      if(millis() - LoopTimer >= 50)
      {
        state = 2;
        LoopTimer = millis();
      }
      
    break;
    case 2:
      //Serial.println("Inside Case 2 of Ceneter");
      if(sensor.isFinished())
      {
        *Reading = sensor.getRange();
        state = 0;
        isFinished == true;
        return true;
      }
    break;
  }
}

bool updata_sensor_R(int *Reading , bool Stop)
{
  static byte state = 0;
  static long LoopTimer = millis();
  static bool isFinished = false;
  //Serial.println("Inside Updata Sensor ");
  if(isFinished == true)
  {
    LoopTimer = millis();
    isFinished == false;
  }
  switch (state)
  {
    case 0:
      //Serial.println("Inside Case 0 of Ceneter");
      LoopTimer = millis();
      if(Stop == true)
      {
        return true;
      }else
      {
        state = 1;
      }
      

    break;
    case 1:
      sensor.start();
      //Serial.println("Inside Case 1 of Ceneter");
      if(millis() - LoopTimer >= 50)
      {
        state = 2;
        LoopTimer = millis();
      }
      
    break;
    case 2:
      //Serial.println("Inside Case 2 of Ceneter");
      if(sensor.isFinished())
      {
        *Reading = sensor.getRange();
        state = 0;
        isFinished == true;
        return true;
      }
    break;
  }
}

bool update_3DIR(int* M,int *L , int *R,long T)
{
  static long long LoopTimer = millis();
  static bool updated = false;
  static int state = 0;
  switch(state)
  {
    case 0:
      LoopTimer = millis();
      state = 1;
      return false;
    break;
    case 1:
      S.write(90);
      if(millis() - LoopTimer >= T)
      {
        state = 2;
        LoopTimer = millis();
      }
    break;
    case 2:
      updated = updata_sensor(M,false);
      if(updated == true)
      {
        Serial.println(*M);
        updated = false;
        LoopTimer = millis();
        state = 3;
        return false;
      }
    break;
    case 3:
      S.write(0);
      if(millis() - LoopTimer >= T)
      {
        state = 4;
        LoopTimer = millis();
        return false;
      }
    break;
    case 4:
      updated = updata_sensor_R(R,false);
      if(updated == true)
      {
        Serial.println(*R);
        updated = false;
        LoopTimer = millis();
        state = 5;
        return false;
      }
    break;
    case 5:
      S.write(180);
      if(millis() - LoopTimer >= T)
      {
        state = 6;
        LoopTimer = millis();
        return false;
      }
    break;
    case 6:
      updated = updata_sensor_L(L,false);
      if(updated == true)
      {
        Serial.println(*L);
        updated = false;
        LoopTimer = millis();
        state = 7;
        return false;
      }
    break;
    case 7:
      /*Serial.print("Middle  = ");
      Serial.print(*M);
      Serial.print("\tLeft = ");
      Serial.print(*L);
      Serial.print("\tRight = ");
      Serial.println(*R);*/
      S.write(90);
      state = 0;
      LoopTimer = millis();
      return true;
    break;
  }
}


/*void move(CAR_DIRECTIONS_TYPE t , int pwm)
{
  switch (t)
  {
    case FORWARD_CAR_DIR :
      forward(pwm);
    break;
    case BACKWARD_CAR_DIR :
      backward(pwm);
    break;
    case LEFT_CAR_DIR :
      left(pwm);
    break;
    case RIGHT_CAR_DIR :
      right(pwm);
    break;
    case STOP_CAR_DIR :
      stop();
    break;
  }
}*/


bool read_bulltooth(byte *x)
{
  if(Serial.available() >= 0)
  {
    *x = Serial.read();
    return true;
  }else
  {
    return false;
  }
}

/*int read_IR() // From Right to left
{ 
  int IRL2S;
  int IRL1S;
  int IRR1S;
  int IRR2S;
  IRL2S =digitalRead(IRRR);   
  IRL1S =digitalRead(IRRL);
  IRR1S =digitalRead(IRLR);
  IRR2S= digitalRead(IRLL);
  int val= IRL2S+IRL1S*10+IRR1S*100+IRR2S*1000;
  return val;
}*/

bool checksides()
{
  static long LoopTimer = millis();
  static byte s = 0;
  bool k = false;
  bool d = false;
  int M,L,R;
  Serial.println("Inside Check Sides");
  switch(s)
  {
    case 0:
      Serial.println("Inside Case 0 Check sides");
      //k = update_3DIR(&M,&L,&R,1000);
      if(k == true)
      {
        state = 1;
        Reading_State = 0;
        Serial.println("Inside Case 0 Check sides");
      }
    break;
  }
  /*switch (state)
  {
    case 0:
      d = update_3DIR(&ULTRA_Reading,&ULTRA_Reading_L,&ULTRA_Reading_R,1000);
      if(d == true)
      {
        state = 1;
        Serial.println("Inside Case 0 Check sides");
      }
    break;
    case 1:
      Serial.println("Inside Case 1 Check sides");
      Serial.print("Middle  = ");
      Serial.print(ULTRA_Reading);
      Serial.print("\tLeft = ");
      Serial.print(ULTRA_Reading_L);
      Serial.print("\tRight = ");
      Serial.println(ULTRA_Reading_R);
      if(ULTRA_Reading_L > ULTRA_Reading_R)
      {
        S.write(0);
        DIR_US = LEFT_CAR_DIR;
      }else if(ULTRA_Reading_L < ULTRA_Reading_R)
      {
        S.write(180);
        DIR_US = RIGHT_CAR_DIR;
      }else
      {
        S.write(180);
        DIR_US = RIGHT_CAR_DIR;
      }
      if(ULTRA_Reading >= 20)
      {
        DIR_US = NO_ACTION_DIR;
        state = 0;
      }
    break;
  }*/
  
}

void excute_servo_routine(SERVO_STATE_TYPE state,Servo *s) // needed to be called inside a timer function
{
  switch (state)
  {
    case SERVO_NO_ACTION:
    break;
    case SERVO_MIDDLE:
      s->write(90);
    break;
    case SERVO_TURN_LEFT:
      s->write(180);
    break;
    case SERVO_TURN_RIGHT:
      s->write(0);
    break;
  }
}


/*void excute_servo_routine(SERVO_STATE_TYPE state,Servo *s , int T)
{
  static long long LoopTimer = millis();
  static SERVO_STATE_TYPE last_state;
  SERVO_STATE_TYPE temp_state;
  static bool excuted = false;
  static bool exctute_time_delay = false;
  bool change_state = false;
  if(excuted == true)
  {
    if(last_state == state)
    {
      
    }else
    {
      exctute_time_delay = true;
      LoopTimer = millis();
      excuted = false;
    }  
  }else
  {
    last_state = state;
    
    excuted = true;
  }
  if(excute_time_delay == true)
  {
    if(millis() - LoopTimer >= T)
    {
      change_state = true;
      LoopTimer = millis();
      excute_time_delay = false;
    }
  }
  if(change_state == true)
  {
    last_state = state;
    change_state = false;
  }
  switch (last_state)
  {
    case SERVO_MIDDLE:
      s->write(90);
    break;
    case SERVO_TURN_LEFT:
      s->write(180);
    break;
    case SERVO_TURN_RIGHT:
      s->write(0);
    break;
  }
}*/
