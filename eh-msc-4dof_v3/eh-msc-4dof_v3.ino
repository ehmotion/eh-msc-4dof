/*
Thanks to cubexvr! based on https://github.com/cubexvr/servoController

1 period = 0.5us

AASD 15A controller parameters:
Push MOD until you see Pn000. This enters the parameter mode.
Change and check these settings on all motors:

Pn2 = 2 - location control mode

Required params set-up
Pn8 = 300 Internal around are torque limit (CCW)
Pn9 = -300 Around inside the torque limit (the CW)
Pn51 = 1200 (3000) The motor running top speed limit
Pn98 = 10 - pulse multiplicator
Pn109 = 1 - smoothing, 1=fixed smoothing, 2=s-Shaped smoothing
Pn110 = 30 - Smoothing Filter Time
Pn113 = 20 - Feedforward %
Pn114 = 10 - Feedforward Filter Time (ms)
Pn115 = 100 - Gain %
--
Pn97 - reverse the motor direction (0=ccw, 1=cw) Instruction selection logic pulse input direction

Extra parameters needed in the AASD-15A drives are the following:
Pn24 = 100 reach predetermined torque > 100%
Pn52 = 1 SigIn1 port functional allocation  > 1/Servo enable
Pn60 = 2 SigOut1 port functional allocation > 2/Servo ready
Pn61 = 6 SigOut2 port functional allocation > 6/Treach

CN2 control signal connector
 p3 - pwm signal
 p4 - direction
 p5 - GND
 p6 - enable: motor ON
 p9 - 5Vcc
p10 - GND
p11 - ready: motor present and ready
p14 - GND
p23 - torque reach: motor blocked

*/
#include <Arduino.h>
//#define FAILSAFE //not useful if run at full speed
#define NO_ACT_CHECK
// 2ms = 0.5 us * 4000 (can be adapted to needs)
const unsigned int samplePeriod = 4000;

// 20us == 50kHz (should not be changed, faster is not possible)
#define SPD_HOMING  500  //slowest homing
//#define SPD_HOMING  250 //faster homing
#define SPD_FASTST  39
unsigned int minPeriod = SPD_FASTST;
const unsigned int maxPeriod = samplePeriod;

// 3us (should not be changed, timing is critical)
const unsigned int pulseLength = 5;

const unsigned int minPos = 0;
const unsigned int centerPos = 11000;
const unsigned int maxPos = 22000;

//msg format: P4aabbccdd!!
//flypt mover: P4<axis1><axis2><axis3><axis4>!!
const unsigned int startMark1 = 'P';
const unsigned int startMark2 = '4';
const unsigned int endMark1 = '!';
const unsigned int endMark2 = '!';

#define MAX_MOTORS 4
byte motors = 0;

volatile unsigned int targetPos[MAX_MOTORS];
volatile unsigned int currentPos[MAX_MOTORS];
volatile int incPos[MAX_MOTORS];
volatile boolean stopSignal[MAX_MOTORS];
volatile boolean isRunning[MAX_MOTORS];

volatile unsigned int targetInput[MAX_MOTORS];

/* Pins */

// pulse 0 -> cn2 m0 p3
const byte ocr1bPin = 12;
// pulse 1 -> cn2 m1 p3
const byte ocr3bPin = 2;
// pulse 2 -> cn2 m2 p3
const byte ocr4bPin = 7;
// pulse 3 -> cn2 m3 p3
const byte ocr5bPin = 45;

// direction 0 -> cn2 m0 p4
const byte dir0 = A0;//30;
// direction 1 -> cn2 m1 p4
const byte dir1 = A1;//31;
// direction 2 -> cn2 m2 p4
const byte dir2 = A2;//32;
// direction 3 -> cn2 m3 p4
const byte dir3 = A3;//33;

//motor enable 0 -> cn2 m0 p6
const byte ena0p = A4;
//motor enable 0 -> cn2 m1 p6
const byte ena1p = A5;
//motor enable 0 -> cn2 m2 p6
const byte ena2p = A6;
//motor enable 0 -> cn2 m3 p6
const byte ena3p = A7;

//todo: move these to bitfield
volatile byte safe0s = LOW;
volatile byte safe1s = LOW;
volatile byte safe2s = LOW;
volatile byte safe3s = LOW;
volatile byte safeOs = LOW;

//torque-reach 0 -> cn2 m0 p23
const byte safe0p = 18;
//torque-reach 0 -> cn2 m1 p23
const byte safe1p = 19;
//torque-reach 0 -> cn2 m2 p23
const byte safe2p = 20;
//torque-reach 0 -> cn2 m3 p23
const byte safe3p = 21;

#ifdef NO_ACT_CHECK
//motor presence 0 -> cn2 m0 p11
const byte pres0p = A8;
//motor presence 1 -> cn2 m1 p11
const byte pres1p = A9;
//motor presence 2 -> cn2 m2 p11
const byte pres2p = A10;
//motor presence 3 -> cn2 m3 p11
const byte pres3p = A11;

byte pres[] = {pres0p, pres1p, pres2p, pres3p};
#endif

byte ena[] = {ena0p, ena1p, ena2p, ena3p};
byte safe[] = {safe0p, safe1p, safe2p, safe3p};

byte dir[] = {dir0, dir1, dir2, dir3};
byte pulse[] = {ocr1bPin, ocr3bPin, ocr4bPin, ocr5bPin};

// fast digital write for direction pins D30..D33
//#define setDirDown(b) PORTC |= (b)
//#define setDirUp(b) PORTC &=~ (b)
//-using analog pins A0..A3 for dir control
#define setDirDown(b) PORTF |= (b)
#define setDirUp(b) PORTF &=~ (b)
const byte dir0bit = B00000001;
const byte dir1bit = B00000010;
const byte dir2bit = B00000100;
const byte dir3bit = B00001000;

//const byte dir0bit = B10000000;
//const byte dir1bit = B01000000;
//const byte dir2bit = B00100000;
//const byte dir3bit = B00010000;

// faster may be possible, but is not needed
const unsigned long baudRate = 115200;

#define SerialC Serial
#define SerialD Serial3
#define SerialDbg Serial

#define DEBUG 1
#define debug_println(...) \
            do { if (DEBUG) SerialDbg.println(__VA_ARGS__); } while (0)
#define debug_print(...) \
            do { if (DEBUG) SerialDbg.print(__VA_ARGS__); } while (0)
#define debug_printf(...) \
            do { if (DEBUG) SerialDbg.printf(__VA_ARGS__); } while (0)
#define debug2_println(...) \
            do { if (DEBUG==2) SerialDbg.println(__VA_ARGS__); } while (0)
#define debug2_print(...) \
            do { if (DEBUG==2) SerialDbg.print(__VA_ARGS__); } while (0)

/* clock divider for timers /8 = 2MHz*/
const byte PRESCALE = B10;

// for direction pins
const bool UP = LOW;
const bool DOWN = HIGH;

volatile int kntr = 0;
const int led_pin = 3;//LED_BUILTIN;

const int ST_IDLING = 0;
const int ST_HOMING = 1;
const int ST_MAXING = 2;
const int ST_CTRING = 3;
const int ST_RUNING = 4;
volatile byte run_state = ST_IDLING;
const unsigned int POS_HOME = 0;
const unsigned int POS_CNTR = 32000;    //not scaled, full range
const unsigned int POS_MAXI = 64000;    //not scaled, full range
const unsigned int RANGE_SAFE = 20000;  //safe maximum position, for 100mm stroke
const unsigned int RANGE_DZ   = 1000;   //dead zone at the end of the range:
                                        // each 200pos means 1 motor rotation, i.e. 5mm stroke
#define STROKE_LIMIT  100               //100mm limit means we do no homing. use 0 for homing,
                                        // use other values at your own risk(!!!) but check RANGE_SAFE above
const unsigned int HOME_DELAY = 2000;
volatile byte motor_state[] = {ST_IDLING, ST_IDLING, ST_IDLING, ST_IDLING};
volatile unsigned int motor_mins[] = {POS_HOME + RANGE_DZ, POS_HOME + RANGE_DZ,
  POS_HOME + RANGE_DZ, POS_HOME + RANGE_DZ};
volatile unsigned int motor_maxs[] = {RANGE_SAFE - RANGE_DZ, RANGE_SAFE - RANGE_DZ,
  RANGE_SAFE - RANGE_DZ, RANGE_SAFE - RANGE_DZ};

#define HOMING_INC 2

void delay4int(int ms)
{
  for (int i = 0; i < ms; i++)
    delayMicroseconds(1000);
}

void fail_loop()
{
  debug_println ("#sON#");
  //going HIGH, stop everything, faster than a for cycle
  digitalWrite(ena[0], HIGH);
  stopSignal[0] = true; //stop motor
  digitalWrite(ena[1], HIGH);
  stopSignal[1] = true; //stop motor
  digitalWrite(ena[2], HIGH);
  stopSignal[2] = true; //stop motor
  digitalWrite(ena[3], HIGH);
  stopSignal[3] = true; //stop motor
  //
  safeOs = HIGH;
  while (1) //do nothing else here, wait for reset
  {
    //never get out, wait for reset
    digitalWrite(led_pin, HIGH);
    delay4int(100);
    digitalWrite(led_pin, LOW);
    delay4int(100);
  }
}

int motors_count()
{
#ifdef NO_ACT_CHECK
  motors = MAX_MOTORS;
  debug_print ("#ACTs used: ");
  debug_println (motors);
  return MAX_MOTORS;
#endif
  //
  motors = 0;
  //check connection to motors 0..1
  if (digitalRead(pres[0]))//NOK
  {
    debug_println ("#eM0!#");
    //fail_loop();
    debug_print ("#ACTs found: ");
    debug_println (motors);
    return motors;
  }
  motors = 1;
  if (digitalRead(pres[1]))//NOK
  {
    debug_println ("#eM1!#");
    //fail_loop();
    debug_print ("#ACTs found: ");
    debug_println (motors);
    return motors;
  }
  motors = 2;
  if (digitalRead(pres[2]))//NOK
  {
    debug_println ("#eM2!#");
    //fail_loop();
    debug_print ("#ACTs found: ");
    debug_println (motors);
    return motors;
  }
  motors = 3;
  if (digitalRead(pres[3]))//NOK
  {
    debug_println ("#eM3!#");
    //fail_loop();
    debug_print ("#ACTs found: ");
    debug_println (motors);
    return motors;
  }
  motors = 4;
  debug_print ("#ACTs found: ");
  debug_println (motors);
  
  return motors;
}

void motors_check()
{
  if (motors == 0)
    fail_loop();
  //check connection to motors 0..1
  #ifdef NO_ACT_CHECK
  return;
  #endif
  //
  if (digitalRead(pres[0]))//NOK
  {
    debug_println ("#eM0!#");
    fail_loop();
  }
  if (motors == 1)
    return;
  if (digitalRead(pres[1]))//NOK
  {
    debug_println ("#eM1!#");
    fail_loop();
  }
  if (motors == 2)
    return;
  if (digitalRead(pres[2]))//NOK
  {
    debug_println ("#eM2!#");
    fail_loop();
  }
  if (motors == 3)
    return;
  if (digitalRead(pres[3]))//NOK
  {
    debug_println ("#eM3!#");
    fail_loop();
  }
}

#if 0
void int_safetyOVR()
{
  if (safeOs == HIGH)
    return;
  //SerialC.println("#sON!");
  //failure! failure! failure!
  fail_loop();
}
#endif

void int_stop0()
{
  debug_print ("#M0!");
  debug_print (safe0s);
  debug_print ("#");
  if (safe0s == HIGH)
    return;
  safe0s = HIGH;
  //digitalWrite(ena[0], HIGH);
  if (run_state == ST_HOMING)
    stopSignal[0] = true; //stop motor
  #ifdef FAILSAFE
  else
  //if (0 && run_state != ST_HOMING)
  {
    digitalWrite(ena[0], HIGH);
    stopSignal[0] = true; //stop motor
    debug_println ("#M0S!#");
    fail_loop();
  }
  #endif
  //debug_println ("M0SonL");
}

void int_stop1()
{
  debug_print ("#M1!");
  debug_print (safe1s);
  debug_print ("#");
  if (safe1s == HIGH)
    return;
  safe1s = HIGH;
  //digitalWrite(ena[0], HIGH);
  if (run_state == ST_HOMING)
    stopSignal[1] = true; //stop motor
  #ifdef FAILSAFE
  else
  //if (0 && run_state != ST_HOMING)
  {
    digitalWrite(ena[1], HIGH);
    stopSignal[1] = true; //stop motor
    debug_println ("#M1S!#");
    fail_loop();
  }
  #endif
  //debug_println ("M1SonL");
}

void int_stop2()
{
  debug_print ("#M2!");
  debug_print (safe2s);
  debug_print ("#");
  if (safe2s == HIGH)
    return;
  safe2s = HIGH;
  //digitalWrite(ena[0], HIGH);
  if (run_state == ST_HOMING)
    stopSignal[2] = true; //stop motor
  #ifdef FAILSAFE
  else
  //if (0 && run_state != ST_HOMING)
  {
    digitalWrite(ena[2], HIGH);
    stopSignal[2] = true; //stop motor
    debug_println ("#M2S!#");
    fail_loop();
  }
  #endif
  //debug_println ("M2SonL");
}

void int_stop3()
{
  debug_print ("#M3!");
  debug_print (safe3s);
  debug_print ("#");
  if (safe3s == HIGH)
    return;
  safe3s = HIGH;
  //digitalWrite(ena[0], HIGH);
  if (run_state == ST_HOMING)
    stopSignal[3] = true; //stop motor
  #ifdef FAILSAFE
  else
  //if (0 && run_state != ST_HOMING)
  {
    digitalWrite(ena[3], HIGH);
    stopSignal[3] = true; //stop motor
    debug_println ("#M3S!#");
    fail_loop();
  }
  #endif
  //debug_println ("M3SonL");
}

void setup()
{
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);
  //
  noInterrupts();
  initTimers();

  for (int i = 0; i < MAX_MOTORS; i++)
  {
    //enable pins
    pinMode(ena[i], OUTPUT);
    digitalWrite(ena[i], HIGH);
    //pwm pins
    pinMode(pulse[i], OUTPUT);
    digitalWrite(pulse[i], LOW);
    //direction pins
    pinMode(dir[i], OUTPUT);
    digitalWrite(dir[i], UP);
    //digitalWrite(ena[i], LOW); //motors on by default!?!
    //safety pins
    pinMode(safe[i], INPUT_PULLUP);
#ifndef NO_ACT_CHECK
    //motor present pins
    pinMode(pres[i], INPUT_PULLUP);
#endif
    //
    targetPos[i] = 0;
    currentPos[i] = 0;
    incPos[i] = 0;
    isRunning[i] = false;
  }
  //enable all interrupts
  interrupts();
  //
  SerialD.begin(baudRate);
  digitalWrite(led_pin, HIGH);
  SerialC.begin(baudRate);
  if (DEBUG)
    delay4int(250);
  #if 1
  //safeOVRp - safety override
  //!!use RESET signal for failsafe
  //-use safetyOVR as LED/output
  //pinMode(safeOVRp, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(safeOVRp), int_safetyOVR, RISING);
  //
  //attachInterrupt(digitalPinToInterrupt(safe[0]), int_stop0, CHANGE);
  //detect the number of motors
  motors_count();
  //motors = 4;
  //attach interrupts
  if (motors > 0)
    attachInterrupt(digitalPinToInterrupt(safe[0]), int_stop0, FALLING);
  if (motors > 1)
    attachInterrupt(digitalPinToInterrupt(safe[1]), int_stop1, FALLING);
  if (motors > 2)
    attachInterrupt(digitalPinToInterrupt(safe[2]), int_stop2, FALLING);
  if (motors > 3)
    attachInterrupt(digitalPinToInterrupt(safe[3]), int_stop3, FALLING);
  #endif
  //check motors for readiness
  motors_check();
  //ready motors
  for (int i = 0; i < motors; i++)
  {
    digitalWrite(ena[i], LOW); //motors on by default!?!
  }
  //do we do homing or use STROKE_LIMIT?
  if (STROKE_LIMIT > 0)
  {
    //set default stroke for all motors to STROKE_LIMIT
    for (int i = 0; i < MAX_MOTORS; i++)
    {
      motor_state[i] = ST_RUNING;
      motor_mins[i] = 0;
      motor_maxs[i] = STROKE_LIMIT * 200;
      motor_printX (i);
      motor_mins[i] = RANGE_DZ;
      motor_maxs[i] -= RANGE_DZ;
      uint32_t ctr_pos = ((uint32_t)motor_mins[i] + (uint32_t)motor_maxs[i]) / 2;
      //start with the motors already parked, wherever they are, that's the mid-point
      targetPos[i] = ctr_pos;
      currentPos[i] = ctr_pos;
    }
    //check range to be more than a few thousands
    if (motor_maxs[0] - motor_mins[0] < RANGE_SAFE - RANGE_DZ * 2)
      fail_loop();
    //park and ready
    minPeriod = SPD_HOMING;
    if (motors > 0)
      motor_parkX (0);
    if (motors > 1)
      motor_parkX (1);
    if (motors > 2)
      motor_parkX (2);
    if (motors > 3)
      motor_parkX (3);
    debug_print("\n>>");
    debug_print(motors);
    debug_println("ACT ready");
    //cleanup the serial line
    while (SerialC.available())
      SerialC.read();
    digitalWrite(led_pin, HIGH);
    minPeriod = SPD_FASTST;
    run_state = ST_RUNING;
  }
}

void command_check()
{
  static int em1, em2;
  while (SerialC.available() < 1)
  {
    ;
  }
  if (SerialC.read() == startMark1)
  {
    debug2_println("CMDm1");
    while (SerialC.available() < 1)
    {
      ;
    }
    if (SerialC.read() == startMark2)
    {
      while (SerialC.available() < MAX_MOTORS * 2 + 2)
      {
        ;
      }
      targetInput[0] = mapToRangeX(0, SerialC.read() << 8 | SerialC.read());
      targetInput[1] = mapToRangeX(1, SerialC.read() << 8 | SerialC.read());
      targetInput[2] = mapToRangeX(2, SerialC.read() << 8 | SerialC.read());
      targetInput[3] = mapToRangeX(3, SerialC.read() << 8 | SerialC.read());
      //
      em1 = SerialC.read();
      em2 = SerialC.read();
      if (em1 == endMark1 && em2 == endMark2)
      {
        if (motors > 0)
          move0(targetInput[0]);
        if (motors > 1)
          move1(targetInput[1]);
        if (motors > 2)
          move2(targetInput[2]);
        if (motors > 3)
          move3(targetInput[3]);
        //bgood += 8;
      }
    } //startMark2
  } //startMark1
}

void motor_printX (int X)
{
    debug_print ("M");
    debug_print (X, DEC);
    debug_print (" min ");
    debug_print (motor_mins[X], DEC);
    debug_print (" range ");
    debug_print (motor_mins[X] + RANGE_DZ, DEC);
    debug_print ("..");
    debug_print (motor_maxs[X] - RANGE_DZ, DEC);
    //compute stroke len
    uint16_t mtr_range = motor_maxs[X] - motor_mins[X];
    debug_print (" (");
    debug_print (mtr_range, DEC);
    debug_print ("pos/~");
    debug_print (mtr_range/200, DEC);
    uint32_t ctr_pos = ((uint32_t)motor_mins[X] + (uint32_t)motor_maxs[X]) / 2;
    debug_print ("mm) center ");
    debug_println (ctr_pos, DEC);
}

void motor_parkX(int X)
{
  uint32_t ctr_pos = ((uint32_t)motor_mins[X] + (uint32_t)motor_maxs[X]) / 2;
  debug_print ("M");
  debug_print (X, DEC);
  debug_print (" parking at ");
  debug_println (ctr_pos, DEC);
  switch (X)
  {
    case 0:
      targetInput[0] = ctr_pos;
      move0(targetInput[0]);
    break;
    case 1:
      targetInput[1] = ctr_pos;
      move1(targetInput[1]);
    break;
    case 2:
      targetInput[2] = ctr_pos;
      move2(targetInput[2]);
    break;
    case 3:
      targetInput[3] = ctr_pos;
      move3(targetInput[3]);
    break;
  }
}

int homing_start()
{
  debug_println("\n>>start homing");
  //M0
  if (motors > 0)
  {
    targetInput[0] = HOMING_INC;//POS_MAXI; //mapToRange(POS_MAXI);
    motor_state[0] = ST_HOMING;
  }
  //M1
  if (motors > 1)
  {
    targetInput[1] = HOMING_INC;//mapToRange(POS_MAXI);
    motor_state[1] = ST_HOMING;
  }
  //M2
  if (motors > 2)
  {
    targetInput[2] = HOMING_INC;//mapToRange(POS_MAXI);
    motor_state[2] = ST_HOMING;
  }
  //M3
  if (motors > 3)
  {
    targetInput[3] = HOMING_INC;//mapToRange(POS_MAXI);
    motor_state[3] = ST_HOMING;
  }
  //
  if (motors > 0)
    move0(targetInput[0]);
  if (motors > 1)
    move1(targetInput[1]);
  if (motors > 2)
    move2(targetInput[2]);
  if (motors > 3)
    move3(targetInput[3]);
}

//checks limit reached and changes target position fwd or bkwd
int homing_limitX(int X, int stp)
{
  //do we need to bail out first?
  if (targetInput[X] == POS_MAXI || targetInput[X] == POS_HOME)
    return 1;
  //
  switch(X)
  {
    case 0:
      if (safe0s == HIGH)
      {
        safe0s = LOW;
        return 1;
      }
    break;
    case 1:
      if (safe1s == HIGH)
      {
        safe1s = LOW;
        return 1;
      }
    break;
    case 2:
      if (safe2s == HIGH)
      {
        safe2s = LOW;
        return 1;
      }
    break;
    case 3:
      if (safe3s == HIGH)
      {
        safe3s = LOW;
        return 1;
      }
    break;
  }
  //we can move on with step <stp>
  targetInput[X] += stp;
  #if 0
  debug_print ("M");
  debug_print (X, DEC);
  debug_print (" moving at ");
  debug_println (targetInput[X], DEC);
  #endif
  return 0;
}

int homing_checkX(int X)
{
  static byte mmove_try[MAX_MOTORS];
  if (targetInput[X] == POS_MAXI || targetInput[X] == POS_HOME)
  {
    digitalWrite(ena[X], HIGH);//disable motor
    debug_print ("M");
    debug_print (X, DEC);
    debug_print (" position ERROR at ");
    debug_print (targetInput[X], DEC);
    debug_print (", disabled\n");
    motor_state[X] = ST_IDLING;
    //if (motors > X)
    //  motors = X;
    return 1;
  }
  if (motor_state[X] == ST_HOMING)
  {
    if (homing_limitX(X, HOMING_INC))
    {
      mmove_try[X] = 1;
      currentPos[X] = POS_MAXI;
      motor_maxs[X] = currentPos[X];
      debug_print ("M");
      debug_print (X, DEC);
      debug_print (" homing at ");
      debug_println (motor_maxs[X], DEC);
      targetInput[X] = POS_MAXI - HOMING_INC;//mapToRange(POS_HOME);
      //targetInput[X] = POS_MAXI - RANGE_DZ;//mapToRange(POS_HOME);
      //
      motor_state[X] = ST_MAXING;
    }
  }
  else if (motor_state[X] == ST_MAXING)
  {
    if (homing_limitX(X, -HOMING_INC))
    {
      debug_print ("M");
      debug_print (X, DEC);
      debug_print(" maxing at ");
      debug_println (currentPos[X], DEC);
      if (currentPos[X] >= POS_MAXI - RANGE_DZ)
      {
        mmove_try[X]++;
        //try again, keep it moving
        //since the Treach signal may be triggered several times
        //targetInput[X] = POS_HOME + mmove_try[X];//mapToRange(POS_HOME);
        targetInput[X] -= HOMING_INC;//mapToRange(POS_HOME);
      }
      else
      {
        mmove_try[X] = 1;
        motor_mins[X] = currentPos[X];
        motor_printX(X);
        uint16_t mtr_range = motor_maxs[X] - motor_mins[X];
        //adjust range to avoid reaching the mechanical ends
        motor_mins[X] += RANGE_DZ;
        motor_maxs[X] -= RANGE_DZ;
        //check range to be more than a few thousands
        if ((motor_maxs[X] < motor_mins[X]) || (motor_maxs[X] - motor_mins[X] < RANGE_SAFE))
        {
          fail_loop();
        }
        //
        targetInput[X] = motor_mins[X]; //range deadzone already computed
        motor_state[X] = ST_CTRING;
      }
    }//homing limit
  }
  else if (motor_state[X] == ST_CTRING)
  {
    debug_print ("M");
    debug_print (X, DEC);
    debug_print(" centering at ");
    debug_println (currentPos[X], DEC);
    if (currentPos[X] <= motor_mins[X])
    {
      //try again, keep it moving
      //since the Treach signal may be triggered several times
      mmove_try[X]++;
      //new center pos: min + (max - min)/2 = (2min + max - min)/2 = (min + max)/2
      uint32_t ctr_pos = ((uint32_t)motor_mins[X] + (uint32_t)motor_maxs[X]) / 2;
      targetInput[X] = (uint16_t)ctr_pos + mmove_try[X];
    }
    else
    {
      motor_state[X] = ST_RUNING;
    }
  }
  //start motor if needed
  if (motor_state[X] != ST_RUNING)
  {
    switch (X)
    {
      case 0:
        move0(targetInput[0]);
      break;
      case 1:
        move1(targetInput[1]);
      break;
      case 2:
        move2(targetInput[2]);
      break;
      case 3:
        move3(targetInput[3]);
      break;
    }
    //delay4int(HOME_DELAY*2);
  }
  return 1;
}

int homing_done()
{
  int mk = 0;
  //M0
  if (motors > 0 && isRunning[0] == false)
  {
    if (motor_state[0] == ST_RUNING || motor_state[0] == ST_IDLING)
      mk++;
    else
      homing_checkX(0);
  }
  //M1
  if (motors > 1 && isRunning[1] == false)
  {
    if (motor_state[1] == ST_RUNING || motor_state[1] == ST_IDLING)
      mk++;
    else
      homing_checkX(1);
  }
  //M2
  if (motors > 2 && isRunning[2] == false)
  {
    if (motor_state[2] == ST_RUNING || motor_state[2] == ST_IDLING)
      mk++;
    else
      homing_checkX(2);
  }
  //M3
  if (motors > 3 && isRunning[3] == false)
  {
    if (motor_state[3] == ST_RUNING || motor_state[3] == ST_IDLING)
      mk++;
    else
      homing_checkX(3);
  }
  return (mk == motors)?1:0;
  //return (mk == 1)?1:0;
}

//keep led low/high a number of 'loops'
long ledk = 0;
void ledk_act(long loops)
{
  if (ledk < loops)
  {
    //debug_print (ledk);
    //debug_println (" loops, led HIGH");
    digitalWrite(led_pin, HIGH);
  }
  else 
  {
    if (ledk > loops && (ledk % loops) == 0)
    {
      //debug_print (loops);
      //debug_println (" loops done!");
      ledk = 0;
      return;
    }
    //debug_print (ledk);
    //debug_println (" loops, led LOW");
    digitalWrite(led_pin, LOW);
  }
  ledk++;
}

void loop()
{
  motors_check ();
  switch (run_state)
  {
    case ST_IDLING:
      minPeriod = SPD_HOMING;
      run_state = ST_HOMING;
      #if 0
      if (motors > 0)
        motor_printX (0);
      if (motors > 1)
        motor_printX (1);
      if (motors > 2)
        motor_printX (2);
      if (motors > 3)
        motor_printX (3);
      #endif
      homing_start ();
      if (0)
      {
        run_state = ST_RUNING;
        motor_parkX (0);
        motor_parkX (1);
      }
    break;
    case ST_HOMING:
    {
      if (homing_done())
      {
        minPeriod = SPD_FASTST;
        run_state = ST_RUNING;
        if (motors > 0)
          motor_parkX (0);
        if (motors > 1)
          motor_parkX (1);
        if (motors > 2)
          motor_parkX (2);
        if (motors > 3)
          motor_parkX (3);
        debug_print("\n>>");
        debug_print(motors);
        debug_println("ACT ready");
        //cleanup the serial line
        while (SerialC.available())
        {
          //digitalWrite(led_pin, HIGH);
          SerialC.read();
          //delay4int (100);
          //digitalWrite(led_pin, LOW);
          //delay4int (100);
        }
        digitalWrite(led_pin, HIGH);
        ledk = 0;
      }
      else
        ledk_act(20000/HOMING_INC);
    }
    break;
    case ST_RUNING:
      if (SerialC.available())
      {
        //digitalWrite(led_pin, LOW);
        ledk_act (2);
        command_check ();
      }
      else
        ;//digitalWrite(led_pin, HIGH);
    break;
  }
}

uint16_t cmap_uint16 (uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
  uint32_t rv = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (rv > out_max)
    rv = out_max;
  if (rv < out_min)
    rv = out_min;
  debug2_print ("Pm ");
  debug2_print (x, DEC);
  debug2_print (" > ");
  debug2_println (rv, DEC);
  return (uint16_t)rv;
}

//map position request to motor range
//- ditribute the 0..65535 interval to our motor's range
unsigned int mapToRangeX(int X, unsigned int pos)
{
  return cmap_uint16 (pos, POS_HOME, POS_MAXI, motor_mins[X], motor_maxs[X]);
}

unsigned int mapToRange(unsigned int pos)
{
  unsigned int mappedPos = pos / 3;
  if (mappedPos > maxPos)
  {
    return maxPos;
  }
  return mappedPos;
}


void initTimers() {
  // disable everything that we don't need, probably not needed
  PCICR = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
  PCMSK2 = 0;
  WDTCSR = 0;

  // disable 8 bit timers
  TIMSK0 = 0;
  TIFR0 = 0;
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;
  OCR0A = 0;

  TIMSK2 = 0;
  TIFR2 = 0;
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 0;

  // The 16 bit timers that are used for generating the pulses
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = pulseLength;
  TCCR1A = 0;
  TCCR1B = 0;
  ICR1 = 0;
  TIMSK1 = 0;
  TIFR1 = 0;

  TCNT3 = 0;
  OCR3A = 0;
  OCR3B = pulseLength;
  TCCR3A = 0;
  TCCR3B = 0;
  ICR3 = 0;
  TIMSK3 = 0;
  TIFR3 = 0;

  TCNT4 = 0;
  OCR4A = 0;
  OCR4B = pulseLength;
  TCCR4A = 0;
  TCCR4B = 0;
  ICR4 = 0;
  TIMSK4 = 0;
  TIFR4 = 0;

  TCNT5 = 0;
  OCR5A = 0;
  OCR5B = pulseLength;
  TCCR5A = 0;
  TCCR5B = 0;
  ICR5 = 0;
  TIMSK5 = 0;
  TIFR5 = 0;
}

void stopTimer(byte motor) {
  switch (motor) {
    case 0:
      TCCR1B = B00011000;
      TIMSK1 = 0;
      //isRunning[0] = false;
      break;
    case 1:
      TCCR3B = B00011000;
      TIMSK3 = 0;
      isRunning[1] = false;
      break;
    case 2:
      TCCR4B = B00011000;
      TIMSK4 = 0;
      isRunning[2] = false;
      break;
    case 3:
      TCCR5B = B00011000;
      TIMSK5 = 0;
      isRunning[3] = false;
      break;
  }
}

void startTimerWithCrop(byte motor, unsigned int period)
{
  if (period < minPeriod)
  {
    period = minPeriod;
  }
  else if (period > maxPeriod)
  {
    period = maxPeriod;
  }
  startTimer(motor, period);
}

void startTimer(byte motor, unsigned int period)
{
  if (0)
  {
    debug_print("TMR");
    debug_print(motor, DEC);
    debug_print(" per ");
    debug_println(period, DEC);
  }
  switch (motor)
  {
    case 0:
      TCNT1 = period - 1;
      ICR1 = period; // Timer TOP
      TIMSK1 |= (1 << OCIE1A); // oca interrupt when counter = 0
      // fast PWM mode, clear ocr bits on compare match
      TCCR1A = B10101010;
      TCCR1B = B00011000 | PRESCALE;
      break;
    case 1:
      TCNT3 = period - 1;
      ICR3 = period; // Timer TOP
      TIMSK3 |= (1 << OCIE3A); // oca interrupt when counter = 0
      // fast PWM mode, clear ocr bits on compare match
      TCCR3A = B10101010;
      TCCR3B = B00011000 | PRESCALE;
      break;
    case 2:
      TCNT4 = period - 1;
      ICR4 = period; // Timer TOP
      TIMSK4 |= (1 << OCIE4A); // oca interrupt when counter = 0
      // fast PWM mode, clear ocr bits on compare match
      TCCR4A = B10101010;
      TCCR4B = B00011000 | PRESCALE;
      break;
    case 3:
      TCNT5 = period - 1;
      ICR5 = period; // Timer TOP
      TIMSK5 |= (1 << OCIE5A); // oca interrupt when counter = 0
      // fast PWM mode, clear ocr bits on compare match
      TCCR5A = B10101010;
      TCCR5B = B00011000 | PRESCALE;
      break;
  }
}

void move0(unsigned int target) {
  if (targetPos[0] != target) {
    if (isRunning[0]) {
      stopSignal[0] = true;
      while (isRunning[0]) {
        ;
      }
    }
    stopSignal[0] = false;
    unsigned int current = currentPos[0];
    if (target != current) {
      if (target > current) {
        setDirUp(dir0bit);
        incPos[0] = 1;
        targetPos[0] = target;
        isRunning[0] = true;
        startTimerWithCrop(0, samplePeriod / (target - current));
      } else {
        setDirDown(dir0bit);
        incPos[0] = -1;
        targetPos[0] = target;
        isRunning[0] = true;
        startTimerWithCrop(0, samplePeriod / (current - target));
      }
    }
  }
}

void move1(unsigned int target) {
  if (targetPos[1] != target) {
    if (isRunning[1]) {
      stopSignal[1] = true;
      while (isRunning[1]) {
        ;
      }
    }
    stopSignal[1] = false;
    unsigned int current = currentPos[1];
    if (target != current) {
      if (target > current) {
        setDirUp(dir1bit);
        incPos[1] = 1;
        targetPos[1] = target;
        isRunning[1] = true;
        startTimerWithCrop(1, samplePeriod / (target - current));
      } else {
        setDirDown(dir1bit);
        incPos[1] = -1;
        targetPos[1] = target;
        isRunning[1] = true;
        startTimerWithCrop(1, samplePeriod / (current - target));
      }
    }
  }
}
#if 1
void move2(unsigned int target) {
  if (targetPos[2] != target) {
    if (isRunning[2]) {
      stopSignal[2] = true;
      while (isRunning[2]) {
        ;
      }
    }
    stopSignal[2] = false;
    unsigned int current = currentPos[2];
    if (target != current) {
      if (target > current) {
        setDirUp(dir2bit);
        incPos[2] = 1;
        targetPos[2] = target;
        isRunning[2] = true;
        startTimerWithCrop(2, samplePeriod / (target - current));
      } else {
        setDirDown(dir2bit);
        incPos[2] = -1;
        targetPos[2] = target;
        isRunning[2] = true;
        startTimerWithCrop(2, samplePeriod / (current - target));
      }
    }
  }
}

void move3(unsigned int target) {
  if (targetPos[3] != target) {
    if (isRunning[3]) {
      stopSignal[3] = true;
      while (isRunning[3]) {
        ;
      }
    }
    stopSignal[3] = false;
    unsigned int current = currentPos[3];
    if (target != current) {
      if (target > current) {
        setDirUp(dir3bit);
        incPos[3] = 1;
        targetPos[3] = target;
        isRunning[3] = true;
        startTimerWithCrop(3, samplePeriod / (target - current));
      } else {
        setDirDown(dir3bit);
        incPos[3] = -1;
        targetPos[3] = target;
        isRunning[3] = true;
        startTimerWithCrop(3, samplePeriod / (current - target));
      }
    }
  }
}
#endif
// timing is critical, change nothing here
ISR(TIMER1_COMPA_vect, ISR_BLOCK) {
  //debug_println ("M0tmr");
  currentPos[0] += incPos[0];
  if (stopSignal[0] || currentPos[0] == targetPos[0]) {
    TCCR1B = B00011000;
    TIMSK1 = 0;
    isRunning[0] = false;
    if (0)
    {
      debug_print ("M0Etmr ");
      debug_println (currentPos[0], DEC);
    }
  }
}

ISR(TIMER3_COMPA_vect, ISR_BLOCK) {
  currentPos[1] += incPos[1];
  if (stopSignal[1] || currentPos[1] == targetPos[1]) {
    TCCR3B = B00011000;
    TIMSK3 = 0;
    isRunning[1] = false;
    if (0)
    {
      debug_print ("M1Etmr ");
      debug_println (currentPos[1], DEC);
    }
  }
}

#if 1
ISR(TIMER4_COMPA_vect, ISR_BLOCK) {
  currentPos[2] += incPos[2];
  if (stopSignal[2] || currentPos[2] == targetPos[2]) {
    TCCR4B = B00011000;
    TIMSK4 = 0;
    isRunning[2] = false;
  }
}

ISR(TIMER5_COMPA_vect, ISR_BLOCK) {
  currentPos[3] += incPos[3];
  if (stopSignal[3] || currentPos[3] == targetPos[3]) {
    TCCR5B = B00011000;
    TIMSK5 = 0;
    isRunning[3] = false;
  }
}
#endif
