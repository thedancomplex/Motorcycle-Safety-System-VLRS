#include "UM7.h"
UM7 imu;


#define MOTOR_ROT 0
#define MOTOR_LIN 1

#define mL00 0 // left motor front   (mot #1)
#define mL01 1 // left motor back    (mot #2)
#define mR00 2 // right motor front  (mot #4)
#define mR01 3 // right motor back   (mot #5)
#define mR1  4 // right motor in/out (mot #6)
#define mL1  5 // left motor in/out  (mot #3)

/* hysteresis angle */
#define THETA_SENS_1 15.0

/* In/out angle */
#define THETA_SENS_2 30.0

/* enable or disable motor */
/* 0 = disable, 1 = enable */
int FLAG_MOTOR[] = {false, false, false, false, false, false};

/* type of motor */
const int TYPE_MOTOR[] = {MOTOR_ROT, MOTOR_ROT, MOTOR_ROT, MOTOR_ROT, MOTOR_LIN, MOTOR_LIN};

/* PWM pins Left and right */
const int PWM_PINS_L[] = {13, 12, 10, 9, 8, 11};
const int PWM_PINS_R[] = { 7,  6,  4, 3, 2,  5};

/* motor direction */
const double MOTOR_DIR[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

/* motor frame conversion */
const double MOTOR_FRAME[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

/* pid gains */
const double MOTOR_Kp[] = {0.2, 0.2, 0.2, 0.2, 1.0, 1.0};

/* Joint offset */
const double JOINT_OFFSET[] = {10.9, 10.9,    39.0,  39.0, 0.0, 0.0};

/* Joint dir */
const double JOINT_DIR[]    = {1.0,   1.0,      -1.0,  -1.0,  -1.0, 1.0};


/* Analog pin for encoders */
const int ENCODER_PIN[] = {A0, A0, A1, A1, A2, A3};

/* encoer offset (deg) */
const double ENCODER_OFFSET[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/* encoder number of turns */
const double ENCODER_NUM_TURNS[] = {0.75, 0.75, 0.75, 0.75, 1.0, 0.75};

/* encoder base unit (deg) */
const double ENCODER_BASE_UNIT[] = {360.0, 360.0, 360.0, 360.0, 360.0, 360.0};

/* encoder direction */
const double ENCODER_DIR[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};



/* desired theta or length to morot */
double THETA_MOTOR[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/* Actuial motor positions (deg)*/
double MOTOR_POS[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/* motor offset from ground */
const double MOTOR_OFFSET[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};



/* desired pwm  morot */
double PWM_MOTOR[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


const double THETA_SAT_MAX[] = { 46.0,  46.0,   46.0,  46.0,  180.0,  180.0};
const double THETA_SAT_MIN[] = {  7.0,  7.0,     0.1,   0.1, -180.0, -180.0};

/* PWM sat */
#define PWM_MAX 0.39

/* Theta max */
#define THETA_MAX 45.0

/* Min theta parking */
#define THETA_MIN_PARKING -20.0

/* Min theta driving */
#define THETA_MIN 0.0



/* IMU value */
/* Reference to Verticle */
double THETA_IMU = 0.0;

/* Mode Button Pin */
#define MODE_BUTTON_PIN A9

/* Num Joints */
#define NUM_JOINTS 6

/* parking angles */
/* order = R0 (angle), L0 (angle), R1 (in/out), R2(in/out) */
const double PARKING_ANGLES[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void setup() {
  /* IMU Serial Port */
  Serial1.begin(115200);
  Serial.begin(115200);

  /* Set input for mode button */
  pinMode(MODE_BUTTON_PIN, INPUT);

  /* Set mode for Analog */
  for (int i = 0; i < NUM_JOINTS; i++) pinMode(ENCODER_PIN[i], INPUT);

  /* PWM pin modes here */
  for (int i = 0; i < NUM_JOINTS; i++) pinMode(PWM_PINS_L[i], OUTPUT);
  for (int i = 0; i < NUM_JOINTS; i++) pinMode(PWM_PINS_R[i], OUTPUT);
}


double ana2ones(int ana)
{
  double theOut = ((ana - 512) / 512.0 );
  return theOut;
}

int updateMotorPos()
{
  for ( int i = 0; i < NUM_JOINTS; i++) MOTOR_POS[i] = getMotorPos(i);
  return 0;
}

double getMotorPos(int mot)
{
  /* ENCODER RATIO */
  double ratio = ENCODER_BASE_UNIT[mot] * ENCODER_NUM_TURNS[mot] * ENCODER_DIR[mot];

  /* read encoder */
  int enc = analogRead(ENCODER_PIN[mot]);

  double enc_ones = ana2ones(enc);

  double deg = enc_ones * ratio / 2.0 + ENCODER_OFFSET[mot];
  //Serial.println(enc);

  return deg;
}





int ii = 0;
int iii = 0;
int getIMU()
{
  if (Serial1.available() > 0) {
    if (imu.encode(Serial1.read()))
    {
      THETA_IMU = (double)(imu.roll) / 91.222;
      return true;
      // Serial.println(ii++);
    }
  }
  return false;
}

int getMode()
{
  /* gets the mode from the mode button */
  return (int)digitalRead(MODE_BUTTON_PIN);
}

int setPark()
{
  for (int i = 0; i++; i < NUM_JOINTS)
  {
    THETA_MOTOR[i] = PARKING_ANGLES[i];
    FLAG_MOTOR[i] = true;
  }

  return true;
}

int doPark()
{
  /* do parking */
  int ret = 0;
  ret += setPark();
  if (ret > 0 ) ret = 0;
  return 0;
}


double doSafty(double s, double val)
{
  /* saturates val to s */
  if ( val > s) val = s;
  if ( val < -s) val = -s;
  return val;
}

double doThetaSat(int mot, double val)
{
  /* saturate max */
  if (val > THETA_SAT_MAX[mot]) val = THETA_SAT_MAX[mot];
  if (val < THETA_SAT_MIN[mot]) val = THETA_SAT_MIN[mot];

  /* saturate min */
  return val;
}

int setPWM(int mot, double pwm)
{
  int pwml = PWM_PINS_L[mot];
  int pwmr = PWM_PINS_R[mot];
  setPWM_LR(pwml, pwmr, pwm);
  return 1;
}

double doPID(int mot, double theta)
{
  double pwm = 0.0;
  updateMotorPos();
  if (MOTOR_ROT == TYPE_MOTOR[mot])
  {
    //double theta_prime = THETA_IMU*MOTOR_FRAME[mot]+ 0.0;
    double e = theta - MOTOR_POS[mot];
    pwm = e * MOTOR_Kp[mot];
  }
  return pwm;
}

int setMotors()
{
  int ret = 0;
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    /* Saturation theta */
    THETA_MOTOR[i] = doThetaSat(i, THETA_MOTOR[i]);

    /* get desired PWM for motor */
    double pwm = doPID(i, THETA_MOTOR[i]);

    /* disable motor if flag is low */
    if (false == FLAG_MOTOR[i]) pwm = 0.0;

    pwm = doSafty(PWM_MAX, pwm);
    if (i == mL01) Serial.println(pwm);

    /* set pwm */
    ret += setPWM(i, pwm);
  }
  if (ret > 0) ret = 1;
  return ret;
}

int doSetIn(int mot)
{
  /* sets the joint to move in */
  double pwm = PWM_MAX * MOTOR_DIR[mot];
  return setPWM(mot, pwm);
}


int doSetOut(int mot)
{
  /* sets the joint to move out */
  double pwm = PWM_MAX * (-MOTOR_DIR[mot]);
  return setPWM(mot, pwm);
}

int doInOut()
{
  if (THETA_IMU > THETA_SENS_2)
  {
    FLAG_MOTOR[mR00] = false;
    FLAG_MOTOR[mR01] = false;
    FLAG_MOTOR[mR1]  = false;

    FLAG_MOTOR[mL00] = true;
    FLAG_MOTOR[mL01] = true;
    FLAG_MOTOR[mL1]  = true;

    return doSetOut(mL1);
  }
  else if (THETA_IMU < -THETA_SENS_2)
  {
    FLAG_MOTOR[mR00] = true;
    FLAG_MOTOR[mR01] = true;
    FLAG_MOTOR[mR1]  = true;

    FLAG_MOTOR[mL00] = false;
    FLAG_MOTOR[mL01] = false;
    FLAG_MOTOR[mL1]  = false;

    return doSetOut(mR1);
  }
  else
  {
    FLAG_MOTOR[mR00] = true;
    FLAG_MOTOR[mR01] = true;
    FLAG_MOTOR[mR1]  = true;

    FLAG_MOTOR[mL00] = true;
    FLAG_MOTOR[mL01] = true;
    FLAG_MOTOR[mL1]  = true;

    int ret = doSetIn(mR1);
    ret += doSetIn(mL1);
    if (ret > 0) ret = 1;
    return ret;
  }
  return 0;
}

double getThetaArm(int mot)
{
  double theta =  MOTOR_FRAME[mot] * THETA_IMU + MOTOR_OFFSET[mot];
  return theta;
}

int doTrackGround()
{
  if (THETA_IMU > THETA_SENS_1)
  {
    /* enable left, disable right */
    FLAG_MOTOR[mR00] = false;
    FLAG_MOTOR[mR01] = false;

    FLAG_MOTOR[mL00] = true;
    FLAG_MOTOR[mL01] = true;

    THETA_MOTOR[mL00] = getThetaArm(mL00);
    THETA_MOTOR[mL01] = getThetaArm(mL00);
  }
  else if (THETA_IMU < -THETA_SENS_1)
  {
    /* enable left, disable right */
    FLAG_MOTOR[mR00] = true;
    FLAG_MOTOR[mR01] = true;

    FLAG_MOTOR[mL00] = false;
    FLAG_MOTOR[mL01] = false;

    THETA_MOTOR[mR00] = getThetaArm(mR00);
    THETA_MOTOR[mR01] = getThetaArm(mR00);
  }
}

int doTrack()
{
  int ret = 0;
  if ((THETA_IMU < THETA_SENS_1) & (THETA_IMU > -THETA_SENS_1))
  {
    /* Disable all motors */
    for (int i = 0; i < NUM_JOINTS; i++) FLAG_MOTOR[i] = false;
  }
  else
  {
    ret += doTrackGround();
  }

  /* do in/out */
  ret += doInOut();

  if (ret > 0) ret = 0;
  return ret;
}


int setPWM_LR(int LPWM_Output, int RPWM_Output, double s)
{
  // s between -1 and 1
  int sensorValue = (int)( ( (s + 1) / 2.0) * 1023.0);
  if (sensorValue < 512)
  {
    // reverse rotation
    int reversePWM = -(sensorValue - 511) / 2;
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, reversePWM);
  }
  else
  {
    // forward rotation
    int forwardPWM = (sensorValue - 512) / 2;
    analogWrite(LPWM_Output, forwardPWM);
    analogWrite(RPWM_Output, 0);
  }

  return 0;
}



/* ----------------------------------------------- */
/* ----------------------------------------------- */
/* -------------- Start Debug Use Only ----------- */
/* ----------------------------------------------- */
/* ----------------------------------------------- */

int disableAllMotors()
{
  for (int i = 0; i < NUM_JOINTS; i++) FLAG_MOTOR[i] = false;
  return 0;
}

/* ----------------------------------------------- */
/* ----------------------------------------------- */
/* -------------- End Debug Use Only ------------- */
/* ----------------------------------------------- */
/* ----------------------------------------------- */





int doCheck(int mot)
{
  /*read analog here */
  double cal_knob = getMotorPos(mL1) + 90.0;

  //cal_knob = 20;

  double the_max = 45;
  double the_min = 0.0;
  if (cal_knob > the_max) cal_knob = the_max;
  if (cal_knob < the_min) cal_knob = the_min;

/*
  Serial.println();
  Serial.print("Set = ");
  Serial.print(cal_knob);
  Serial.print("  Enc = ");
  Serial.print(getMotorPos(mot));
  Serial.println();
*/
 // THETA_MOTOR[mot] = cal_knob;
  setPos(mot, cal_knob);
  //for (int i = 0; i < NUM_JOINTS; i++) FLAG_MOTOR[i] = false;
  
  
  //FLAG_MOTOR[mot] = true;
  enableMotor(mot);
  return true;

  return 0;
}

int enableMotor(int mot)
{
  FLAG_MOTOR[mot] = true;
  return 0;
}

int setPos(int mot, double pos)
{
  /* Sets your motor position */
  double to_motor = pos*JOINT_DIR[mot] + JOINT_OFFSET[mot];
  Serial.println();
    Serial.print("  joint = ");
  Serial.print(mot);
  Serial.print(" Set = ");
  Serial.print(pos);
  Serial.print(" To Motor = ");
  Serial.print(to_motor);
  THETA_MOTOR[mot] = to_motor;

  Serial.println();
  return 0;
}

void loop() {
  // put your main code here, to run repeatedly:


  /* get IMU data*/
  //if (getIMU())
  if (true)
  {
    /* do parking or tracking based on mode */
    //if(1 == getMode()) doPark();
    //else doTrack();
    disableAllMotors();

    /* Left Test */
    doCheck(mL00);
    doCheck(mL01);

    /* Right Test */
    //doCheck(mR00);
    //doCheck(mR01);
    
    /* disabling all motors for debuggins */
    //disableAllMotors();

    /* Print IMU */
    //Serial.println(THETA_IMU);





    /* set PWM to the motors */
    setMotors();
  }
}
