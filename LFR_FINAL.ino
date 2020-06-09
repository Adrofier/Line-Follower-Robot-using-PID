//=====Pin Declaration=====//
//left to right
#define IR1 6
#define IR2 7
#define IR3 8
#define IR4 9
#define IR5 12
#define IR6 13

#define L2 2
#define L1 3
#define R2 4
#define R1 5

#define ENA 10
#define ENB 11

#define FWD 1
#define FWDL 0
#define FWDR 14
#define REVL 15


//=====PID values=====//
float P = 11.5;
float D = 0;
float I = 0;
float error = 0 ;
float oldErrorP = 0;
float oldErrorI = 0;
float totalError;


int baseSpeed = 60; //motor initial speed
float LMS;
float RMS;
float a;
float b;
float c;
float d;
float e;
float f;

int sensor[6] = {0, 0, 0, 0, 0, 0}; //array to store sensor readings


//=====Setup=====//
void setup()
{
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);

  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  LMS = baseSpeed;
  RMS = baseSpeed;

  analogWrite(ENA, LMS);
  analogWrite(ENB, RMS);

}


//=====Destroy=====//
void loop()
{
  readSensors();
  calculateError();
  PID();

}


//=====Sensor Readings=====//
void readSensors()
{
  sensor[0] = digitalRead(IR1);
  sensor[1] = digitalRead(IR2);
  sensor[2] = digitalRead(IR3);
  sensor[3] = digitalRead(IR4);
  sensor[4] = digitalRead(IR5);
  sensor[5] = digitalRead(IR6);
}


//=====Error Calculation=====//
void calculateError()
{
  //WHITE AND DISCONTINUITY
  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1)
    error = 0;

  //PATH SPLIT
  if (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 0)
    error  = 0;

  //T-POINT TO BE IGNORED
  if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0 && sensor[5] == 0)
    error = 0.1;

  //90 DEGREE
  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0 && sensor[5] == 0)
    error = 12;

  //90 DEGREE
  if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1 && sensor[5] == 1)
    error = -9;

  //90 DEGREE
  if (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0 && sensor[5] == 0)
    error = 10;

  //90 DEGREE
  if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0 && sensor[5] == 1)
    error  = -9;

  //ACUTE ANGLE
  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1 && sensor[5] == 0)
    error  = 19;
  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 0)
    error  = 19;
  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 1 && sensor[5] == 0)
    error  = 19;
  if (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 1 && sensor[5] == 1)
    error  = 19;

  //ACUTE ANGLE
  if (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1 && sensor[5] == 1)
    error  = -16;
  if (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 1 && sensor[5] == 1)
    error  = -16;
  if (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1)
    error  = -16;
  //arrow
  if (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1 && sensor[5] == 0)
  error  = 0;
  if (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0 && sensor[5] == 1)
    error  = 0;
  //SETPOINT
  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1 && sensor[5] == 1)
    error = 0;

  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 0)
    error = 15;

  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 0 && sensor[5] == 0)
    error = 5;

  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 0 && sensor[5] == 0)
    error = 5;

  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 0 && sensor[5] == 1)
    error = 8;

  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 0 && sensor[5] == 1)
    error = 4;

  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0 && sensor[5] == 1)
    error = 3;

  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 1 && sensor[5] == 1)
    error = 3;

  if (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1 && sensor[5] == 1)
    error = -3;

  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1)
    error = -2;

  if (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1)
    error = -4;

  if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1)
    error = -5;

  if (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1)
    error = -5;

  if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1)
    error = -5;

  if (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1)
    error = -15;

  return;
}

void motor(int dir)
{
  if (dir == FWD)
  {
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
  }

  if (dir == FWDL)
  {
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
  }
  if (dir == FWDR)
  {
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
  }
}

void PID()
{
  float errorP = error;
  float errorD = error - oldErrorP;
  float errorI = I + oldErrorI;

  totalError = (P * errorP) + (D * errorD) + (I * errorI);

  oldErrorP = errorP;
  oldErrorI = errorI;

  LMS = baseSpeed + totalError;
  RMS = baseSpeed - totalError;

  if (LMS >= 0 && RMS >= 0)
  { e = LMS;
    f = RMS;

    constrain(e, 0, (baseSpeed));
    constrain(f, 0, (baseSpeed));
    analogWrite(ENA, e);
    analogWrite(ENB, f);
    motor(FWD);
  }
  if ((LMS >= 0) && (RMS < 0))
  { a = abs(RMS);
    b = abs(LMS);
    constrain(b, 0, (baseSpeed ));
    constrain(a, 0, (baseSpeed ));
    analogWrite(ENA, b);
    analogWrite(ENB, a);
    motor(FWDL);
  }
  if ((LMS < 0) && (RMS >= 0))
  { c = abs(LMS);
    d = abs(RMS);
    constrain(c, 0, (baseSpeed ));
    constrain(d, 0, (baseSpeed ));
    analogWrite(ENA, c);
    analogWrite(ENB, d);
    motor(FWDR);
    if (error == (19))
      delay(400);
    if (error == (-16))
      delay(300);
    if (error == 0.1)
      delay(200);
  }
}
