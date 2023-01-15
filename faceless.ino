#define left_motor_positive 8
#define left_motor_negative 7
#define right_motor_positive 4
#define right_motor_negative 2

#define en1 6 //left
#define en2 3  

void forward(int,int);
void left(int);
void right(int);
void sharpturn();
void stopmotor();
void leds();
void pid();


int P, D, I=0, previousError=0, PIDvalue, error;
int lsp, rsp;
int lfspeed = 100;

int led=9;

float Kp = 0;
float Kd = 0;
float Ki = 0 ;


int minValues[6], maxValues[6], threshold[6];

void setup()
{
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(left_motor_positive,OUTPUT);
  pinMode(left_motor_negative,OUTPUT);
  pinMode(right_motor_positive,OUTPUT);
  pinMode(right_motor_negative,OUTPUT);
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);
}


void loop()
{
  delay(2000);
  calibrate();
  delay(4000);

  while (1)
  {
    //If Left Found
    if ((analogRead(5) + analogRead(4)) < (threshold[5] + threshold[4]) && (analogRead(0) + analogRead(1)) > (threshold[0] + threshold[1] ))
    {
      lsp = 0;
      rsp = lfspeed;
      do{
        left(rsp);
      }while(analogRead(3) > threshold[3] ||  analogRead(4) > threshold[4]);
      
    }
    //If Right Found
    else if ((analogRead(5) + analogRead(4)) > (threshold[5] + threshold[4]) && (analogRead(0) + analogRead(1)) < (threshold[0] + threshold[1] ))
    { 
      //To check left is present or not
      for(int i=0;i<100;i++)
      {
        if((analogRead(5) + analogRead(4)) < (threshold[5] + threshold[4]))
        {
          lsp = 0;
          rsp = lfspeed;
          do{
              left(rsp);
             }while(analogRead(3) > threshold[3] ||  analogRead(4) > threshold[4]);
          break;
        }
      }
      //If straight is found
      if((analogRead(2) + analogRead(3)) < (threshold[2] + threshold[3] ))
      {
        pid();
      }
      else
      {
        lsp = lfspeed;
        rsp = 0;
        do{
          right(lsp);
        }while(analogRead(3) > threshold[3] ||  analogRead(4) > threshold[4]);
        
      }

    }
    //If No line Found
    else if(analogRead(5)>threshold[5]&&analogRead(4)>threshold[4]&&analogRead(3)>threshold[3]&&analogRead(2)>threshold[2]&&analogRead(1)>threshold[1]&&analogRead(0)>threshold[0])
    {
      do{
          sharpturn();
        }while(analogRead(3) > threshold[3] ||  analogRead(4) > threshold[4]);
    }
    //if T section is found
    else if(analogRead(5)<threshold[5]&&analogRead(4)<threshold[4]&&analogRead(3)<threshold[3]&&analogRead(2)<threshold[2]&&analogRead(1)<threshold[1]&&analogRead(0)<threshold[0])
    {
      if(analogRead(5)<threshold[5]&&analogRead(4)<threshold[4]&&analogRead(3)<threshold[3]&&analogRead(2)<threshold[2]&&analogRead(1)<threshold[1]&&analogRead(0)<threshold[0])
      {
        stopmotor();  //end maze
      }
      else
      {
        lsp = 0;
      rsp = lfspeed;
      do{
        left(rsp);
      }while(analogRead(3) > threshold[3] ||  analogRead(4) > threshold[4]);
      }
      
    }
    else if ((analogRead(2) + analogRead(3)) < (threshold[2] + threshold[3] ))
    {
      pid();
    }
  }
}
void pid()
{
      Kp = 0.00055* ( 0-(analogRead(2)+analogRead(3))/2);
      Kd = 10 * Kp;
      //Ki = 0.0001;
      linefollow();
}
void linefollow()
{
  int error = (analogRead(4) - analogRead(1));

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
 forward(lsp,rsp);

}

void calibrate()
{
  for ( int i = 0; i < 6; i++)
  {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  
  for (int i = 0; i < 3000; i++)
  {
    sharpturn();

    for ( int i = 0; i < 6; i++)
    {
      if (analogRead(i) < minValues[i])
      {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i])
      {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for ( int i = 0; i < 6; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
 stopmotor();
}

void forward(int spd1, int spd2)
{

  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, HIGH);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
}

void right(int spd)
{

  analogWrite(en1, spd);
  analogWrite(en2, 0);
  digitalWrite(left_motor_positive, HIGH);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, LOW);
}

void left(int spd)
{

  analogWrite(en1, 0);
  analogWrite(en2, spd);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
}
void sharpturn()
{
  analogWrite(en1, 100);
  analogWrite(en2, 100);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, HIGH);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
}
void stopmotor()
{
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, LOW);
}
void leds()
{
  digitalWrite(led,HIGH);
  delay(2000);
  digitalWrite(led,LOW);
}
