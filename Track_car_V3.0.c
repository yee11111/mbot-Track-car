
int trigPin = 12;                  //Trig Pin

long duration, cm, inches;
int R1=A0,R2=A1,Mid=A3,L2=A4,L1=A2;                                    //ledin
double R1Max=500,R2Max=500,MMax=500,L1Max=500,L2Max=500;                //Max             ***********************************************V2.0
double R1min=90,R2min=90,Mmin=90,L1min=90,L2min=90;                     //min
double R1n,R2n,Mn,L1n,L2n;                                              //database normalization
float ffix=0.97,bfix=0.97;                                             //fix num
double hop=0,kp=150,ki=0.001,kd=1000,error=0,np=0;
double pre_error=0,sum_error=0,maxSpeed=250,baseSpeed=230;

/*
180 60 40
70  0.001  1000
speed 100 pid 150 0.001 1000
*/

void font(int sp,float ffix)  //6L5R
{ analogWrite(5, sp);analogWrite(6, sp*ffix); digitalWrite(4, 0);digitalWrite(7, 1); }
void back(int sp,float bfix)
{ analogWrite(5, sp);analogWrite(6, sp*bfix); digitalWrite(4, 1);digitalWrite(7, 0); }
void ltrn(int sp,float ffix)
{ analogWrite(5, sp);analogWrite(6, sp*ffix); digitalWrite(4, 1);digitalWrite(7, 1); }
void rtrn(int sp,float ffix)
{ analogWrite(5, sp);analogWrite(6, sp*ffix); digitalWrite(4, 0);digitalWrite(7, 0); }
void ltrnS(int sp,float ffix)
{ analogWrite(5, 0);analogWrite(6, sp*ffix); digitalWrite(4, 1);digitalWrite(7, 1); }
void rtrnS(int sp,float ffix)
{ analogWrite(5, sp);analogWrite(6, 0); digitalWrite(4, 0);digitalWrite(7, 0); }
void whelstop()
{ analogWrite(5, 0);analogWrite(6, 0); digitalWrite(4, 0);digitalWrite(7, 0); }

void Maxmin_norml_V2()
{ int L2r,L1r,Mr,R1r,R2r;
  L2r=analogRead(L2);  if(L2r>=L2Max){L2Max=L2r;} if(L2r<=L2min){L2min=L2r;} L2n=double(analogRead(L2)-L2min)/(L2Max-L2min); 
  L1r=analogRead(L1);  if(L1r>=L1Max){L1Max=L1r;} if(L1r<=L1min){L1min=L1r;} L1n=double(analogRead(L1)-L1min)/(L1Max-L1min);   
  Mr= analogRead(Mid); if(Mr>=MMax){MMax=Mr;} if(Mr<=Mmin){Mmin=Mr;}         Mn =double(analogRead(Mid)-Mmin)/(MMax-Mmin);    
  R1r=analogRead(R1);  if(R1r>=R1Max){R1Max=R1r;} if(R1r<=R1min){R1min=R1r;} R1n=double(analogRead(R1)-R1min)/(R1Max-R1min); 
  R2r=analogRead(R2);  if(R2r>=R2Max){R2Max=R2r;} if(R2r<=R2min){R2min=R2r;} R2n=double(analogRead(R2)-R2min)/(R2Max-R2min); 
}
void init_V3()
{Serial.println("Inital....."); Maxmin_norml_V2(); int Hs=700,Ls=200,Op=100,Opm=100;
 while(!(analogRead(L2)>Hs&&analogRead(R2)<Ls)) {rtrn(Opm,ffix);Maxmin_norml_V2();}                                        //1xxx0
 while(!(analogRead(L2)<Ls&&analogRead(R2)>Hs)) {ltrn(Opm,ffix);Maxmin_norml_V2();}                                        //0xxx1
 while(analogRead(L2)<Ls&&analogRead(Mid)>Hs&&analogRead(R2)<Ls){rtrn(Opm,ffix);Maxmin_norml_V2();}                        //0x1x0
 whelstop();
}


void take_err() 
{ double d2=2,d1=1,d0=0,dd2=1,dd4=1,dd1=1,dd5=1; int Hs=700,Ls=200; 
if(analogRead(L2)>Hs&&analogRead(L1)>Hs&&analogRead(Mid)>Hs&&analogRead(R1)>Hs&&analogRead(R2)<Ls) {dd4=0;dd5=0; }   //11110
if(analogRead(L2)<Ls&&analogRead(L1)>Hs&&analogRead(Mid)>Hs&&analogRead(R1)>Hs&&analogRead(R2)>Hs) {dd2=0;dd1=0; }   //01111
if(analogRead(L2)<Ls&&analogRead(L1)<Ls&&analogRead(Mid)<Ls&&analogRead(R1)<Ls&&analogRead(R2)<Ls)
{ delay(0.7); if(pre_error>=0){error =3;} else{error =-3;}  // delayMicroseconds(5);
}else
{np=((L2n*-d2*dd1)+(L1n*-d1*dd2)+(Mn*d0)+(R1n*d1*dd4)+(R2n*d2*dd5)); error=np-hop;}
}

void crtl_V5()
{double motorSpeed,leftSpeed,rightSpeed,minSpeed=-250;
   motorSpeed = kp*error + kd*(error - pre_error) + ki*(sum_error);
   leftSpeed = baseSpeed + motorSpeed; rightSpeed = baseSpeed - motorSpeed;   
   if(leftSpeed > maxSpeed) leftSpeed = maxSpeed; if(rightSpeed > maxSpeed) rightSpeed = maxSpeed;
   if(leftSpeed < minSpeed) leftSpeed = minSpeed; if(rightSpeed < minSpeed) rightSpeed = minSpeed;
   if(rightSpeed<0){digitalWrite(7, 0);}else{digitalWrite(7, 1);}
   if(leftSpeed<0) {digitalWrite(4, 1);}else{digitalWrite(4, 0);}
   analogWrite(6,abs(rightSpeed));analogWrite(5,abs(leftSpeed));  
   //Serial.print(error); Serial.print("   "); Serial.print(rightSpeed); Serial.print("   "); Serial.println(leftSpeed);
   pre_error = error;  sum_error += error;
}

void take_dist_V1()
{
  pinMode(trigPin, OUTPUT);   digitalWrite(trigPin, HIGH); delayMicroseconds(5);  digitalWrite(trigPin, LOW);
  pinMode(trigPin, INPUT);  // Serial.println(analogRead(trigPin));          // read echo 
  duration = pulseIn(trigPin, HIGH);   // high time 
  cm = (duration/2) / 29.1; inches = (duration/2) / 74; // trans into cm or inch
  //Serial.print("Distance : ");  Serial.print(inches);Serial.print("in,   ");Serial.print(cm);Serial.print("cm");Serial.println();
  //delay(250);
}

void rond_V1()
{ int cc1=8,cc2=5;
  if(inches<=cc1) { maxSpeed=100; }
  while(cm<=cc2)
  {
   whelstop();
   while(inches<200){ltrnS(100,ffix);}
   font(100,ffix); delay(1);
   rtrnS(100,ffix); delay(1);
  }
  if(cm==0) { maxSpeed=120; }
}


void fontline_V3() {take_dist_V1(); Maxmin_norml_V2(); take_err(); crtl_V5(); }
/***********************************************************************************************setup******************************************************************************************************************************************/
void setup() {
  Serial.begin(9600);
  pinMode(4, OUTPUT);pinMode(5, OUTPUT);pinMode(6, OUTPUT);pinMode(7, OUTPUT); //motor pinput
  pinMode(A0, INPUT); pinMode(A1, INPUT); pinMode(A2, INPUT); pinMode(A3, INPUT); pinMode(A4, INPUT);pinMode(A7, INPUT); //ledin
  pinMode(trigPin, OUTPUT);        //Define inputs and outputs of vol
  hop=0; Maxmin_norml_V2(); take_err(); 
}

void loop() {
  Maxmin_norml_V2(); //take_dist_V1();
  if(analogRead(A7)==0)
  {  //init_V3();
    while(1)
    { 
        if(analogRead(A7)==0) { while(1)  {  fontline_V3();}  }
    }   
  }
 
}

/**************************************************************************************************pinout*******************************************************************************************************************************************************/







/*
 A0----->R1
 A1----->R2
 A2----->L1
 A3----->M
 A4----->L2


out=(x-min)/(max-min)  //database normalization


A0 RJ25 plug 4 (default not-connected)
A1 RJ25 plug 4 (default not-connected)
A2 RJ25 plug 3 ultrasonic
A3 RJ25 plug 3 ultrasonic
A6 light sensor
A7 button
D2 IR RCV
D3 IR TX
D4 DIR2 - direction motor2
D5 PWM2 - pwm motor2
D6 PWM1 - pwn motor1
D7 DIR1 - direction motor1
D8 buzzer
D9 RJ25 plug 2 linefollower
D10 RJ25 plug 2 linefollower
D11 RJ25 plug 1 (default not-connected)
D12 RJ25 plug 1 (default not-connected)
D13 2 WS2812 RGB leds
 */
