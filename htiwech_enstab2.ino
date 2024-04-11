#include <QTRSensors.h>


const uint8_t SensorCount = 12;
unsigned char sensorPins[] = {25,26,27,14,12,13,23,22,21,19,18,5}; // Broches des capteurs
unsigned int s[SensorCount];
QTRSensorsRC qtr(sensorPins, SensorCount); // Déclaration des broches des capteurs

int position;

//motor
int rightF=32;
int rightR=33;
int leftF=4;
int leftR=15;

int n=0;
//PID
int right_speed,left_speed; 

// float kp=0.1,kd=0.2,ki=0,P,D,I; left=158,right=120
// float kp=0.06,kd=0.08,ki=0,P,D,I;
//float kp=0.06,kd=0.06;
//float kp=3,kd=30,ki=0
//float kp=0.05,kd=0.2,ki=0.,P,D,I;
//float kp=0.4,kd=1,ki=0.,P,D,I;
//float kp=0.4,kd=1.1,ki=0.,P,D,I;
//float kp=0.45,kd=1.2,ki=0.,P,D,I;
//float kp=0.2,kd=0.085,ki=0.,P,D,I;
//float kp=0.25,kd=0.2,ki=0.,P,D,I;
////float kp=0.25,kd=0.3,ki=0.,P,D,I;
//float kp=0.2,kd=0.65,ki=0.,P,D,I;
//0.9/0.105
float kp=0.095,kd=0.8 ,ki=0,P,D,I;
//kp=0.095,kd=0.7 
//kp=0.08,kd=0.117,ki=0,P,D,I;
// kp=0.06,kd=2
//kp=0.1 kd=0.9
//kp=0.5 kd=0.2
//kp=0.08,kd=0.4
//kp=0.08,kd=0.6

float PIDvalue,lasterror,error,error1,lasterror1;
int left_base,right_base;
//ultrason
#define trig 17 
#define echo 39 
#define trig1 16
#define echo1 34
unsigned long current_time,last_time,lunch_time;
int inter=36;
  int led3=1 ;
 int led2=0 ;
 //int led3=32;
int led1=2 ;

 int c1;
 int c2;
 int c3;
 int c4;
 int c5;
 int c6;
 int c7;
 int c8;
 int c9;
 int c10;
 int c11;
 int c12;
 int c13;
 int c14;
 int c15;
 int c16;
 //pid_ultr
float error_ultr,PIDvalue_ultr,lasterror_ultr,integrale,derivative;
float Kp_u=12,Kd_u=100,Ki_u=0;
//Kp_u=15,Kd_u=350,Ki_u=0.00005;
//Kp_u=8.5,Kd_u=350,Ki_u=0;
//Kp_u=10,Kd_u=100,Ki_u=0;
//Kp_u=9.5,Kd_u=95,Ki_u=0;
// Kp_u=15,Kd_u=250,Ki_u=0;
void setup() {
  
Serial.begin(9600);

  //Serial.print(distanceA());

  int tst=0;
  
  //pinMode(led,OUTPUT);
   pinMode(led1,OUTPUT);
   pinMode(led2,OUTPUT);
  pinMode(led3,OUTPUT);
   //moteurs
  pinMode(rightF,OUTPUT);
  pinMode(leftF,OUTPUT);
  pinMode(rightR,OUTPUT);
  pinMode(leftR,OUTPUT);
  //ultrason
    pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
pinMode(trig1,OUTPUT);
  pinMode(echo1,INPUT);
  pinMode(inter,INPUT);
  
 // qtr.setEmitterPin(2);
// forward(234567);
  // pinMode(LED_BUILTIN, OUTPUT);
 // pinMode(led, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(led1, HIGH);
   

   // turn on Arduino's LED to indicate we are in calibration mode
   for (int i = 0 ; i < 200; i++)
  {
    qtr.calibrate();
    tst=1;
    delay(10);
  }
  

   
  digitalWrite(led1, LOW); 

   
   current_time=millis();
   int x=0;
while(1){
  x=digitalRead(inter);
 Serial.println(x);
  if(x==HIGH){
   lunch_time=millis();
     if(lunch_time - current_time>400)
        break;}
}
  
 
   //while(1)
  //{
   // forward(59);
   //PID();
   //forwardPID();  
   //}
  stp(500);
 last_time=millis();
 //forward(100);
 forward(200);
 //int n=9;
   
}
 

void loop() {
 position= qtr.readLine(s);
  // n=14;
 

 int c1=s[0];
 int c2=s[1];
 int c3=s[2];
 int c4=s[3];
 int c5=s[4];
 int c6=s[5];
 int c7=s[6];
 int c8=s[7];
 int c9=s[8];
 int c10=s[9];
 int c11=s[10];
 int c12=s[11];
   

 
int somme=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12;
int somme1=c5+c6+c7+c8;
 int current_time=millis(); 
  
 while(1){
  left_base=140,right_base=140;
  position= qtr.readLine(s);
  // n=14;
 

 int c1=s[0];
 int c2=s[1];
 int c3=s[2];
 int c4=s[3];
 int c5=s[4];
 int c6=s[5];
 int c7=s[6];
 int c8=s[7];
 int c9=s[8];
 int c10=s[9];
 int c11=s[10];
 int c12=s[11];
   PID_cercle();
   forwardPID();

 

 }
 
    


  
  
  
    
  //vitesse(240, 40);
  //stp(3000);
  if(n<=3 ||n==24||n==25||n==28){
    PID();
  forwardPID();
    left_base=190,right_base=190;}
   if(n==26||n==27||n==29||n==30||n==31||n==32||n==33){
    PID();
  forwardPID();
    left_base=140,right_base=140;
   }
   if(n==35||n==36){
     PID();
  forwardPID();
    left_base=190,right_base=190;
   }
   
   if (n==34){
  left_base=190,right_base=190;
  position= qtr.readLine(s);
  // n=14;
 

 int c1=s[0];
 int c2=s[1];
 int c3=s[2];
 int c4=s[3];
 int c5=s[4];
 int c6=s[5];
 int c7=s[6];
 int c8=s[7];
 int c9=s[8];
 int c10=s[9];
 int c11=s[10];
 int c12=s[11];
   

 
int somme=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12;
int somme1=c5+c6+c7+c8;
  PID_specialE();
 //PID_noir();
 forwardPID();
 if (somme <2000){
  forward(5);
 }
 
 
}
   
    
  
  if (n>=4 && n<=23){
    PID();
  forwardPID();
  
    left_base=100,right_base=100;
    }
  //doura1
  if(n==0 && c2+c1>1500 && current_time>500){
    forward(30);
    last_time=current_time;
    n=1;
  }

  //doret2
   if(n==1 && c2+c1>1500 && current_time-last_time>500){
    forward(30);
    //left_base=140,right_base=140;
    last_time=current_time;
    n=2;
   }
   //angle 90 
   if(n==2 && c12+c11+c10>2500 && current_time-last_time>500){
    stp(200);
    //forward_b();
   // stp(100);
    left_pro(20);
    stp(100);
    last_time=current_time;
    n=3;
   }

  //cercle
  if(n==3 && c12+c11>1500 && current_time-last_time>500){
    left_base=100,right_base=100;
    stp(50);
    forward(30);
    last_time=current_time;
    n=4;
   }
   if(n==4 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
   // forward(20);
       left(30);

        last_time=current_time;
    n=5;
   }
   if(n==5 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
        left(30);

    //forward(20);
        last_time=current_time;
    n=6;
   }
   if(n==6 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
        left(30);

    //forward(20);
        last_time=current_time;
    n=7;
   }
     //5arjet cercle
   if(n==7 &&c1+c2>1500 && current_time-last_time >300 ){
    //stp(1000);
    left_pro(50);
        last_time=current_time;
    n=8;
   }
   if(n==8 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
    left(30);
        last_time=current_time;
    n=9;
   }
   if(n==9 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
    //forward(20);
        left(30);

        last_time=current_time;
    n=10;
   }
   if(n==10 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
    //forward(20);
        left(30);

        last_time=current_time;
    n=11;
   }
   //tour2
    if(n==11 &&c1+c2>1500 && current_time-last_time >100 ){
      left_pro(40);
      forward(25);
    
    stp(50);
        last_time=current_time;
    n=12;
   }

   if(n==12 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
   // forward(20);
       left(30);

        last_time=current_time;
    n=13;
   }
   if(n==13 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
        left(30);

    //forward(20);
        last_time=current_time;
    n=14;
   }
   if(n==14 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
        left(30);

    //forward(20);
        last_time=current_time;
    n=15;
   }
     //5arjet cercle
   if(n==15 &&c1+c2>1500 && current_time-last_time >100 ){
   // stp(100);
    left_proX(50);

    n=16;


        last_time=current_time;
    
   }
   ////////tou3
   
     if(n==16 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
   // forward(20);
       left(30);

        last_time=current_time;
    n=17;
   }
   if(n==17 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
        left(30);

    //forward(20);
        last_time=current_time;
    n=18;
   }
   if(n==18 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
        left(30);

    //forward(20);
        last_time=current_time;
    n=19;
   }
     
   if(n==19 &&c1+c2>1500 && current_time-last_time >300 ){
     left_pro(40);
      forward(25);
        last_time=current_time;
    n=20;
   }
   if(n==20 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
    left(30);
        last_time=current_time;
    n=21;
   }
   if(n==21 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
    //forward(20);
        left(30);

        last_time=current_time;
    n=22;
   }
   if(n==22 &&c1+c2>1500 && current_time-last_time >100 ){
    //stp(1000);
    //forward(20);
        left(30);

        last_time=current_time;
    n=23;
   }
   ///5arjet tour3
   if(n==23 &&c1+c2>1500 && current_time-last_time >100 ){
     stp(50);
      right_proX(50);
      digitalWrite(led1, HIGH);
    
    
        last_time=current_time;
    n=24;
   }
     

   if ( n==24 && somme>7000 &&  current_time-last_time >600){
    stp(120);
    left_proX(20);
    stp(50);
    last_time=current_time;
    n=25;}
    ///pause charge
    if ( n==25 && somme>7000 &&  current_time-last_time >700){
      digitalWrite(led1, LOW);
      digitalWrite(led3, HIGH);
     stp(5000);
     digitalWrite(led3, LOW);
     left_proX(20);
     stp(80);
    last_time=current_time;
    n=26 ;}
    //dora ba3d charge
    if ( n==26 && c11+c12>1500 &&  current_time-last_time >400){
       
       stp(120);
     left_proX(20);
     stp(50);

    last_time=current_time;
    n=27 ;}
    //taksira doura
    if ( n==27 && c11+c12+c10+c9>3500 &&  current_time-last_time >400){
       stp(120);
     right_proX(40);
     //stp(50);

    last_time=current_time;
    n=28 ;
    }
    //ba3 lem3awej
  if ( n==28 && somme>6000 &&  current_time-last_time >1000){
       stp(120);
      
     right_proX(30);
    last_time=current_time;
    n=29 ;}
    //angle90
     if ( n==29 && c1+c2>1500 &&  current_time-last_time >100){
       stp(100);
      //forward_b();
     right_proX(70);
     //stp(100);

    last_time=current_time;
    n=30;}
    //gabel marewha
if(n==30 && c1+c2>1500 && current_time-last_time >300){
  stp(100);
    right_proX(20);
    
        last_time=current_time;

    n=31;
   }
   //da5let marewha
   if(n==31 && c6+c7<1500 && current_time-last_time >400){
     stp(100);
    left_proX(20);
   
        last_time=current_time;

    n=32;

   }
   //teksirt l marw7a
   if(n==32 && somme>7000 && current_time-last_time >400){
    forward_b();
    right_proX(40);
    stp(100);
        last_time=current_time;

    n=33;
   }
   if(n==33 && c12+c11>1500 && current_time-last_time >400){
    stp(100);
    forward_b();
    left_proX(20);
    stp(100);
        last_time=current_time;

    n=34;
   }
   if(n==34 && c12+c11+c10>2500 && current_time-last_time>300){
    stp(100);
    //forward_b();
   // stp(100);
    left_pro(20);
    stp(100);
    last_time=current_time;
    n=35;
   }
   if(n==35 && c12+c11+c10>2500 && current_time-last_time>500){
    stp(100);
    //forward_b();
   // stp(100);
    left_pro(20);
    stp(100);
    last_time=current_time;
    n=36;
   }
    if(n==36 && somme>7500 && current_time-last_time>500){
      forward(30);
    stp(5000);
    //forward_b();
   // stp(100);
    
    last_time=current_time;
    n=37;
   }

   
   


   
   

  


   
    
 

   

  
 





}

void PID(){
   
  position = qtr.readLine(s);
 
  error=position-5500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(255,max(left_speed,0));
  right_speed=min(255,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void pidforcee(){
  position = qtr.readLine(s);
  if(position<2000){left(1);}
  else if (position>13000){right(1);} 
 else{ error=position-7500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0)); } 
}
void PID_noir(){
  position = qtr.readLine(s,QTR_EMITTERS_ON,1,true);
  error=position-5500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  left_speed=left_base-PIDvalue;
  right_speed=right_base+PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void PID_special(){
  bool v=true;
  position = qtr.readLine(s,QTR_EMITTERS_ON,0,v);
  error=position-5500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void forwardPID(){
  
  analogWrite(rightF,right_speed);
   analogWrite(leftF,left_speed);
  analogWrite(rightR,0);
  analogWrite(leftR,0);

}
void forward(int x){
  
  analogWrite(rightF,160);
   analogWrite(leftF,160);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}

void left(int x){
  
  analogWrite(rightF,100);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}






void right(int x){
  
  analogWrite(rightF,0);
  analogWrite(leftF,160);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void left_fblstou(int x){
  
  analogWrite(rightF,140);
  analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,140);
  delay(x);
}
void left_fblstou1(int x){
  
  analogWrite(rightF,70);
  analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,70);
  delay(x);
}
void right_fblstou(int x){
  
  analogWrite(rightF,0);
  analogWrite(leftF,140);
  analogWrite(rightR,140);
  analogWrite(leftR,0);
  delay(x);
}
void stp(long int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void back(int x){
   analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,100);
  analogWrite(leftR,100);
  delay(x);
}
void PID_speed(){
  position = qtr.readLine(s);
  error=position-7000;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void PID_chlot(){
  
  position = qtr.readLineM(s);
  error=position-3500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(255,max(left_speed,0));
  right_speed=min(255,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void PID_noirE(){
  position = qtr.readLineM(s,QTR_EMITTERS_ON,1,false);
  error=position-3500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  left_speed=left_base-PIDvalue;
  right_speed=right_base+PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void PID_specialE(){
  bool v=false;
  position = qtr.readLine(s,QTR_EMITTERS_ON,0,v);
  error=position-5500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void right_proX(int x){
  right_fblstou( x);
  int somme1=c5+c6+c7+c8;
  while(somme1<2500){
     position= qtr.readLine(s);
  
 
 int c1=s[0];
 int c2=s[1];
 int c3=s[2];
 int c4=s[3];
 int c5=s[4];
 int c6=s[5];
 int c7=s[6];
 int c8=s[7];
 int c9=s[8];
 int c10=s[9];
 int c11=s[10];
 int c12=s[11];
 int c13=s[12];
 int c14=s[13];
 int c15=s[14];
 int c16=s[15];
 somme1=c5+c6+c7+c8;
    right_fblstou(x/6);
    stp(10);}
    
  
}
int distance(){
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
int  duration = pulseIn(echo, HIGH);
 int distanceCm = duration * 0.034 / 2;
  int distanceInch = duration * 0.0133 / 2;
return distanceCm;
}
int distance_A(){
 
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
int  duration = pulseIn(echo1, HIGH);
 int distanceCm = duration * 0.034 / 2;
  int distanceInch = duration * 0.0133 / 2;
return distanceCm;
}

/*
# Paramètres PID
Kp = 1.0
Ki = 0.1
Kd = 0.05

# Variables PID
error_prior = 0
integral = 0

# Boucle principale
while True:
    # Lecture de la distance à l'obstacle
    distance = ultrasonic_sensor.get_distance()
    
    # Calcul de l'erreur
    error = distance - distance_cible
    
    # Calcul des termes PID
    integral = integral + error
    derivative = error - error_prior
    
    # Calcul de la commande
    output = Kp * error + Ki * integral + Kd * derivative
    
    # Limitation de la commande pour éviter les saturations
    output = min(max(output, -max_speed), max_speed)
    
    # Contrôle des moteurs
    motor_controller.set_speed(output, output)
    
    # Mise à jour de l'erreur précédente
    error_prior = error*/
void pid_ultr(){
error_ultr = distance()-12;

integrale = integrale + error_ultr;

derivative = error_ultr - lasterror_ultr;

PIDvalue_ultr = (Kp_u*error_ultr) + (Ki_u*integrale )+ (Kd_u*derivative);
 lasterror_ultr = error_ultr;
  left_speed=left_base+PIDvalue_ultr;
  right_speed=right_base-PIDvalue_ultr;
  left_speed=min(100,max(left_speed,0));
  right_speed=min(100,max(right_speed,0));
}
void forwardPID_ultr(){
  
  analogWrite(rightF,right_speed);
   analogWrite(leftF,left_speed);
  analogWrite(rightR,0);
  analogWrite(leftR,0);

}
void vitesse(int x,int y){
  left_base=x,right_base=x;
  while(left_base>140){
     

    PID();
    forwardPID();
    left_base--;
    right_base=left_base;
    delay(y);
    

  }
}
void left_pro(int x){
  left_fblstou1( x);
  int somme1;
  position= qtr.readLine(s);
  
 
 int c1=s[0];
 int c2=s[1];
 int c3=s[2];
 int c4=s[3];
 int c5=s[4];
 int c6=s[5];
 int c7=s[6];
 int c8=s[7];
 int c9=s[8];
 int c10=s[9];
 int c11=s[10];
 int c12=s[11];
 int c13=s[12];
 int c14=s[13];
 int c15=s[14];
 int c16=s[15];
 somme1=c8+c7;
  while(somme1<1200){
     position= qtr.readLine(s);
  
 
 int c1=s[0];
 int c2=s[1];
 int c3=s[2];
 int c4=s[3];
 int c5=s[4];
 int c6=s[5];
 int c7=s[6];
 int c8=s[7];
 int c9=s[8];
 int c10=s[9];
 int c11=s[10];
 int c12=s[11];

 somme1=c5+c6+c7+c8;
    left_fblstou1(x/8);}
    stp(20);
    
  
}
void left_prot(int x){
  left_fblstou1( x);
  int somme;
  position= qtr.readLine(s);
  
 
 int c1=s[0];
 int c2=s[1];
 int c3=s[2];
 int c4=s[3];
 int c5=s[4];
 int c6=s[5];
 int c7=s[6];
 int c8=s[7];
 int c9=s[8];
 int c10=s[9];
 int c11=s[10];

 somme=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12;
  while(somme<10000){
     position= qtr.readLine(s);
  
 
 int c1=s[0];
 int c2=s[1];
 int c3=s[2];
 int c4=s[3];
 int c5=s[4];
 int c6=s[5];
 int c7=s[6];
 int c8=s[7];
 int c9=s[8];
 int c10=s[9];
 int c11=s[10];
 int c12=s[11];

 somme=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12;
    left_fblstou1(x/6);};
    stp(15);
    
  
}
void stp1(){
  
  analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  
}
void forward_b(){
  int somme1 =c5+c6+c7+c8 ;
  while (somme1>2500){
     position= qtr.readLine(s);
  
 
 int c1=s[0];
 int c2=s[1];
 int c3=s[2];
 int c4=s[3];
 int c5=s[4];
 int c6=s[5];
 int c7=s[6];
 int c8=s[7];
 int c9=s[8];
 int c10=s[9];
 int c11=s[10];
 int c12=s[11];
 forward(5);
 }

}
void left_proX(int x){
  left_fblstou( x);
  int somme1;
  position= qtr.readLine(s);
  
 
 int c1=s[0];
 int c2=s[1];
 int c3=s[2];
 int c4=s[3];
 int c5=s[4];
 int c6=s[5];
 int c7=s[6];
 int c8=s[7];
 int c9=s[8];
 int c10=s[9];
 int c11=s[10];
 int c12=s[11];
 int c13=s[12];
 int c14=s[13];
 int c15=s[14];
 int c16=s[15];
 somme1=c8+c7;
  while(somme1<1200){
     position= qtr.readLine(s);
  
 
 int c1=s[0];
 int c2=s[1];
 int c3=s[2];
 int c4=s[3];
 int c5=s[4];
 int c6=s[5];
 int c7=s[6];
 int c8=s[7];
 int c9=s[8];
 int c10=s[9];
 int c11=s[10];
 int c12=s[11];

 somme1=c7+c8;
    left_fblstou(x/6);}
    stp(20);
    
  
}
void PID_cercle(){
   
  position = qtr.readLines(s);
 
  error=position-5500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  


  
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(255,max(left_speed,0));
  right_speed=min(255,max(right_speed,0));
  //delay(200);
  
  //Serial.println(error);
}