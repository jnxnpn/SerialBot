/* JX Pan 2014.8
email: 736107120@qq.com*/

#include <AccelStepper.h>
#include <Servo.h> 
/******************************---configuration---******************************/
#define cacheSize  64 
#define aStepPin   5 
#define bStepPin   2 
#define aDrctPin   4 
#define bDrctPin   3 
#define aSwthPin   6  
#define bSwthPin   7  
#define laserPin   8 
#define servoPin   9
#define laserOn    1   // if a high signal on laser pin turns on the laser, write 1 here, 
                       //other wise write -1. 
#define aSlogic    1   // logic for the switches  
#define bSlogic    1  
#define stepsPerRevo_A   3600 
#define stepsPerRevo_B   3600    // for easydrivers with 1/8 microstep
#define segment_mm       20.000  // remenber decimal
#define pi 3.14159265

#define servoStartDeg  81  //integer
#define servoLiftDeg   10  //integer
/***************************---configuration ends----***************************/


AccelStepper A(1,aStepPin,aDrctPin);
AccelStepper B(1,bStepPin,bDrctPin);
int cache[cacheSize];
byte G =0;
byte M =0;
float X =0;
float Y =0;
int F =0;
float E =0;
float curX=-160.000;
float curY=0.000;
float curE=0.000;
float relX=0.00;
float relY=0.00;
float relE=0.00;
float Ycompli = 0.00;
float Xcompli = 0.00;
float Ecompli = 0.00;
boolean cmdRead=false; 
boolean Gcg=false;
boolean Mcg=false;
boolean Xcg=false;
boolean Ycg=false;
boolean Ecg=false;
int curA = stepsPerRevo_A/2;
int curB = -stepsPerRevo_B/4; 
Servo servo;



void setup()
{
  Serial.begin(9600);
  pinMode(laserPin, OUTPUT);
  pinMode(bSwthPin, INPUT);
  pinMode(aSwthPin, INPUT);
  pinMode(bDrctPin, OUTPUT);
  pinMode(aDrctPin, OUTPUT);
  pinMode(bStepPin, OUTPUT);
  pinMode(aStepPin, OUTPUT);

  Serial.println("ready");
     Serial.println("ok");

  A.setMaxSpeed(6000);
  B.setMaxSpeed(6000);
  servo.attach(servoPin);
  servo.write(servoStartDeg+servoLiftDeg);
}

void loop()
{



  for(int i=0; Serial.available()>0; i++){
    cache[i] = Serial.read();
    delay(3);
    cmdRead = true;
  } //get all the raw ascii codes and store them in the cache. 

  if(cmdRead == true){
    for(int i=0; i<cacheSize || (cache[i]!=10 && cache[i+1]!=0); i++){
      if(cache[i]=='G'){
        G = readNum(i);
        Gcg = true;
      } 
      else if(cache[i]=='M'){
        M = readNum(i);
        Mcg = true;
      } 
      else if(cache[i]=='X'){
        X = readNum(i);
        Xcg = true;
      } 
      else if(cache[i]=='Y'){
        Y = readNum(i);
        Ycg = true;
      } 
      else if(cache[i]=='F'){
        F = readNum(i);
      } 
      else if(cache[i]=='E'){
        E = readNum(i);
        Ecg = true;
      }
    }
  }//reads X,Y,F,G,M value,ang gives a bool for each one of them. (if F=0, then it'll be assumed
  // that no value has been received, either there's actually a F value or not. )






  // the following codes identify which command is is received and actually control the machine
  if(Gcg == true){
    switch (G) {
    case 0 :

      Y=curY*relX+Y+Ycompli;
      X=curX*relY+X+Xcompli; 

      if( Xcg == true && Ycg == true){
        toCord( X ,Y,F);
      }
      else if(Xcg == true){
        toCord(X,curY,F);
      }
      else if(Ycg == true){
        toCord(curX,Y,F);
      }
      test();
      break;  //move// G0  -> G1
    case 1 : 


      Y=curY*relX+Y+Ycompli;
      X=curX*relY+X+Xcompli;  

      if( Xcg == true && Ycg == true){
        toCord( X ,Y,F);
      }
      else if(Xcg == true){
        toCord(X,curY,F);
      }
      else if(Ycg == true){
        toCord(curX,Y,F);
      }
      test();
      break;  //move// G1  - Coordinated Movement X Y - E
    case 90: 

      relY = 0.000;
      relX = 0.000;
      relE = 0.000;
      Ycompli = 0;
      Xcompli = 0;
      Ecompli = 0;
      test();
      break;// G90 - Use absolute Coordinates
      
    case 91: 

      relY = 1.000;
      relX = 1.000;
      relE = 1.000;
      test();
      break;      // G91 - Use Relative Coordinates
      
    case 28: 
    

      Home();
      test();
      break; //home all
    case 92: 

      if(Ecg){
        Ecompli=-E;
        relE=1.000;

        if(Xcg){
          Xcompli=-X;
          relX=1.000;
        }
        if(Ycg){
          Ycompli=-Y;
          relY=1.000;
        }// G92 - Set current position to coordinates given
      default: 
        Serial.print("command not supported yet, command received is G");
        Serial.println(G);
        break;
      }
    }
  }
  else if(Mcg == true){
    switch (M) {
    case 82: 

      relE=0.000;
      break;// M82  - Set E codes absolute (default)
    case 83: 

      relE=1.000;
      break;// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
    default:
      Serial.print("command not supported yet, command received is M");
      Serial.println(M);
      break;
    }
  }


  if(Ecg == true){
    E=curE*relE+E+Ecompli;
    if(E>=curE){
      digitalWrite(laserPin, 1 + laserOn);
      servo.write(servoStartDeg);
    } 
    else{
      digitalWrite(laserPin, 1 - laserOn);
      servo.write(servoStartDeg+servoLiftDeg);
    }
  }// e axis controls the laser.  


 
  if(cmdRead==true){
    G=0;
    M=0;
   E=0.000;
   Y=0.000;
    X=0.000;
    F=0;
   Gcg=false;
   Ecg=false;
   Xcg=false;
   Ycg=false;
   Mcg=false;
   cmdRead=false;
   Serial.println("ok");
   for(int i=0; i<cacheSize; i++){
       cache[i]=0;
   }
  }//initialization
}



void toCord(float toX,float toY, unsigned int volicity){
  if (volicity==0){
    volicity=1000;
  }
  while(distance(toX,toY)>1/segment_mm && toCoordinate( curX+(toX-curX)/distance(toX,toY)/segment_mm,curY+(toY-curY)/distance(toX,toY)/segment_mm,volicity)==true){
    
  }
  
  toCoordinate(toX,toY,volicity);




}



boolean num ( int input ) {
  if ( input > 47 &&  input < 58) {
    return true; 
  }
  else if(input == 45 ){
    return true;
  }
  else if(input == 46){
    return true;
  }
  else{
    return 0; 
  }
}//judge if it's a number that's read. input the testee, returns 1 for 0-9, 2 for negative sign,
//3 for decimal point. otherwise returns 0. 







float readNum ( int pos ) {

  char temp[cacheSize];
  for (int poss=0; poss<cacheSize;poss++){
    temp[poss]=0;
  }
  pos++;

  for( int poss=0; pos<cacheSize && num(cache[pos]) ==true ; pos++, poss++){

    temp[poss]=cache[pos];
  }
  return atof(temp);
  
  for (int poss=0; poss<cacheSize;poss++){
    temp[poss]=0;
  }
  
} //start reading number from the input position. untill a number ends. 



void Home (){
  int i=0;
  A.setSpeed(1000);
  while( i < stepsPerRevo_A/1.8 && digitalRead(aSwthPin)!=aSlogic ){
    
    if(A.runSpeed( )==true){
      i++;
    }
  }

  i=0;

  B.setSpeed(1000);
  while( i < stepsPerRevo_B/1.75 && digitalRead(bSwthPin)!=bSlogic ){
    
    if(B.runSpeed()==true){
      i++;
    }
  }



  curX = -160;
  curY = 0; 
  A.setCurrentPosition(0);
  B.setCurrentPosition(0);
}









boolean toCoordinate(float toX, float toY, unsigned int linearSpeed){

  double c = sqrt(pow(toX,2)+pow(toY,2));

  if(c<=160){
      

   float intime= distance(toX,toY)*1000.00/float(linearSpeed);

  
   B.moveTo( (acos(c/2.000/80.000)+atan2(toX,toY))/2.00/pi*stepsPerRevo_B );
   int bToGo=B.distanceToGo();
   A.moveTo( (2.0000*acos(c/2.000/80.000)/2.000/pi+(B.targetPosition()/stepsPerRevo_B-0.25))*stepsPerRevo_A); //Absolute
   int aToGo=A.distanceToGo();
   A.setSpeed(aToGo/(intime/1000.00));
   B.setSpeed(bToGo/(intime/1000.00));
  
 //  int mil=millis();
 //  for(int mill=millis(); mill<mil+intime+1000; mill=millis()){ //pending/abandoned part -- delete it or tinker with it...
   while(A.distanceToGo()!=0 ||  B.distanceToGo()!=0){
     A.runSpeedToPosition();
     B.runSpeedToPosition();
   } 

  
  /*if(abs(aStep)>abs(bStep)){
   int ratio= abs(aStep)/abs(bStep);
   
   for(int i=0; i<bStep; i++){
   
   }
   }else{
   int ratio= abs(bStep)/abs(aStep);
   }    //pending/abandoned part -- delete it or tinker with it...*/


   curA = B.currentPosition() + A.currentPosition() - stepsPerRevo_A/4 ; //comparative to B axis
   curB = B.currentPosition() ;
   curX = toX;
   curY = toY;
   return true;
  }else{return false;
  }
}




double distance(float a, float b){ //from input to cur_x,cur_y
  return sqrt( pow((curX-a),2)+pow((curY-b),2));
}


void test(){ //get rid of the test function here and everywhere else, if not prefered. 
  Serial.print("Command: ");
  if(Gcg == true){
      Serial.print("G");
      Serial.print(G);
      Serial.print(" (XYE)");
      Serial.print(X);
      Serial.print(Y);
      Serial.println(E);
  }
  if(Mcg == true){
      Serial.print("M");
      Serial.print(M);
      Serial.print(" (XYE)");
      Serial.print(X);
      Serial.print(Y);
      Serial.println(E);
  }
  Serial.print("Current Positioin(X,Y,A,B): (");

  Serial.print(curX);
  Serial.print(", ");
  Serial.print(curY);
  Serial.print(", ");
  Serial.print(A.currentPosition());
  Serial.print(", ");
  Serial.println(B.currentPosition());
  Serial.print(A.targetPosition());
  Serial.println(B.targetPosition());

}



