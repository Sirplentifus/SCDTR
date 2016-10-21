#include <TimerOne.h>

const int pinLed[] = {3,5};
const int pinSensor[] = {0,1};

double integrator[] = {0, 0};
const double timestep = 150000;

double K[2][2] = {{0,0},{0,0}};
double theta[2] = {0,0};

struct params{
  double K, theta;
};

void setup() {
  // put your setup code here, to run once:
  for(int i=0; i<2; i++){
    pinMode(pinLed[i], OUTPUT);
  }
  Serial.begin(9600); 
  Serial.println("Starting system identification");

  struct params aux_params;

  /*for(int i=0; i<2; i++){
    for(int j=0; j<2; j++){
      aux_params = identificationRoutine(pinLed[j], pinSensor[i], 256, 2); //Increase Number of samples maybe
      K[i][j] = aux_params.K;
      theta[i] = theta[i]+aux_params.theta;
    }

    theta[i] = theta[i]/2.0;
  }*/
   
  Serial.print("K = {");
  for(int i=0; i<2; i++){
    for(int j=0; j<2; j++){
        Serial.print(K[i][j],10);

        if(i!=1 || j!=1){
          Serial.print(", ");
        }
        else{
          Serial.print("} ");
        }
          
    }
  }
  Serial.print(" || ");
  Serial.print("Theta = ");
  for(int i=0; i<2; i++){
    Serial.println(theta[i],10);
    if(i!=1){
      Serial.print(", ");
    }
    else{
      Serial.print("} ");
    }    
  }

  Timer1.initialize(timestep);
  Timer1.attachInterrupt(control);
}

void loop() {
  // put your main code here, to run repeatedly:

}

float analogInput2Lux(int analogInput){
  return pow(( 1023.0/(float)analogInput - 1.0),-1.429)*10.0;
}

params identificationRoutine(int pinLed, int pinSensor, int range, int samples){
  double Sv=0; //Sum of v (applied voltage to the led) for every sample
  double Sv2=0; //Sum of v^2 for every sample
  double N=0; //Number of samples (total)
  double Sl=0; //Sum of measured lux
  double Svl=0; //Sum of the product of v with measured lux
  double d_v;
  double measuredLux;
  double det;
  params ret;
  
  for (int v = 0; v<range; v++) {
    d_v = (double)v;
    analogWrite(pinLed,v);
    delay(200);
    for(int j=0; j<samples; j++){
      measuredLux = analogInput2Lux(analogRead(pinSensor));
      Sv = Sv+d_v;
      Sv2 = Sv2+d_v*d_v;
      N = N+1;
      Sl = Sl + measuredLux;
      Svl = Svl + d_v*measuredLux;
    }
  }

  analogWrite(pinLed,0);
  
  det = N*Sv2 - Sv*Sv;
  ret.K = (N*Svl - Sv*Sl)/det;
  ret.theta = (-Sv*Svl + Sv2*Sl)/det;
  Serial.println("One fourth done");
  return ret;
}

void controller(double Lux[], double LuxRef[], int v[]){

  double det = K[0][0]*K[1][1] - K[0][1]*K[1][0];
  
  double FFwd[] = { K[0][0]*(LuxRef[0]-theta[0])/det - K[0][1]*(LuxRef[1]-theta[1])/det,
                   -K[1][0]*(LuxRef[0]-theta[0])/det + K[1][1]*(LuxRef[1]-theta[1])/det};
  
  double FBck[] = { 0.2*(LuxRef[0]-theta[0]), 0.2*(LuxRef[1]-theta[1])};
  
  double ret[] = {0,0};

  int actual_ret[] = {0,0};
  
  for(int i=0; i<2; i++){
    ret[i] = /*FFwd[i] + */FBck[i] + integrator[i];

    if( (ret[i] < 255.0 || (LuxRef[i]-Lux[i])<0) && (ret[i]>0 || (LuxRef[i]-Lux[i])>0)) {
      integrator[i] = integrator[i] + timestep/1000000.0*(LuxRef[i]-Lux[i]);
    }
    
    ret[i] = /*FFwd[i]*0 + */FBck[i] + integrator[i];
  
    if(ret[i] >= 255.0){
      ret[i]  = 255.0;
    }
    
    if(ret[i] <= 0){
      ret[i] = 0;
    }
  
  }

  
  for(int i=0; i<2; i++){
    Serial.print(i);
    Serial.print(": FFwd = ");
    Serial.print(FFwd[i]);
    Serial.print(" || FBck = ");
    Serial.print(FBck[i]);
    Serial.print(" || Intg = ");
    Serial.println(integrator[i]);
  }
  Serial.println("");
  
  for(int i=0; i<2; i++){
    v[i] = round(ret[i]);
  }
}

void control(){
  double L[2], Lref[2]={20.0,20.0};

  for(int i=0; i<2; i++){
    L[i] = analogInput2Lux(analogRead(pinSensor[i]));
  }
    
  int v[2];
  controller(L, Lref, v);

  for(int i=0; i<2; i++){
    analogWrite(pinLed[i],v[i]);
  
    Serial.print(i);
    Serial.print(": Lux = ");
    Serial.println(L[i]);
  }
}

