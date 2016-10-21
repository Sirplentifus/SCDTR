#include <TimerOne.h>

const int pinLed = 5;
const int pinSensor = 1;
double integrator = 0.0;
const double timestep = 150000;

struct params{
  double k=0,theta=0;
} Params;


void setup() {
  // put your setup code here, to run once:
  pinMode(pinLed, OUTPUT);
  Serial.begin(9600); 
  Serial.println("Starting system identification");
  Params = identificationRoutine(pinLed, pinSensor, 256, 15);

  Serial.print("K = ");
  Serial.print(Params.k,10);
  Serial.print(" || ");
  Serial.print("Theta = ");
  Serial.println(Params.theta,10);

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

  det = N*Sv2 - Sv*Sv;
  ret.k = (N*Svl - Sv*Sl)/det;
  ret.theta = (-Sv*Svl + Sv2*Sl)/det;
  return ret;
}

int controller(double Lux, double LuxRef){
  double FFwd = (1/Params.k)*(LuxRef-Params.theta);
  double FBck = 0.2*(LuxRef-Lux);
  
  double ret = (FFwd+FBck+integrator);
  
  if( (ret < 255.0 || (LuxRef-Lux)<0) && (ret>0 || (LuxRef-Lux)>0)) {
    integrator = integrator + 10.0*timestep/1000000.0*(LuxRef-Lux);
  }
  
  ret = (FFwd+FBck+integrator);

  if(ret >= 255.0){
    ret  = 255.0;
  }
  
  if(ret <= 0){
    ret = 0;
  }
  
  Serial.print("FFwd = ");
  Serial.print(FFwd);
  Serial.print(" || FBck = ");
  Serial.print(FBck);
  Serial.print(" || Intg = ");
  Serial.println(integrator);
  
  return round(ret);
}

void control(){
  double L = analogInput2Lux(analogRead(pinSensor));
  int v = controller(L, 20);
  analogWrite(pinLed,v);

  Serial.print("Lux = ");
  Serial.println(L);
}

