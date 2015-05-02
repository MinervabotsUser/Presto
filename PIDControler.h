#ifndef CONTROL_H
#define CONTROL_H
#include "Arduino.h"

class PIDControler
{
  public:
    void Init(float Kp, float Kd, float Ki,float alpha);
    void setConstants(float Kp, float Kd, float Ki);
    float run(float error);
    float integral, derivative, proportional;
    void reset();
  private:
    float p;
    float d, i;
    float lastError;
    unsigned long lastRun;
};

void PIDControler::Init(float Kp, float Kd, float Ki)
{
  this->p = Kp;
  this->d = Kd;
  this->i = Ki;
  lastRun = millis();
}

void PIDControler::setConstants(float Kp, float Kd, float Ki)
{
  
  this->p = Kp;
  this->d = Kd;
  this->i = Ki;
  integral = derivative = proportional = lastError = 0;
  lastRun = millis();
}

float PIDControler::run(float error)
{
  float output;
  
  float dt = (millis () - lastRun) * 0.001;
  lastRun = millis();
  integral += i * error * dt ;
  derivative = d*(error - lastError)/dt;
  lastError = error;
  
  proportional = p * error;
  output = proportional + integral + derivative;
  return output;
}
void PIDControler::reset(){
  lastError = 0;
  integral = 0;
  lastRun = millis();
}
#endif

