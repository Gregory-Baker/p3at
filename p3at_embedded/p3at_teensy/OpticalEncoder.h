#pragma once

#include <Encoder.h>
#include <std_msgs/Float32.h>

const boolean LEFT = 0;
const boolean RIGHT = 1;

class OpticalEncoder {

  private:
  
    Encoder* encoder;
    boolean wheelLR;
    const long ticksPerRev;
    
    long lastRead;
    unsigned long lastReadTime;
    double velocity;
    double angle;
    const int k;
  
    
  public:
  
    OpticalEncoder(const int, const int, const boolean, const long);
    long GetCount();
    float GetVelocity();
    float GetAngle();  
};

OpticalEncoder::OpticalEncoder(const int A_PIN, const int B_PIN, boolean wheelLR, const long ticksPerRev)
  : encoder(new Encoder(A_PIN, B_PIN))
  , wheelLR(wheelLR)
  , ticksPerRev(ticksPerRev)
  , k(wheelLR == LEFT? -1 : 1)
{
  lastRead = encoder->read();
  lastReadTime = millis();
}

float OpticalEncoder::GetVelocity() {
  long ticks = encoder->read();
  long tickChange = (ticks - lastRead);
  float revsTurned = (float)tickChange / (float)ticksPerRev; // Number of revolutions turned since last read
  float time_step = millis() - lastReadTime;
  velocity = k * 2 * PI * revsTurned / (time_step/1000); // velocity of wheel (rad/s)
  lastRead = ticks;
  lastReadTime = millis();
  return velocity;
}

long OpticalEncoder::GetCount() {
  return encoder->read();
}

float OpticalEncoder::GetAngle() {
  angle = k*2*PI*GetCount()/(float)ticksPerRev;
  return angle;
}

//void OpticalEncoder::PublishVelocity(){
//  msg.data = velocity;
//  pub.publish(&msg);
//}
//
//void OpticalEncoder::PublishAngle(){
//  msgAng.data = angle;
//  pubAng.publish(&msgAng);
//}
