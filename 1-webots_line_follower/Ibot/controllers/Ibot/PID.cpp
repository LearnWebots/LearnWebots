#include "PID.h"

PID::PID(float kp, float ki, float kd, int time)
{
  mPid.kp = kp;
  mPid.ki = ki;
  mPid.kd = kd;
  mPid.T = (time) / 1000.0f;
}

PID::~PID()
{

}

float PID::Process(float setValue)
{
  mPid.setValue = setValue;
  
  // 位置式数字PID算法
  mPid.err = mPid.setValue - mPid.actualValue;
  mPid.integral = mPid.integral + mPid.err;
  mPid.actualValue = mPid.err * mPid.kp + mPid.integral * mPid.T * mPid.ki + (mPid.err - mPid.lastErr) * mPid.kd;
  mPid.lastErr = mPid.err;
  
  return mPid.actualValue;
}
