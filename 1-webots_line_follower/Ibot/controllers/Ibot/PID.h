#ifndef PID_H
#define PID_H

struct Pid
{
  float kp;
  float ki;
  float kd;
  float T;                  // 更新周期
  float setValue = 0.0f;    // 设置值
  float actualValue = 0.0f; // 当前值
  float err = 0.0f;         // 偏差
  float lastErr = 0.0f;     // 上一次的偏差
  float integral = 0.0f;    // 累积误差
};

class PID
{
public:
  PID(float kp, float ki, float kd, int time);
  ~PID();
  
  float Process(float setValue);

private:
  Pid mPid;
};

#endif // PID_H