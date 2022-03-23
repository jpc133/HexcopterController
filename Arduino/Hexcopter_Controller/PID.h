//Based on code by Bradley J. Snyder <snyder.bradleyj@gmail.com> available here: https://gist.github.com/bradley219/5373998

class PID{
  private:
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;

  public:  
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    PID(double dt, double max, double min, double Kp, double Kd, double Ki);
  
    // Returns the manipulated variable given a setpoint and current process value
    double Calculate (double setpoint, double pv);
};
