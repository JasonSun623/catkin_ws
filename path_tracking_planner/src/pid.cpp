//
// Created by kadupitiya on 10/10/18.
//

#include "pid.h"

#include <iostream>
#include <cmath>
using namespace std;

class PIDImpl
{
public:
    PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
    ~PIDImpl();
    double calculate( double setpoint, double pv );
    double preError(){return _pre_error;}
private:
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;

    double err, err_1, err_2, uk_last;
};


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID()
{
    delete pimpl;
}

double PID::preError(){
  return pimpl->preError();
}
/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
        _dt(dt),
        _max(max),
        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        err(0),err_1(0),err_2(0),uk_last(0),
        _pre_error(0),
        _integral(0)
{
}

double PIDImpl::calculate( double setpoint, double pv )
{


/*
    //error
    double error = setpoint - pv;

    // Proportional portion
    double Pout = _Kp * error;

    // Integral portion
    _integral += error * _dt;
    double Iout = _Ki * _integral;
    //Iout = 0.0;
    // Derivative portion
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

 */

    err_2 = err_1;
    err_1 = err;
    err = setpoint - pv;

    double Pout = _Kp * (err - err_1);
    double Iout = _Ki * err;
    double Dout = _Kd * (err - 2.0f * err_1 + err_2);
    // Total output
    double output = Pout + Iout + Dout + uk_last;

    uk_last = setpoint;

    // Limit to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
//    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}
