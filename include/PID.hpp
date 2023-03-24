#pragma once

template<typename T>
class PID
{
    private:

    T _p;
    T _i;
    T _d;

    T last_output;

    double _dt;

    public:

    PID()
    {
        _p=0;
        _i=0;
        _d=0;
        last_output=0;
        _dt=0.f;
    }

    PID(T p,T i,T d)
    : _p(p),_i(i),_d(d)
    {
        last_output=0;
        _dt=0.f;
    }

    void setParams(T p,T i=0,T d=0)
    {
        _p=p;
        if(i)
        {
            setI(i);
        }
        if(d)
        {
            setD(d);
        }
    }

    void setP(T p)
    {
        _p=p;
    }

    void setI(T i)
    {
        _i=i;
    }

    void setD(T d)
    {
        _d=d;
    }

    const T& P()
    {
        return _p;
    }

    const T& I()
    {
        return _i;
    }

    const T& D()
    {
        return _d;
    }

    void setTimeStep(double dt)
    {
        _dt=dt;
    }

    const double& TimeStep()
    {
        return _dt;
    }

    T step(T x,double dt)
    {

        last_output=_p*x + _i*x*dt + _d*((last_output-x)/dt);

        return last_output;
    }

    T step(T x)
    {
        return step(x,_dt);
    }

};