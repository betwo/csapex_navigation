#ifndef PID_HPP
#define PID_HPP

/// SYSTEM
#include <cmath>
#include <chrono>

namespace csapex
{
template <class T>
class PID
{
public:
    PID(const double fKp, const double fKi, const double fKd)
        : e_(0.0f),
          integral_(0.0f),
          kp_(fKp),
          ki_(fKi),
          kd_(fKd)
    {
        last_time_ = std::chrono::high_resolution_clock::now();
    }

    PID()
        : PID(1.0, 0.0, 0.0)
    {
    }

    T regulate(const T& e)
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now - last_time_;

        double dt = std::chrono::duration_cast<std::chrono::microseconds>(duration).count() * 1e-6;

        last_time_ = now;
        T last_e = e_;

        e_  = e;
        integral_ += e_ * dt;

        return  kp_ * e_ + ki_ * integral_  + kd_ * (e_ - last_e) / dt;
    }

    void setKP(double p)
    {
        kp_ = p;
    }
    void setKI(double i)
    {
        ki_ = i;
    }
    void setKD(double d)
    {
        kd_ = d;
    }

private:
    T e_;
    T integral_;

    // coefficients
    double kp_;
    double ki_;
    double kd_;

    std::chrono::high_resolution_clock::time_point last_time_;
};

}

#endif // PID_HPP
