#include <CVModel2D.hpp>


CVModel2D::StateVector CVModel2D::f(const typename CVModel2D::StateVector& x, const typename CVModel2D::StateVector& u, double dt)
{
    StateVector x_pred = x;
    x_pred(0) += x(2) * dt;
    x_pred(1) += x(3) * dt;
    return x_pred;
}

CVModel2D::StateMatrix CVModel2D::jacobian(const typename CVModel2D::StateVector& x, const typename CVModel2D::StateVector& u, double dt)
{
    StateMatrix F = StateMatrix::Identity();
    F(0,2) = 1.0;
    F(1,3) = 1.0;
    return F;
}

CVModel2D::StateMatrix CVModel2D::ProcessNoise()
{
    return StateMatrix::Identity() * 0.1;
}
