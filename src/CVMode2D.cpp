#include <MotionModelBase.hpp>

class CVModel2D : public MotionModelBase<4> {
public:
    StateVector f(const StateVector& x) const override {
        StateVector x_pred = x;
        x_pred(0) += x(2);
        x_pred(1) += x(3);
        return x_pred;
    }

    StateMatrix jacobian(const StateVector& x) const override {
        StateMatrix F = StateMatrix::Identity();
        F(0,2) = 1.0;
        F(1,3) = 1.0;
        return F;
    }
};