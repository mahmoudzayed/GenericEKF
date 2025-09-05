#include <XYMeasurementModel.hpp>

XYMeasurementModel::XYMeasurementModel() {
    // Default constructor implementation
}

XYMeasurementModel::MeasurementVector XYMeasurementModel::h(const StateVector& x) const
{
    MeasurementVector z;
    z << x(0), x(1);
    return z;
}

XYMeasurementModel::CrossCovMatrix XYMeasurementModel::jacobian(const StateVector&) const
{
    CrossCovMatrix H = CrossCovMatrix::Zero();
    H(0,0) = 1.0;
    H(1,1) = 1.0;
    return H;
}

XYMeasurementModel::MeasurementMatrix XYMeasurementModel::MeasNoise(const double dtime) const
{
    MeasurementMatrix R = MeasurementMatrix::Zero();
    R(0,0) = 0.01;
    R(1,1) = 0.01;
    return R;
}
