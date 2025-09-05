#include <MeasurementModelBase.hpp>

class XYMeasurementModel : public MeasurementModelBase<4,2>
{
public:
    using Base = MeasurementModelBase<4,2>;
    using typename Base::MeasurementVector;
    using typename Base::MeasurementMatrix;

    XYMeasurementModel();

    MeasurementVector h(const StateVector& x) const override;

    CrossCovMatrix jacobian(const StateVector&) const override;

    MeasurementMatrix MeasNoise(const double dtime) const override;
};
