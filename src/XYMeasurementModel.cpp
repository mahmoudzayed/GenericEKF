#include <MeasurementModelBase.h>

class XYMeasurementModel : public MeasurementModelBase<4,2> 
{
public:
    MeasurementVector h(const StateVector& x) const override
    {
        MeasurementVector z;
        z << x(0), x(1);
        return z;
    }

    MeasurementMatrix jacobian(const StateVector&) const override
    {
        MeasurementMatrix H = MeasurementMatrix::Zero();
        H(0,0) = 1.0;
        H(1,1) = 1.0;
        return H;
    }
};