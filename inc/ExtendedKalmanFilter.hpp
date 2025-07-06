#pragma once
#include <KalmanFilterBase.hpp>
#include <MotionModelBase.hpp>
#include <MeasurementModelBase.hpp>
#include <functional>

template<int StateDim, int MeasurementDim>
class ExtendedKalmanFilter : public KalmanFilterBase<StateDim, MeasurementDim>
{
public:
    using Base = KalmanFilterBase<StateDim, MeasurementDim>;
    using typename Base::StateVector;
    using typename Base::StateMatrix;
    using typename Base::MeasurementVector;
    using typename Base::MeasurementMatrix;
    using typename Base::CrossCovMatrix;

    ExtendedKalmanFilter(std::shared_ptr<MotionModelBase<StateDim>> motionModel,
                         std::shared_ptr<MeasurementModelBase<StateDim, MeasurementDim>> measurementModel)
        : motionModel_(motionModel), measurementModel_(measurementModel);

    void initialize(const StateVector& x0, const StateMatrix& P0,
                    const StateMatrix& Q, const MeasurementMatrix& R);
    void predict();

    void update(const MeasurementVector& z);
    StateVector state();

private:
    StateVector x_;
    StateMatrix P_, Q_;
    MeasurementMatrix R_;
    std::shared_ptr<MotionModelBase<StateDim>> motionModel_;
    std::shared_ptr<MeasurementModelBase<StateDim, MeasurementDim>> measurementModel_;
};