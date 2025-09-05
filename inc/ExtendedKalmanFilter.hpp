#pragma once

#include <KalmanFilterBase.hpp>
#include <MeasurementModelBase.hpp>
#include <MotionModelBase.hpp>
#include <functional>
#include <memory>
#include <stdexcept>
#include <cassert>

template<int StateDim, int MeasurementDim>
class ExtendedKalmanFilter : public KalmanFilterBase<StateDim, MeasurementDim>
{
public:
    using Base = KalmanFilterBase<StateDim, MeasurementDim>;
    using typename Base::StateVector;
    using typename Base::StateMatrix;
    using typename Base::MeasurementVector;
    using typename Base::MeasurementMatrix;

    ExtendedKalmanFilter(
        std::unique_ptr<MotionModelBase<StateDim, MeasurementDim>> motionModel,
        std::unique_ptr<MeasurementModelBase<StateDim, MeasurementDim>> measurementModel)
        : motionModel_(std::move(motionModel)),
          measurementModel_(std::move(measurementModel))
    {
    }

    ExtendedKalmanFilter() = delete;
    ExtendedKalmanFilter(ExtendedKalmanFilter&&) = default;
    ExtendedKalmanFilter(const ExtendedKalmanFilter&) = delete;
    ExtendedKalmanFilter& operator=(ExtendedKalmanFilter&&) = default;
    ExtendedKalmanFilter& operator=(const ExtendedKalmanFilter&) = delete;

    ~ExtendedKalmanFilter() override = default;


    void initialize(const StateVector& x0, const StateMatrix& P0) override
    {
        x_ = x0;
        P_ = P0;
    }

    void predict(const StateVector& u, const unsigned int dtime) override
    {
        assert(motionModel_ && "Motion model must be set before prediction");

        StateMatrix F = motionModel_->jacobian(x_, u, dtime);
        x_ = motionModel_->f(x_, u, dtime);
        P_ = F * P_ * F.transpose() + motionModel_->ProcessNoise();
    }

    void update(const MeasurementVector& z) override
    {
        assert(measurementModel_ && "Measurement model must be set before update");
        double dtime = 0.05;
        auto measnoise = measurementModel_->MeasNoise(dtime);

        Eigen::Matrix<double, MeasurementDim, StateDim> H = measurementModel_->jacobian(this->x_);
        MeasurementVector y = z - measurementModel_->h(this->x_);
        MeasurementMatrix S = H * this->P_ * H.transpose() + measnoise;
        Eigen::Matrix<double, StateDim, MeasurementDim> K = this->P_ * H.transpose() * S.inverse();
        StateMatrix Josephform = StateMatrix::Identity() - K * H;

        // Corrected line:
        this->P_ = Josephform * this->P_ * Josephform.transpose() + K * measnoise * K.transpose();
        this->x_ = this->x_ + K * y;
    }

    const StateVector& state() const override
    {
        return x_;
    }

    void setMotionModel(std::unique_ptr<MotionModelBase<StateDim, MeasurementDim>> motionModel)
    {
        if (!motionModel) throw std::invalid_argument("motionModel cannot be null");
        motionModel_ = std::move(motionModel);
    }

    void setMeasurementModel(std::unique_ptr<MeasurementModelBase<StateDim, MeasurementDim>> measurementModel)
    {
        if (!measurementModel) throw std::invalid_argument("measurementModel cannot be null");
        measurementModel_ = std::move(measurementModel);
    }

private:
    StateVector x_;
    StateMatrix P_;
    std::unique_ptr<MotionModelBase<StateDim, MeasurementDim>> motionModel_;
    std::unique_ptr<MeasurementModelBase<StateDim, MeasurementDim>> measurementModel_;

};
