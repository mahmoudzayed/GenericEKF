#include <ExtendedKalmanFilter.hpp>

ExtendedKalmanFilter(std::shared_ptr<MotionModelBase<StateDim>> motionModel,
                        std::shared_ptr<MeasurementModelBase<StateDim, MeasurementDim>> measurementModel)
    : motionModel_(motionModel), measurementModel_(measurementModel) {}

void ExtendedKalmanFilter::initialize(const StateVector& x0, const StateMatrix& P0,
                                      const StateMatrix& Q, const MeasurementMatrix& R) override 
{
    this->x_ = x0; 
    this->P_ = P0; 
    this->Q_ = Q; 
    this->R_ = R;
}

void ExtendedKalmanFilter::predict(const VectorX& u = VectorX::Zero()) override 
{
    MatrixX F = processJacobian_(this->x_);
    this->x_ = processFunc_(this->x_);  // u can be added here if needed
    this->P_ = F * this->P_ * F.transpose() + this->Q_;
}

void ExtendedKalmanFilter::update(const VectorZ& z) override 
{
    MatrixZX H = measurementJacobian_(this->x_);
    VectorZ y = z - measurementFunc_(this->x_);
    MatrixZ S = H * this->P_ * H.transpose() + this->R_;
    MatrixX K = this->P_ * H.transpose() * S.inverse();
    MatrixX Josephform = MatrixX::Identity() - K * H;

    // Update state and covariance using Joseph form
    this->x_ = this->x_ + K * y;
    this->P_ = Josephform * this->P_ Josephform.transpose() + K * this->R_ * K.transpose();
}

StateVector ExtendedKalmanFilter::state() const override 
{
    return x_;
}
