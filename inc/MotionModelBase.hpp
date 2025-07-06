#include <Eigen/Dense>

template<int StateDim>
class MotionModelBase {
public:
    using StateVector = Eigen::Matrix<double, StateDim, 1>;
    using StateMatrix = Eigen::Matrix<double, StateDim, StateDim>;

    virtual ~MotionModelBase() = default;
    virtual StateVector f(const StateVector& x) const = 0;
    virtual StateMatrix jacobian(const StateVector& x) const = 0;
};
