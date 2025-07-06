#include <ExtendedKalmanFilter.hpp>
#include <iostream>

int main()
{
    auto motion = std::make_shared<CVModel2D>();
    auto measurement = std::make_shared<XYMeasurementModel>();
    ExtendedKalmanFilter<4,2> ekf(motion, measurement);

    Eigen::Vector4d x0; x0 << 0,0,1,1;
    ekf.initialize(x0, Eigen::Matrix4d::Identity(), 0.01*Eigen::Matrix4d::Identity(), 0.1*Eigen::Matrix2d::Identity());

    for (int t = 0; t < 10; ++t)
    {
        ekf.predict();
        Eigen::Vector2d z; z << t+0.5, t+0.4; // Fake noisy observation
        ekf.update(z);
        std::cout << "State at step " << t << ": " << ekf.state().transpose() << std::endl;
    }
}