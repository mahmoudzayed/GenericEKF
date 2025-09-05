#include <ExtendedKalmanFilter.hpp>
#include <CVModel2D.hpp>
#include <MotionModelHandler.hpp>
#include <XYMeasurementModel.hpp>
#include <iostream>

void initializeMotionModels()
{
    MotionModelHandler::registerModel(MotionModelType::CVV, std::make_shared<CVModel2D>());
}

int main()
{
    const unsigned int stateDim = 4;
    const unsigned int measDim = 2;
    initializeMotionModels();

    ExtendedKalmanFilter<stateDim, measDim> ExtendedKalmanFilter(std::move(std::make_unique<CVModel2D>()), std::move(std::make_unique<XYMeasurementModel>()));

    Eigen::Vector4d x0 = {0, 0, 1, 1};
    ExtendedKalmanFilter.initialize(x0, Eigen::Matrix4d::Identity());

    for (int t = 0; t < 10; ++t)
    {
        Eigen::Vector4d x0 = {0, 0, 0, 0};
        ExtendedKalmanFilter.predict(x0, 0.05);
        Eigen::Vector2d z; z << t+0.5, t+0.4; // Fake noisy observation
        ExtendedKalmanFilter.update(z);
        std::cout << "State at step " << t << ": " << ExtendedKalmanFilter.state().transpose() << std::endl;
    }
}
