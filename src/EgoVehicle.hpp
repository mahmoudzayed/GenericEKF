#include <Eigen/Dense>

class EgoVehicle
{
public:
    EgoVehicle();

    void setPosition(const Eigen::Vector3f& position);
    void setVelocity(const Eigen::Vector3f& velocity);
    void setEulerAngles(const Eigen::Vector3f& euler_angles);

    Eigen::Matrix4f calculateAffineMatrix(const Eigen::Vector3f& gyro, const float dt)
    {
        // --- 1. Integrate angular velocity into Euler angles ---
        euler_angles_ += gyro * dt;

        // --- 2. Compute rotation matrix from Euler angles ---
        Eigen::Matrix3f rotation = calculeRotationMatrix(euler_angles_);

        // --- 3. Construct affine transformation matrix ---
        Eigen::Matrix4f affine_matrix = Eigen::Matrix4f::Identity();
        affine_matrix.block<3, 3>(0, 0) = rotation;

        return affine_matrix;
    }

    void updatePosition (const Eigen::Vector3f& acceleration, const Eigen::Vector3f& gyro, const float dt)
    {
        Eigen::Matrix4f affine = calculateAffineMatrix(gyro, dt);
        affine *= input_to_ego;

        Eigen::Matrix3f R = affine.block<3,3>(0,0);

        // trasformed Acceleration
        Eigen::Vector3f new_acceleration = R * acceleration - gravity_;

        //update velocity
        velocity_ += new_acceleration * dt;

        //update position
        position_ += velocity_ * dt + 0.5F * new_acceleration * dt * dt;

    }


    Eigen::Matrix4f getFinalPose() const
    {
        Eigen::Matrix4f finalpose = Eigen::Matrix4f::Identity();
        finalpose.block<3,3>(0,0) = calculeRotationMatrix(euler_angles_);
        finalpose.block<3,1>(0,3) = position_;

        return finalpose;
    }

    Eigen::Vector3f getVelocity() const
    {
        return velocity_;
    }
private:
    Eigen::Matrix3f calculeRotationMatrix(const Eigen::Vector3f& angles) const
    {
        Eigen::Matrix3f rotation =   Eigen::AngleAxisf(angles.x(), Eigen::Vector3f::UnitX())
                                   * Eigen::AngleAxisf(angles.y(), Eigen::Vector3f::UnitY())
                                   * Eigen::AngleAxisf(angles.z(), Eigen::Vector3f::UnitZ());
        return rotation;
    }


private:
    const Eigen::Vector3f gravity_ = Eigen::Vector3f(9.81, 0, 0); // x-axis is looking down
    const Eigen::Matrix4f input_to_ego = (Eigen::Matrix4f() << 0, 0, -1, 0,
                                                               0, 1,  0,  0,
                                                               1, 0,  0,  0,
                                                               0, 0,  0,  1).finished();
    Eigen::Vector3f position_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f euler_angles_;
};
