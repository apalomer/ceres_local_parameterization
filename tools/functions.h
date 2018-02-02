#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Eigen/Dense>

double fRand(double fMin, double fMax);

template<typename T>
Eigen::Matrix<T,3,3> mrot(T roll, T pitch, T yaw)
{
    // Create unit vectors
    Eigen::Matrix<T,3,1> unit_z = Eigen::Matrix<T,3,1>::UnitZ();
    Eigen::Matrix<T,3,1> unit_y = Eigen::Matrix<T,3,1>::UnitY();
    Eigen::Matrix<T,3,1> unit_x = Eigen::Matrix<T,3,1>::UnitX();

    // Create rotations 0
    Eigen::Matrix<T,3,3> rot_yaw = Eigen::AngleAxis<T>(yaw,unit_z).toRotationMatrix();
    Eigen::Matrix<T,3,3> rot_pitch = Eigen::AngleAxis<T>(pitch,unit_y).toRotationMatrix();
    Eigen::Matrix<T,3,3> rot_roll = Eigen::AngleAxis<T>(roll,unit_x).toRotationMatrix();

    // Exit
    return rot_yaw*rot_pitch*rot_roll;
}

template<typename T>
void getrpy(Eigen::Matrix<T,3,3> rotation_matrix, T& roll, T& pitch, T& yaw)
{
    Eigen::Matrix<T,3,1> ea = rotation_matrix.eulerAngles(2,1,0);
    yaw = ea(0);
    pitch = ea(1);
    roll = ea(2);
}

template<typename T>
T todeg(T ang)
{
    return ang*T(180)/T(M_PI);
}
template<typename T>
T torad(T ang)
{
    return ang*T(M_PI)/T(180);
}

#endif // FUNCTIONS_H
