#include "IMUErrorModel.h"
template <class Scalar>
IMUErrorModel<Scalar>::IMUErrorModel(const Eigen::Matrix<Scalar, 6, 1> b_ag): b_a(b_ag.template head<3>()),
    b_g(b_ag.template tail<3>()),
    S_a(Eigen::Matrix<Scalar, 3, 3>::Zero()),S_g(Eigen::Matrix<Scalar, 3, 3>::Zero()), T_s(Eigen::Matrix<Scalar, 3, 3>::Zero()),
    invT_a(Eigen::Matrix<Scalar, 3, 3>::Identity()),invT_g(Eigen::Matrix<Scalar, 3, 3>::Identity())
{
}
//template <class Scalar>
template <class Scalar>
void IMUErrorModel<Scalar>::estimate(const Eigen::Matrix<Scalar, 3, 1> a_m, const Eigen::Matrix<Scalar, 3, 1> w_m)
{
    a_est = invT_a*(a_m- b_a);
    w_est = invT_g*(w_m- b_g - T_s* a_est);
}
template <class Scalar>
void IMUErrorModel<Scalar>::predict(const Eigen::Matrix<Scalar, 3, 1> a_s, const Eigen::Matrix<Scalar, 3, 1> w_s)
{
    a_obs = S_a*a_s +a_s+ b_a;
    w_obs = S_g*w_s + w_s + T_s*a_s + b_g;
}
