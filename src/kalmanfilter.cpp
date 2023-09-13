#include "ekf/kalmanfilter.h"

void KalmanFilter::start(
    const int nin, const VectorXd& xin, const MatrixXd& Pin, const MatrixXd& Fin, const MatrixXd& Qin) {
  this->n = nin;    // * 数据维度
  this->I = MatrixXd::Identity(this->n, this->n);   // * 单位矩阵
  this->x = xin;  // * 变量值
  this->P = Pin;  // * 协方差矩阵
  this->F = Fin;  // * 状态转移矩阵
  this->Q = Qin;  // * 协方差噪声
}

void KalmanFilter::setQ(const MatrixXd& Qin) { this->Q = Qin; }

void KalmanFilter::updateF(const double dt) {
  this->F(0, 2) = dt;
  this->F(1, 3) = dt;
}

VectorXd KalmanFilter::get() const { return this->x; }

void KalmanFilter::predict() {
  this->x = this->F * this->x;
  this->P = this->F * this->P * this->F.transpose() + this->Q;
}

void KalmanFilter::update(const VectorXd& z, const MatrixXd& H, const VectorXd& Hx, const MatrixXd& R) {
  const MatrixXd PHt = this->P * H.transpose(); 
  const MatrixXd S = H * PHt + R;     // * 测量协方差矩阵更新 S = H * P' * (H.t) + R
  const MatrixXd K = PHt * S.inverse(); // * 卡尔曼增益 K = P' * (H.t) * (S.i)
  VectorXd y = z - Hx;  // * 误差值， 通过矩阵 H 将状态变量转换成与传感器同样的单位

  // Assume this is radar measurement
  // y(1) is an angle (phi), it shoulde be normalized
  // refer to the comment at the bottom of this file
  if (y.size() == 3) y(1) = atan2(sin(y(1)), cos(y(1)));

  this->x = this->x + K * y;    // * 测量更新
  this->P = (this->I - K * H) * this->P;  // * 协方差矩阵更新
}

/*
Normalizing Angles
In C++, atan2() returns values between -pi and pi.
When calculating phi in y = z - h(x) for radar measurements,
the resulting angle phi in the y vector should be adjusted
so that it is between -pi and pi. The Kalman filter is expecting
small angle values between the range -pi and pi.
*/
