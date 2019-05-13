#ifndef __POSE_EKF_H
#define __POSE_EKF_H
#include <iostream>
#include <Eigen/Dense>

// state for kalman filter
// 0-3 quaternion
// 4-6 Px Py Pz
// 7-9 Vx Vy Vz
// 10-12 bwx bwy bwz
// 13-15 bax bay baz 
// inertial frame: ENU

class Pose_ekf
{
public:
	Pose_ekf();
	~Pose_ekf();	
	void predict(Eigen::Vector3d gyro, Eigen::Vector3d acc, double t);
	void correct(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d mag, double t);
	void process(Eigen::Vector3d gyro, Eigen::Vector3d acc, Eigen::VectorXd& xdot, Eigen::MatrixXd& F, Eigen::MatrixXd& G);
	Eigen::MatrixXd computeF(Eigen::Vector3d gyro, Eigen::Vector3d acc);
	
	Eigen::VectorXd measurement(Eigen::VectorXd x, Eigen::Vector3d mag);
	Eigen::MatrixXd computeH(Eigen::Vector3d mag);

	void measurement_fix(Eigen::Vector2d& position, Eigen::MatrixXd &H);
	void measurement_fix_velocity(Eigen::Vector3d& velocity, Eigen::MatrixXd& H);
	void measurement_sonar_height(Eigen::VectorXd& sonar_height, Eigen::MatrixXd& H);
	void measurement_magnetic_field(Eigen::Vector3d& magnetic_field, Eigen::MatrixXd& H);
	void measurement_gravity(Eigen::Vector3d& acc, Eigen::MatrixXd& H);

	void correct(Eigen::VectorXd z, Eigen::VectorXd zhat, Eigen::MatrixXd H, Eigen::MatrixXd R);
	void correct_fix(Eigen::Vector3d position, double t);
	void correct_fix_velocity(Eigen::Vector3d velocity, double t);
	void correct_sonar_height(double sonar_height, double t);//todo, without considering the roll and pitch
	void correct_magnetic_field(Eigen::Vector3d mag, double t);
	void correct_gravity(Eigen::Vector3d acc, double t);
	// void measurement_altimeter(double& altimeter_height, Eigen::MatrixXd H);
	void getState(Eigen::Quaterniond& q, Eigen::Vector3d& position, Eigen::Vector3d& velocity, Eigen::Vector3d & bw, Eigen::Vector3d&  ba);
	double get_time() { return current_t;}
private:
	Eigen::VectorXd x;//state 
	Eigen::MatrixXd P;//covariance


	const Eigen::Vector3d GRAVITY = Eigen::Vector3d(0, 0, 9.8);
	//covariance parameter
	const double fix_cov = 2.0;
	const double sonar_height_cov = 0.2;
	const double fix_velocity_cov = 2.0;
	
	const double gyro_cov = 0.01;
	const double acc_cov = 0.1;

	const double gravity_cov = 5.0;
	const double mag_cov = 5.0;

	const int n_state = 16;
	Eigen::MatrixXd Q;//imu observation noise
	const Eigen::MatrixXd R_fix = Eigen::Matrix2d::Identity()*fix_cov;
	const Eigen::MatrixXd R_fix_velocity = Eigen::Matrix3d::Identity()*fix_velocity_cov;
	const Eigen::MatrixXd R_sonar_height = Eigen::MatrixXd::Identity(1, 1)*sonar_height_cov;
	const Eigen::MatrixXd R_magnetic = Eigen::Matrix3d::Identity()*mag_cov;
	const Eigen::MatrixXd R_gravity = Eigen::Matrix3d::Identity()*gravity_cov;

	Eigen::Vector3d acc;
	Eigen::Vector3d gyro;

	Eigen::Vector3d referenceMagneticField_;
	double current_t;
	bool initialized;

	bool fix_initialized;
	bool imu_initialized;
	bool altimeter_initialized;
	bool sonar_initialized;
	bool magnetic_initialized;
	
};

#endif 