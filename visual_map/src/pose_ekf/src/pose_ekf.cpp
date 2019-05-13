#include "pose_ekf.h"
#include <iostream>
#include <Eigen/Dense>
#include "conversion.h"

//quaternion: body fram to navigation frame
//Rnb
// state for kalman filter
// 0-3 quaternion
// 4-6 Px Py Pz
// 7-9 Vx Vy Vz
// 10-12 bwx bwy bwz
// 13-15 bax bay baz 
// inertial frame: ENU

Eigen::Matrix3d skew_symmetric(Eigen::Vector3d v)
{
	Eigen::Matrix3d m;
	m << 0, -v(2), v(1),
		 v(2), 0,  -v(0),
		 -v(1), v(0), 0;
	return m;
}

//diff_(p*q) /diff_q
Eigen::Matrix4d diff_pq_q(Eigen::Quaterniond p)
{
	double p0 = p.w();
	Eigen::Vector3d pv = p.vec();

	Eigen::Matrix4d D;
	D(0, 0) = p0;
	D.block<1, 3>(0, 1) = -pv.transpose();
	D.block<3, 1>(1, 0) = pv;
	D.block<3, 3>(1, 1) = Eigen::Matrix3d::Identity()*p0 + skew_symmetric(pv);
	return D;
}


//diff_(p*q)/ diff_p
Eigen::Matrix4d diff_pq_p(Eigen::Quaterniond q)
{
	double q0 = q.w();
	Eigen::Vector3d qv = q.vec();
	Eigen::Matrix4d D;
	D(0, 0) = q0;
	D.block<1, 3>(0, 1) = -qv.transpose();
	D.block<3, 1>(1, 0) = qv;
	D.block<3, 3>(1, 1) = Eigen::Matrix3d::Identity()*q0 - skew_symmetric(qv);
	return D;
}

//diff_(q*v*q_star)/ diff_q
Eigen::MatrixXd diff_qvqstar_q(Eigen::Quaterniond q, Eigen::Vector3d v)
{
	double q0 = q.w();
	Eigen::Vector3d qv = q.vec();
	Eigen::MatrixXd D(3, 4);
	D.col(0) = 2*(q0*v + skew_symmetric(qv)*v);
	D.block<3, 3>(0, 1) = 2*(-v*qv.transpose() + v.dot(qv)*Eigen::Matrix3d::Identity() - q0*skew_symmetric(v));
	return D; 
}

//diff_(qstar*v*q)/ diff_q
Eigen::MatrixXd diff_qstarvq_q(Eigen::Quaterniond q, Eigen::Vector3d v)
{
	double q0 = q.w();
	Eigen::Vector3d qv = q.vec();
	Eigen::MatrixXd D(3, 4);
	D.col(0) = 2*(q0*v - skew_symmetric(qv)*v);
	D.block<3, 3>(0, 1) = 2*(-v*qv.transpose() + v.dot(qv)*Eigen::Matrix3d::Identity() + q0*skew_symmetric(v));
	return D; 
}
//diff_(q*v*q_star)/ diff_v
Eigen::Matrix3d diff_qvqstar_v(Eigen::Quaterniond q)
{
	double q0 = q.w();
	Eigen::Vector3d qv = q.vec();
	Eigen::Matrix3d D;
	D = (q0*q0 - qv.dot(qv))*Eigen::Matrix3d::Identity() + 2*qv*qv.transpose() + 2*q0*skew_symmetric(qv);
	return D; 
}

Pose_ekf::Pose_ekf()
{	
	initialized = false;
	x = Eigen::VectorXd::Zero(n_state);
	x.head(4) << 1, 0, 0, 0;
	P = Eigen::MatrixXd::Identity(n_state, n_state);

	Q = Eigen::MatrixXd::Zero(6, 6);
	Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity()*gyro_cov;
	Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity()*acc_cov;


	initialized = false;
	fix_initialized = false;
	imu_initialized = false;
	altimeter_initialized = false;
	sonar_initialized = false;
	magnetic_initialized = false;
}

Pose_ekf::~Pose_ekf()
{
	
}


void Pose_ekf::predict(Eigen::Vector3d gyro, Eigen::Vector3d acc, double t)
{
	if(!imu_initialized)
	{
		imu_initialized = true; initialized = true;
		this->current_t = t;
		double phy = atan2(acc(1), acc(2));
		double theta = atan2(-acc(0), acc(2));
		Eigen::Vector3d rpy(phy, theta, 0);
		Eigen::Quaterniond q = euler2quaternion(rpy);
		x(0) = q.w(); x.segment<3>(1) = q.vec();
		return;
	}
	if(t <= current_t) return;

	double dt = t - current_t;
	Eigen::VectorXd xdot(n_state);
	Eigen::MatrixXd F(n_state, n_state);
	Eigen::MatrixXd G(n_state, 6);//G = dx/du

	process(gyro, acc, xdot, F, G);
	
	x += xdot*dt;
	F = Eigen::MatrixXd::Identity(n_state, n_state) + F*dt;//continous F and discrete F
	G = G*dt;
	// cout << "G: " << G << endl;
	// cout << "GQG: " << G*Q*G << endl;

	P = F*P*F.transpose() + G*Q*G.transpose();
	x.head(4).normalize();
	
	this->current_t = t;
	this->acc = acc;
	this->gyro = gyro;
}

//xdot = f(x, u);
void Pose_ekf::process(Eigen::Vector3d gyro, Eigen::Vector3d acc, Eigen::VectorXd& xdot, Eigen::MatrixXd& F, Eigen::MatrixXd& G)
{
	
	Eigen::Quaterniond q;
	Eigen::Vector3d p, v, bw, ba;
	getState(q, p, v, bw, ba);
	
	xdot.setZero();
	F.setZero();
	G.setZero();
	
	Eigen::Quaterniond gyro_q(0, 0, 0, 0);
	gyro_q.vec() = gyro - bw;
	Eigen::Quaterniond q_dot = q*gyro_q;
	q_dot.w() /= 2; q_dot.vec() /= 2;//* and / to scalar is not support for Quaternion
	xdot(0) = q_dot.w(); xdot.segment<3>(1) = q_dot.vec();
	xdot.segment<3>(4) = v;

	Eigen::Quaterniond acc_b_q(0, 0, 0, 0);
	acc_b_q.vec() = acc - ba;
	Eigen::Quaterniond acc_n_q =  q*acc_b_q*q.inverse();
	xdot.segment<3>(7) = acc_n_q.vec() - GRAVITY;//body frame to n frame 
	
	F.block<4, 4>(0, 0) = 0.5*diff_pq_p(gyro_q);
	F.block<4, 3>(0, 10) = -0.5*(diff_pq_q(q).block<4, 3>(0, 1));
	F.block<3, 3>(4, 7) = Eigen::Matrix3d::Identity();
	F.block<3, 4>(7, 0) = diff_qvqstar_q(q, acc_b_q.vec());
	F.block<3, 3>(7, 13) = -diff_qvqstar_v(q);

	//G = d_xdot/du
	G.block<4, 3>(0, 0) = 0.5*diff_pq_q(q).block<4, 3>(0, 1);//diff(0.5*q*gyro_q)/diff(gyro_q)
	G.block<3, 3>(7, 3) = diff_qvqstar_v(q);//diff(q*a*qstar)/diff(a)
}

void Pose_ekf::getState(Eigen::Quaterniond& q, Eigen::Vector3d& p, Eigen::Vector3d& v, Eigen::Vector3d & bw, Eigen::Vector3d& ba)
{
	q.w() = x(0);
	q.vec() = x.segment<3>(1);
	p = x.segment<3>(4);
	v = x.segment<3>(7);
	bw = x.segment<3>(10);
	ba = x.segment<3>(13);
}


void Pose_ekf::measurement_fix(Eigen::Vector2d& position, Eigen::MatrixXd &H)
{
	position = x.segment<2>(4);
	H = Eigen::MatrixXd::Zero(2, n_state);
	H.block<2, 2>(0, 4) = Eigen::Matrix2d::Identity();
}
void Pose_ekf::measurement_fix_velocity(Eigen::Vector3d& velocity, Eigen::MatrixXd& H)
{
	velocity = x.segment<3>(7);
	H = Eigen::MatrixXd::Zero(3, n_state);
	H.block<3, 3>(0, 7) = Eigen::Matrix3d::Identity();
}

void Pose_ekf::measurement_sonar_height(Eigen::VectorXd& sonar_height, Eigen::MatrixXd& H)
{
	sonar_height = Eigen::VectorXd(1);
	sonar_height(0) = x(6);
	H = Eigen::MatrixXd::Zero(1, n_state);
	H(0, 6) = 1;
}

void Pose_ekf::measurement_magnetic_field(Eigen::Vector3d& magnetic_field, Eigen::MatrixXd& H)
{
	Eigen::Quaterniond q;
	q.w() = x(0); q.vec() = x.segment<3>(1);
	Eigen::Quaterniond ref_mag_q;
	ref_mag_q.w() = 0; ref_mag_q.vec() = referenceMagneticField_;
	Eigen::Quaterniond magnetic_field_q =  q.inverse()*ref_mag_q*q; //r_n to r_b
	magnetic_field = magnetic_field_q.vec();

	H = Eigen::MatrixXd::Zero(3, n_state);
	H.block<3, 4>(0, 0) = diff_qstarvq_q(q, referenceMagneticField_);
}

void Pose_ekf::measurement_gravity(Eigen::Vector3d& acc, Eigen::MatrixXd& H)
{
	Eigen::Quaterniond q;
	q.w() = x(0); q.vec() = x.segment<3>(1);
	Eigen::Vector3d ba = x.segment<3>(13);
	Eigen::Quaterniond g_n_q;
	g_n_q.w() = 0; g_n_q.vec() = Eigen::Vector3d(0, 0, 1);//only direction is used
	Eigen::Quaterniond acc_q =  q.inverse()*g_n_q*q; //r_n to r_b
	acc = acc_q.vec();

	H = Eigen::MatrixXd::Zero(3, n_state);
	H.block<3, 4>(0, 0) = diff_qstarvq_q(q, GRAVITY);
}


void Pose_ekf::correct(Eigen::VectorXd z, Eigen::VectorXd zhat, Eigen::MatrixXd H, Eigen::MatrixXd R)
{
   	Eigen::MatrixXd K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
    x += K*(z - zhat);
    
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_state, n_state);
    P = (I - K*H)*P;
    x.head(4).normalize();	
}

void Pose_ekf::correct_fix(Eigen::Vector3d position, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	if(t < current_t) return;
	
	predict(this->gyro, this->acc, t);
	double dt = t - current_t;
	Eigen::Vector2d z = position.head(2);
	Eigen::Vector2d zhat;
	Eigen::MatrixXd H;
	measurement_fix(zhat, H);
	correct(z, zhat, H, R_fix);
}
void Pose_ekf::correct_fix_velocity(Eigen::Vector3d velocity, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	if(t < current_t) return;
	
	predict(this->gyro, this->acc, t);
	
	Eigen::Vector3d z = velocity;
	Eigen::Vector3d zhat;
	Eigen::MatrixXd H;
	measurement_fix_velocity(zhat, H);
	correct(z, zhat, H, R_fix_velocity);
}
void Pose_ekf::correct_sonar_height(double sonar_height, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	if(t < current_t) return;	
	predict(this->gyro, this->acc, t);

	Eigen::VectorXd z(1);
	z(0) = sonar_height;
	Eigen::VectorXd zhat(1);
	Eigen::MatrixXd H;

	measurement_sonar_height(zhat, H);
	correct(z, zhat, H, R_sonar_height);
}

void Pose_ekf::correct_magnetic_field(Eigen::Vector3d mag, double t)
{
	if(!magnetic_initialized)
	{
		//note, mag in ENU should be [0 1 x], but for the simulated data it is [1 0 x], maybe a bug
		referenceMagneticField_(0) = mag.head(2).norm();//todo
		referenceMagneticField_(1) = 0;
		referenceMagneticField_(2) = mag(2);
		magnetic_initialized = true;
		current_t = t;
		return;
	}
	if(t < current_t) return;
	predict(this->gyro, this->acc, t);
	
	Eigen::Vector3d z = mag;
	Eigen::Vector3d zhat;
	Eigen::MatrixXd H;
	measurement_magnetic_field(zhat, H);
	correct(z, zhat, H, R_magnetic);
}

void Pose_ekf::correct_gravity(Eigen::Vector3d acc, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}
	if(t < current_t) return;
	predict(this->gyro, this->acc, t);
	
	Eigen::Vector3d z = acc/acc.norm();
	Eigen::Vector3d zhat;
	Eigen::MatrixXd H;
	measurement_gravity(zhat, H);
	correct(z, zhat, H, R_gravity);
}