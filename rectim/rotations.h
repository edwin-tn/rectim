#pragma once
/* class to set up rotations between oblique and vertical image*/

#include <math.h>
#include<corecrt_math_defines.h>
#include <opencv2/core/core.hpp>


class rotations
{
public:
	rotations();
	~rotations();
	rotations(const double& omegaDeg, const double& phiDeg, const double& kappaDeg);

	/* get a 3x3 rotation matrix with rotation sequence: Omega(primary), Phi(secondary), Kappa(tertiary) */
	cv::Matx33d rotationOmegaPhiKappa_object2camera() { return rotationOmegaPhiKappa_object2camera(omega, phi, kappa); };

	cv::Vec3d toOPK_object2camera(const cv::Matx33d& opk);	//return Omega-Phi_Kappa; OPK in deg

	double getOmega() { return this->omega; };	void setOmega(const double& omega) { this->omega = omega; };
	double getPhi() { return this->phi; };		void setPhi(const double& phi) { this->phi = phi; };
	double getKappa() { return this->kappa; };	void setKappa(const double& kappa) { this->kappa = kappa; };

	double getAzimuth() { return this->azimuth; };	void setAzimuth(const double& azimuth) { this->azimuth = azimuth; };
	double getTilt() { return this->tilt; };	void setTilt(const double& tilt) { this->tilt = tilt; };
	double getSwing() { return this->swing; };	void setSwing(const double& swing) { this->azimuth = swing; };

	void rotAts(double& degAzimuth, double& degTilt, double& degSwing);
	void rotAts();

private:
	/* ALL ROTATIONS ARE IN DEGREES*/
	// omega rotation around x-axis in degrees
	double omega;
	// phi rotation around y-axis in degrees
	double phi;
	// kappa rotation around z-axis in degrees
	double kappa;

	double azimuth, tilt, swing;	//in degrees

	// Converts degrees to radians
	double degreesToRadians(const double& angleDegrees) { return (angleDegrees * M_PI / 180.0); }
	// Converts radians to degrees
	double radiansToDegrees(const double& angleRadians) { return (angleRadians * 180.0 / M_PI); }


	/*rotations on each axis: right handed system, from object space coords system into camera system (or image) */
	cv::Matx33d rotX_o2i(const double& rad);	//rotation RHS around +x-axis object to image
	cv::Matx33d rotY_o2i(const double& rad);	//rotation RHS around +y-axis object to image
	cv::Matx33d rotZ_o2i(const double& rad);	//rotation RHS around +z-axis object to image

	/* get a 3x3 rotation matrix with rotation sequence: Omega(primary), Phi(secondary), Kappa(tertiary) */
	cv::Matx33d rotationOmegaPhiKappa_object2camera(double omegaDeg, double phiDeg, double kappaDeg);


};

rotations::rotations()
{
	omega = 0.0;	phi = 0.0;	kappa = 0.0;
	azimuth = 0.0;	tilt = 0.0; swing = 0.0;
}

rotations::~rotations()
{}

rotations::rotations(const double& omegaDeg, const double& phiDeg, const double& kappaDeg)
{
	this->omega = omegaDeg;		this->phi = phiDeg;		this->kappa = kappaDeg;
	azimuth = 0.0;	tilt = 0.0; swing = 0.0;
	//Matx33d r = rotationOmegaPhiKappa_object2camera();
	rotAts(omega, phi, kappa);
}


//rotation RHS omega around +x-axis object space system to camera system
//input is in radians, output: a 3x3 matrix in double
cv::Matx33d rotations::rotX_o2i(const double& rad)
{
	cv::Matx33d m;	m.zeros();
	double c = cos(rad), s = sin(rad);
	m(0, 0) = 1.0;
	m(1, 1) = c;	m(1, 2) = s;
	m(2, 1) = -s;	m(2, 2) = c;
	return m;
}

//rotation RHS phi around +y-axis object space system to camera system
//input is in radians, output: a 3x3 matrix in double
cv::Matx33d rotations::rotY_o2i(const double& rad)
{
	cv::Matx33d m;	m.zeros();
	double c = cos(rad), s = sin(rad);
	m(0, 0) = c;	m(0, 2) = -s;
	m(1, 1) = 1.0;
	m(2, 0) = s;	m(2, 2) = c;
	return m;
}

// rotation RHS kappa around + z - axis object space system to camera system
//input is in radians, output: a 3x3 matrix in double
cv::Matx33d rotations::rotZ_o2i(const double& rad)
{
	cv::Matx33d m;	m.zeros();
	double c = cos(rad), s = sin(rad);
	m(0, 0) = c;	m(0, 1) = s;
	m(1, 0) = -s;	m(1, 1) = c;
	m(2, 2) = 1.0;
	return m;
}

cv::Matx33d rotations::rotationOmegaPhiKappa_object2camera(double omegaDeg, double phiDeg, double kappaDeg)
{
	//convert to radians
	double o = degreesToRadians(omegaDeg);	//primary rotation
	double p = degreesToRadians(phiDeg);	//secondary rotation
	double k = degreesToRadians(kappaDeg);	//tertiary rotation

	return(rotZ_o2i(k) * rotY_o2i(p) * rotX_o2i(o));
}

//input: Matrix rotation
inline void rotations::rotAts(double& degOmega, double& degPhi, double& degKappa)
{
	Matx33d r = rotationOmegaPhiKappa_object2camera(degOmega, degPhi, degKappa);

	double a = 0, t = 0, s = 0;
	t = std::acos(r(2, 2));				this->tilt = radiansToDegrees(t);
	s = atan2(-r(0,2), -r(1, 2));		this->swing = radiansToDegrees(s);
	a = atan2(-r(2, 0), -r(2, 1));		this->azimuth = radiansToDegrees(a);
}

inline void rotations::rotAts()
{
	rotAts(omega, phi, kappa);
}

//cv::Matx33d rotations::rotationOmegaPhiKappa_object2camera()
//{
//	return rotationOmegaPhiKappa_object2camera(omega, phi, kappa);
//}

cv::Vec3d rotations::toOPK_object2camera(const cv::Matx33d& opk)
{
	cv::Vec3d v; v.zeros();
	double phi = asin(opk(2, 0));					//phi in radians
	double omega = atan2(-opk(2, 1), opk(2, 2));	//omega in radians
	double kappa = atan2(-opk(1, 0), opk(0, 0));	//kappa in radians

	//construct OPK
	v(0) = radiansToDegrees(omega);	//primary rotation
	v(1) = radiansToDegrees(phi);	//secondary rotation
	v(2) = radiansToDegrees(kappa);	//tertiary rotation

	return v;
}





