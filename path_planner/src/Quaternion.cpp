#include "Quaternion.h"
#include <cmath>
# define M_PI           3.14159265358979323846
/*
* The equations and some of the source code are taken from Wekipidea page 
* https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
*/
Quaternion::Quaternion()
{
    this->r = 0.0;
    this->i = MatrixXf(3, 1);
}
Quaternion::Quaternion(MatrixXf euleur_angles) // euleur_angles = [roll , pitch, yaw]
{
    
    this->r = 0.0;
    this->i = MatrixXf(3, 1);
    /* Rotation matrix */
    double cr = cos(euleur_angles(0, 0) /2);
    double sr = sin(euleur_angles(0, 0) /2);
    double cp = cos(euleur_angles(1, 0) /2);
    double sp = sin(euleur_angles(1, 0) /2);
    double cy = cos(euleur_angles(2, 0) /2);
    double sy = sin(euleur_angles(2, 0) /2);
    /*Rotation matrix to Quaternion angles*/
    this->r = cr * cp * cy + sr * sp * sy;
    this->i(0,0) = sr * cp * cy - cr * sp * sy;
    this->i(1,0) = cr * sp * cy + sr * cp * sy;
    this->i(2,0) = cr * cp * sy - sr * sp * cy;
}

Quaternion::~Quaternion()
{}

double Quaternion::Magnitude()
{
    return  sqrt(pow(r,2)+ pow(i(0, 0), 2)+ pow(i(1, 0), 2)+ pow(i(2, 0), 2));
}

Quaternion Quaternion::Conjugate()
{
    Quaternion conj;
    conj.r = this->r;
    conj.i = this->i * -1;
    return conj;
}

Quaternion Quaternion::Inverse()
{
    Quaternion conj;
    conj = this->Conjugate();
    return (conj/pow(this->Magnitude(),2));
}

Quaternion Quaternion::operator*(Quaternion Q2)
{
    Quaternion Multiply;
    Vector3f v1 = i , v2 = Q2.i;

    MatrixXf cross = v1.cross(v2);
    Multiply.r = (r * Q2.r) + (Q2.i(0,0) * i(0,0)) + (Q2.i(1, 0) * i(1, 0)) + (Q2.i(2, 0) * i(2, 0));
    Multiply.i = r * Q2.i + Q2.r * i + cross;
	return Multiply;
}

Quaternion Quaternion::operator+(Quaternion Q2)
{
    Quaternion sum;
    sum.r = r + Q2.r;
    sum.i = i + Q2.i;
    return sum;
}

MatrixXf Quaternion::operator-(Quaternion Q2)
{
    MatrixXf diff(3,1);
    Vector3f v1 = i, v2 = Q2.i;

    MatrixXf cross = v1.cross(v2);
    diff = -r * Q2.i + Q2.r * i - cross;
	return diff;
}

Quaternion Quaternion::operator*(double d)
{
    Quaternion q;
    q.r = d * r;
    q.i = d * i;
    return q;
}
Quaternion Quaternion::operator/(double d)
{
    Quaternion q;
    q.r = r / d;
    q.i = i / d;
    return q;
}
MatrixXf Quaternion::Quaternion2euleur()
{
    MatrixXf euleur(3,1);
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (r * i(0,0) + i(1, 0) * i(2, 0));
    double cosr_cosp = 1 - 2 * (pow(i(0, 0),2) + pow(i(1,0), 2));
    euleur(0, 0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (r * i(1, 0) - i(0, 0) * i(2, 0)));
    double cosp = std::sqrt(1 - 2 * (r * i(1, 0) - i(0, 0) * i(2, 0)));
    euleur(1, 0) = (2 * std::atan2(sinp, cosp) - M_PI/2);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (r * i(2, 0) + i(0, 0) * i(1, 0));
    double cosy_cosp = 1 - 2 * (pow(i(1, 0), 2) + pow(i(2, 0), 2));
    euleur(2, 0) = std::atan2(siny_cosp, cosy_cosp);
    return euleur;
}
