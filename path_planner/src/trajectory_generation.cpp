#include "trajectory_generation.h"
// AX = B
// A = inv(X)*B
//                  T
// A = [a0 a1 a2 a3]

// (row, column)
/*
    MatrixXf A(3, 3);
    A(0, 0) = 1;
    A(1, 0) = 4;
    A(2, 0) = 7;
    A(0, 1) = 2;
    A(1, 1) = 5;
    A(2, 1) = 8;
    A(0, 2) = 3;
    A(1, 2) = 6;
    A(2, 2) = 9;
    1 2 3
    // 4 5 6
    7 8 9
*/

//  M-file to compute a quintic polynomial reference trajectory
//  q0 = initial position
//  v0 = initial velocity
//  ac0 = initial acceleration
//  q1 = final position
//  v1 = final velocity
//  ac1 = final acceleration
//  t0 = initial time
//  tf = final time




MatrixXf computeCubicCoeff(float t0, float tf, std::vector<float> vec_q0, std::vector<float> vec_qf)
{

    MatrixXf X(4, 4);
    MatrixXf B(4, 1);

    X(0, 0) = 1;
    X(0, 1) = t0;
    X(0, 2) = std::pow(t0, 2);
    X(0, 3) = std::pow(t0, 3);

    X(1, 0) = 0;
    X(1, 1) = 1;
    X(1, 2) = 2 * t0;
    X(1, 3) = 3 * std::pow(t0, 2);

    X(2, 0) = 1;
    X(2, 1) = tf;
    X(2, 2) = std::pow(tf, 2);
    X(2, 3) = std::pow(tf, 3);

    X(3, 0) = 0;
    X(3, 1) = 1;
    X(3, 2) = 2 * tf;
    X(3, 3) = 3 * std::pow(tf, 2);

    B(0, 0) = vec_q0[0];
    B(1, 0) = vec_q0[1];
    B(2, 0) = vec_qf[0];
    B(3, 0) = vec_qf[1];

    return (X.inverse() * B);
}
//----------------------- Cubic order polynomial Trajectory -----------------------------------------------
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> computeCubicTraj(MatrixXf A, float t0, float tf, int n)
{

    std::vector<float> a = { A(0, 0), A(1, 0), A(2, 0), A(3, 0) };

    std::vector<double> qd; // position
    std::vector<double> d_qd; // velocity
    std::vector<double> dd_qd; // acceleration
    std::vector<double> time;

    float step = (tf - t0) / n;
    for (float t = t0; t < tf; t += step)
    {

        double qdi = a[0] + a[1] * t + a[2] * std::pow(t, 2) + a[3] * std::pow(t, 3);
        double d_qdi = a[1] + 2 * a[2] * t + 3 * a[3] * std::pow(t, 2);

        qd.push_back(qdi);
        d_qd.push_back(d_qdi);
        time.push_back(t);
    }

    return std::make_tuple(time, qd, d_qd);
}

MatrixXf computeQuinticCoeff(float t0, float tf, std::vector<float> vec_q0, std::vector<float> vec_qf)
{
    MatrixXf X(6, 6);
    // MatrixXf A(6, 1);
    MatrixXf B(6, 1);

    X(0, 0) = 1;
    X(0, 1) = t0;
    X(0, 2) = std::pow(t0, 2);
    X(0, 3) = std::pow(t0, 3);
    X(0, 4) = std::pow(t0, 4);
    X(0, 5) = std::pow(t0, 5);

    X(1, 0) = 0;
    X(1, 1) = 1;
    X(1, 2) = 2 * t0;
    X(1, 3) = 3 * std::pow(t0, 2);
    X(1, 4) = 4 * std::pow(t0, 3);
    X(1, 5) = 5 * std::pow(t0, 4);

    X(2, 0) = 0;
    X(2, 1) = 0;
    X(2, 2) = 2;
    X(2, 3) = 6 * t0;
    X(2, 4) = 12 * std::pow(t0, 2);
    X(2, 5) = 20 * std::pow(t0, 3);

    X(3, 0) = 1;
    X(3, 1) = tf;
    X(3, 2) = std::pow(tf, 2);
    X(3, 3) = std::pow(tf, 3);
    X(3, 4) = std::pow(tf, 4);
    X(3, 5) = std::pow(tf, 5);

    X(4, 0) = 0;
    X(4, 1) = 1;
    X(4, 2) = 2 * tf;
    X(4, 3) = 3 * std::pow(tf, 2);
    X(4, 4) = 4 * std::pow(tf, 3);
    X(4, 5) = 5 * std::pow(tf, 4);

    X(5, 0) = 0;
    X(5, 1) = 0;
    X(5, 2) = 2;
    X(5, 3) = 6 * tf;
    X(5, 4) = 12 * std::pow(tf, 2);
    X(5, 5) = 20 * std::pow(tf, 3);

    B(0, 0) = vec_q0[0];
    B(1, 0) = vec_q0[1];
    B(2, 0) = vec_q0[2];
    B(3, 0) = vec_qf[0];
    B(4, 0) = vec_qf[1];
    B(5, 0) = vec_qf[2];

    return (X.inverse() * B);
}

std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> computeQuinticTraj(MatrixXf A, float t0, float tf, int n)
{
    std::vector<float> a = { A(0, 0), A(1, 0), A(2, 0), A(3, 0), A(4, 0), A(5, 0) };

    std::vector<float> qd;      // position
    std::vector<float> d_qd;    // velocity = derivative(position)
    std::vector<float> dd_qd;   // acceleration derivative(derivative(position))
    std::vector<float> time;    

    float step = (tf - t0) / n;
    for (float t = t0; t < tf+step; t += step)
    {

        float qdi = a[0] + a[1] * t + a[2] * std::pow(t, 2) + a[3] * std::pow(t, 3) + a[4] * std::pow(t, 4) + a[5] * std::pow(t, 5);
        float d_qdi = a[1] + 2 * a[2] * t + 3 * a[3] * std::pow(t, 2) + 4 * a[4] * std::pow(t, 3) + 5 * a[5] * std::pow(t, 4);
        float dd_qdi = 2 * a[2] + 6 * a[3] * t + 12 * a[4] * std::pow(t, 2) + 20 * a[5] * std::pow(t, 3);

        qd.push_back(qdi);
        d_qd.push_back(d_qdi);
        dd_qd.push_back(dd_qdi);
        time.push_back(t);
    }

    return std::make_tuple(time, qd, d_qd, dd_qd);
}
//-------------------------- 7 order polynomial trajectory --------------------------------------------

MatrixXf compute7orderCoeff(float t0, float t1, float t2, float t3, std::vector<float> vec_q0x, std::vector<float> vec_qwx, std::vector<float> vec_q3x)
{

    MatrixXf X(8, 8);
    MatrixXf B(8, 1);

    X(0, 0) = 1;
    X(0, 1) = t0;
    X(0, 2) = std::pow(t0, 2);
    X(0, 3) = std::pow(t0, 3);
    X(0, 4) = std::pow(t0, 4);
    X(0, 5) = std::pow(t0, 5);
    X(0, 6) = std::pow(t0, 6);
    X(0, 7) = std::pow(t0, 7);

    X(1, 0) = 0;
    X(1, 1) = 1;
    X(1, 2) = 2 * t0;
    X(1, 3) = 3 * std::pow(t0, 2);
    X(1, 4) = 4 * std::pow(t0, 3);
    X(1, 5) = 5 * std::pow(t0, 4);
    X(1, 6) = 6 * std::pow(t0, 5);
    X(1, 7) = 7 * std::pow(t0, 6);

    X(2, 0) = 0;
    X(2, 1) = 0;
    X(2, 2) = 2;
    X(2, 3) = 6 * t0;
    X(2, 4) = 12 * std::pow(t0, 2);
    X(2, 5) = 20 * std::pow(t0, 3);
    X(2, 6) = 30 * std::pow(t0, 4);
    X(2, 7) = 42 * std::pow(t0, 5);

    X(3, 0) = 1;
    X(3, 1) = 1 * t1;
    X(3, 2) = 1 * std::pow(t1, 2);
    X(3, 3) = 1 * std::pow(t1, 3);
    X(3, 4) = 1 * std::pow(t1, 4);
    X(3, 5) = 1 * std::pow(t1, 5);
    X(3, 6) = 1 * std::pow(t1, 6);
    X(3, 7) = 1 * std::pow(t1, 7);

    X(4, 0) = 1;
    X(4, 1) = 1 * t1;
    X(4, 2) = 1 * std::pow(t2, 2);
    X(4, 3) = 1 * std::pow(t2, 3);
    X(4, 4) = 1 * std::pow(t2, 4);
    X(4, 5) = 1 * std::pow(t2, 5);
    X(4, 6) = 1 * std::pow(t2, 6);
    X(4, 7) = 1 * std::pow(t2, 7);

    X(5, 0) = 1;
    X(5, 1) = 1 * t1;
    X(5, 2) = 1 * std::pow(t3, 2);
    X(5, 3) = 1 * std::pow(t3, 3);
    X(5, 4) = 1 * std::pow(t3, 4);
    X(5, 5) = 1 * std::pow(t3, 5);
    X(5, 6) = 1 * std::pow(t3, 6);
    X(5, 7) = 1 * std::pow(t3, 7);

    X(6, 0) = 0;
    X(6, 1) = 1;
    X(6, 2) = 2 * t3;
    X(6, 3) = 3 * std::pow(t3, 2);
    X(6, 4) = 4 * std::pow(t3, 3);
    X(6, 5) = 5 * std::pow(t3, 4);
    X(6, 6) = 6 * std::pow(t3, 5);
    X(6, 7) = 7 * std::pow(t3, 6);

    X(7, 0) = 0;
    X(7, 1) = 0;
    X(7, 2) = 2;
    X(7, 3) = 6 * t3;
    X(7, 4) = 12 * std::pow(t3, 2);
    X(7, 5) = 20 * std::pow(t3, 3);
    X(7, 6) = 30 * std::pow(t3, 4);
    X(7, 7) = 42 * std::pow(t3, 5);

    B(0, 0) = vec_q0x[0];
    B(1, 0) = vec_q0x[1];
    B(2, 0) = vec_q0x[2];
    B(3, 0) = vec_qwx[0];
    B(4, 0) = vec_qwx[1];
    B(5, 0) = vec_q3x[0];
    B(6, 0) = vec_q3x[1];
    B(7, 0) = vec_q3x[2];

    return (X.inverse() * B);
}

std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> computeWaypointTraj(MatrixXf A, float t0, float t3, int n)
{

    std::vector<float> a = { A(0, 0), A(1, 0), A(2, 0), A(3, 0), A(4, 0), A(5, 0), A(6, 0), A(7, 0) };

    std::vector<float> qd;
    std::vector<float> d_qd;
    std::vector<float> dd_qd;
    std::vector<float> ddd_qd;
    std::vector<float> time;

    float step = (t3 - t0) / n;
    for (float t = t0; t < t3 + step; t += step)
    {

        float qdi = a[0] + a[1] * t + a[2] * std::pow(t, 2) + a[3] * std::pow(t, 3) + a[4] * std::pow(t, 4) + a[5] * std::pow(t, 5) + a[6] * std::pow(t, 6) + a[7] * std::pow(t, 7);
        float d_qdi = (a[1] + 2 * a[2] * t + 3 * a[3] * std::pow(t, 2) + 4 * a[4] * std::pow(t, 3) + 5 * a[5] * std::pow(t, 4) + 6 * a[6] * std::pow(t, 5) + 7 * a[7] * std::pow(t, 6)) / 2;
        float dd_qdi = (2 * a[2] + 6 * a[3] * t + 12 * a[4] * std::pow(t, 2) + 20 * a[5] * std::pow(t, 3) + 30 * a[6] * std::pow(t, 4) + 42 * a[7] * std::pow(t, 5)) / 20;
        float ddd_qdi = (6 * a[3] + 24 * a[4] * std::pow(t, 1) + 60 * a[5] * std::pow(t, 2) + 120 * a[6] * std::pow(t, 3) + 210 * a[7] * std::pow(t, 4)) / 500;

        qd.push_back(qdi);
        d_qd.push_back(d_qdi);
        dd_qd.push_back(dd_qdi);
        ddd_qd.push_back(ddd_qdi);
        time.push_back(t);
    }

    return std::make_tuple(time, qd, d_qd, dd_qd, ddd_qd);
}

std::tuple<std::vector<float>, std::vector<MatrixXf>, std::vector<MatrixXf>> computeCubicTraj_vect(MatrixXf q0 , MatrixXf qf, MatrixXf q0_d, MatrixXf qf_d, float t0, float tf, int n)
{
    MatrixXf a0 = q0;
    MatrixXf a1 = q0_d;
    MatrixXf a2 = (3/std::pow(tf,2)) *(qf-q0) - (2 / tf) * q0_d - (1/tf)*qf_d;
    MatrixXf a3 = (-2 / std::pow(tf, 3)) * (qf - q0) + (1 / tf) * (qf_d + q0_d);
    std::vector<MatrixXf> q, d_q;
    std::vector<float> time;
    float step = (tf - t0) / n;
    for (float t = t0; t < tf+step ; t+=step)
    {
        MatrixXf qi = (a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3));
        MatrixXf d_qi = (a1 + 2*a2* t + 3*a3 * std::pow(t, 2));
        q.push_back(qi);
        d_q.push_back(d_qi);
        time.push_back(t);
    }
    return std::make_tuple(time,q,d_q);
}