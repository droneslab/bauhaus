/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "ImuTypes.h"
#include "../orbslam_types/Converter.h"

#include "../orbslam_types/GeometricTools.h"
#include "../../../target/cxxbridge/g2o/src/lib.rs.h"

#include<iostream>

namespace g2o
{

namespace IMU
{

const float eps = 1e-4;

Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R){
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z)
{
    Eigen::Matrix3f I;
    I.setIdentity();
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    Eigen::Vector3f v;
    v << x, y, z;
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);
    if(d<eps) {
        return I;
    }
    else {
        return I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v)
{
    return RightJacobianSO3(v(0),v(1),v(2));
}

Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y, const float &z)
{
    Eigen::Matrix3f I;
    I.setIdentity();
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    Eigen::Vector3f v;
    v << x, y, z;
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);

    if(d<eps) {
        return I;
    }
    else {
        return I + W/2 + W*W*(1.0f/d2 - (1.0f+cos(d))/(2.0f*d*sin(d)));
    }
}

Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v)
{
    return InverseRightJacobianSO3(v(0),v(1),v(2));
}

IntegratedRotation::IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time) {
    const float x = (angVel(0)-imuBias.bwx)*time;
    const float y = (angVel(1)-imuBias.bwy)*time;
    const float z = (angVel(2)-imuBias.bwz)*time;

    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);

    Eigen::Vector3f v;
    v << x, y, z;
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);
    if(d<eps)
    {
        deltaR = Eigen::Matrix3f::Identity() + W;
        rightJ = Eigen::Matrix3f::Identity();
    }
    else
    {
        deltaR = Eigen::Matrix3f::Identity() + W*sin(d)/d + W*W*(1.0f-cos(d))/d2;
        rightJ = Eigen::Matrix3f::Identity() - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

Preintegrated::Preintegrated(const Bias &b_, const Calib &calib)
{
    Nga = calib.Cov;
    NgaWalk = calib.CovWalk;
    Initialize(b_);
}


// Copy constructor
Preintegrated::Preintegrated(Preintegrated* pImuPre): dT(pImuPre->dT),C(pImuPre->C), Info(pImuPre->Info),
     Nga(pImuPre->Nga), NgaWalk(pImuPre->NgaWalk), b(pImuPre->b), dR(pImuPre->dR), dV(pImuPre->dV),
    dP(pImuPre->dP), JRg(pImuPre->JRg), JVg(pImuPre->JVg), JVa(pImuPre->JVa), JPg(pImuPre->JPg), JPa(pImuPre->JPa),
    avgA(pImuPre->avgA), avgW(pImuPre->avgW), bu(pImuPre->bu), db(pImuPre->db), mvMeasurements(pImuPre->mvMeasurements)
{

}

// Sofiya... create C++ Preintegrated type from data from rust
Preintegrated::Preintegrated(g2o::RustImuPreintegrated * pre)
{
    // Sofiya... not setting all the variables, just the ones we need for the g2o optimization, because I don't want to figure it out
    // These are used by the EdgeInertialGS object (G2oTypes.cc), in calls to:
    // GetDeltaRotation, GetDeltaVelocity, GetDeltaPosition, GetDeltaBias
    // on the mpInt object
    b = IMU::Bias(pre->bias.b_acc_x, pre->bias.b_acc_y, pre->bias.b_acc_z, pre->bias.b_ang_vel_x, pre->bias.b_ang_vel_y, pre->bias.b_ang_vel_z);

    // Nga = calib.Cov;
    // NgaWalk = calib.CovWalk;
    Initialize(b);

    dT = pre->t;

    JRg << pre->jrg[0][0], pre->jrg[0][1], pre->jrg[0][2],
        pre->jrg[1][0], pre->jrg[1][1], pre->jrg[1][2],
        pre->jrg[2][0], pre->jrg[2][1], pre->jrg[2][2];
    JPg << pre->jpg[0][0], pre->jpg[0][1], pre->jpg[0][2],
        pre->jpg[1][0], pre->jpg[1][1], pre->jpg[1][2],
        pre->jpg[2][0], pre->jpg[2][1], pre->jpg[2][2];
    JPa << pre->jpa[0][0], pre->jpa[0][1], pre->jpa[0][2],
        pre->jpa[1][0], pre->jpa[1][1], pre->jpa[1][2],
        pre->jpa[2][0], pre->jpa[2][1], pre->jpa[2][2];
    JVg << pre->jvg[0][0], pre->jvg[0][1], pre->jvg[0][2],
        pre->jvg[1][0], pre->jvg[1][1], pre->jvg[1][2],
        pre->jvg[2][0], pre->jvg[2][1], pre->jvg[2][2];
    JVa << pre->jva[0][0], pre->jva[0][1], pre->jva[0][2],
        pre->jva[1][0], pre->jva[1][1], pre->jva[1][2],
        pre->jva[2][0], pre->jva[2][1], pre->jva[2][2];    

    dR << pre->dr[0][0], pre->dr[0][1], pre->dr[0][2],
        pre->dr[1][0], pre->dr[1][1], pre->dr[1][2],
        pre->dr[2][0], pre->dr[2][1], pre->dr[2][2];

    dV = Eigen::Vector3f(pre->dv.data());
    db = Eigen::Matrix<float,6,1>(pre->db.data());
    dP = Eigen::Vector3f(pre->dp.data());

    avgA = Eigen::Vector3f(pre->avga.data());
    avgW = Eigen::Vector3f(pre->avgw.data());

    C.setZero();
    Eigen::Matrix<float,15,15> C_;
    C_ << pre->c[0][0], pre->c[0][1], pre->c[0][2], pre->c[0][3], pre->c[0][4], pre->c[0][5], pre->c[0][6], pre->c[0][7], pre->c[0][8], pre->c[0][9], pre->c[0][10],  pre->c[0][11],  pre->c[0][12],  pre->c[0][13],  pre->c[0][14],
        pre->c[1][0], pre->c[1][1], pre->c[1][2], pre->c[1][3], pre->c[1][4], pre->c[1][5], pre->c[1][6], pre->c[1][7], pre->c[1][8], pre->c[1][9], pre->c[1][10],  pre->c[1][11],  pre->c[1][12],  pre->c[1][13],  pre->c[1][14],
        pre->c[2][0], pre->c[2][1], pre->c[2][2], pre->c[2][3], pre->c[2][4], pre->c[2][5], pre->c[2][6], pre->c[2][7], pre->c[2][8], pre->c[2][9], pre->c[2][10],  pre->c[2][11],  pre->c[2][12],  pre->c[2][13],  pre->c[2][14],
        pre->c[3][0], pre->c[3][1], pre->c[3][2], pre->c[3][3], pre->c[3][4], pre->c[3][5], pre->c[3][6], pre->c[3][7], pre->c[3][8], pre->c[3][9], pre->c[3][10],  pre->c[3][11],  pre->c[3][12],  pre->c[3][13],  pre->c[3][14],
        pre->c[4][0], pre->c[4][1], pre->c[4][2], pre->c[4][3], pre->c[4][4], pre->c[4][5], pre->c[4][6], pre->c[4][7], pre->c[4][8], pre->c[4][9], pre->c[4][10],  pre->c[4][11],  pre->c[4][12],  pre->c[4][13],  pre->c[4][14],
        pre->c[5][0], pre->c[5][1], pre->c[5][2], pre->c[5][3], pre->c[5][4], pre->c[5][5], pre->c[5][6], pre->c[5][7], pre->c[5][8], pre->c[5][9], pre->c[5][10],  pre->c[5][11],  pre->c[5][12],  pre->c[5][13],  pre->c[5][14],
        pre->c[6][0], pre->c[6][1], pre->c[6][2], pre->c[6][3], pre->c[6][4], pre->c[6][5], pre->c[6][6], pre->c[6][7], pre->c[6][8], pre->c[6][9], pre->c[6][10],  pre->c[6][11],  pre->c[6][12],  pre->c[6][13],  pre->c[6][14],
        pre->c[7][0], pre->c[7][1], pre->c[7][2], pre->c[7][3], pre->c[7][4], pre->c[7][5], pre->c[7][6], pre->c[7][7], pre->c[7][8], pre->c[7][9], pre->c[7][10],  pre->c[7][11],  pre->c[7][12],  pre->c[7][13],  pre->c[7][14],
        pre->c[8][0], pre->c[8][1], pre->c[8][2], pre->c[8][3], pre->c[8][4], pre->c[8][5], pre->c[8][6], pre->c[8][7], pre->c[8][8], pre->c[8][9],  pre->c[8][10],  pre->c[8][11],  pre->c[8][12],  pre->c[8][13],  pre->c[8][14],
        pre->c[9][0], pre->c[9][1], pre->c[9][2], pre->c[9][3], pre->c[9][4], pre->c[9][5], pre->c[9][6], pre->c[9][7], pre->c[9][8], pre->c[9][9], pre->c[9][10],  pre->c[9][11],  pre->c[9][12],  pre->c[9][13],  pre->c[9][14],
        pre->c[10][0], pre->c[10][1], pre->c[10][2], pre->c[10][3], pre->c[10][4], pre->c[10][5], pre->c[10][6], pre->c[10][7], pre->c[10][8], pre->c[10][9], pre->c[10][10], pre->c[10][11], pre->c[10][12], pre->c[10][13], pre->c[10][14],
        pre->c[11][0], pre->c[11][1], pre->c[11][2], pre->c[11][3], pre->c[11][4], pre->c[11][5], pre->c[11][6], pre->c[11][7], pre->c[11][8], pre->c[11][9], pre->c[11][10], pre->c[11][11], pre->c[11][12], pre->c[11][13], pre->c[11][14],
        pre->c[12][0], pre->c[12][1], pre->c[12][2], pre->c[12][3], pre->c[12][4], pre->c[12][5], pre->c[12][6], pre->c[12][7], pre->c[12][8], pre->c[12][9], pre->c[12][10], pre->c[12][11], pre->c[12][12], pre->c[12][13], pre->c[12][14],
        pre->c[13][0], pre->c[13][1], pre->c[13][2], pre->c[13][3], pre->c[13][4], pre->c[13][5], pre->c[13][6], pre->c[13][7], pre->c[13][8], pre->c[13][9], pre->c[13][10], pre->c[13][11], pre->c[13][12], pre->c[13][13], pre->c[13][14],
        pre->c[14][0], pre->c[14][1], pre->c[14][2], pre->c[14][3], pre->c[14][4], pre->c[14][5], pre->c[14][6], pre->c[14][7], pre->c[14][8], pre->c[14][9], pre->c[14][10], pre->c[14][11], pre->c[14][12], pre->c[14][13], pre->c[14][14];

    C = C_;
    pre->c.data();
}

void Preintegrated::CopyFrom(Preintegrated* pImuPre)
{
    dT = pImuPre->dT;
    C = pImuPre->C;
    Info = pImuPre->Info;
    Nga = pImuPre->Nga;
    NgaWalk = pImuPre->NgaWalk;
    b.CopyFrom(pImuPre->b);
    dR = pImuPre->dR;
    dV = pImuPre->dV;
    dP = pImuPre->dP;
    JRg = pImuPre->JRg;
    JVg = pImuPre->JVg;
    JVa = pImuPre->JVa;
    JPg = pImuPre->JPg;
    JPa = pImuPre->JPa;
    avgA = pImuPre->avgA;
    avgW = pImuPre->avgW;
    bu.CopyFrom(pImuPre->bu);
    db = pImuPre->db;
    mvMeasurements = pImuPre->mvMeasurements;
}


void Preintegrated::Initialize(const Bias &b_)
{
    dR.setIdentity();
    dV.setZero();
    dP.setZero();
    JRg.setZero();
    JVg.setZero();
    JVa.setZero();
    JPg.setZero();
    JPa.setZero();
    C.setZero();
    Info.setZero();
    db.setZero();
    b=b_;
    bu=b_;
    avgA.setZero();
    avgW.setZero();
    dT=0.0f;
    mvMeasurements.clear();
}

void Preintegrated::Reintegrate()
{
    const std::vector<integrable> aux = mvMeasurements;
    Initialize(bu);
    for(size_t i=0;i<aux.size();i++)
        IntegrateNewMeasurement(aux[i].a,aux[i].w,aux[i].t);
}

void Preintegrated::IntegrateNewMeasurement(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &angVel, const float &dt)
{
    mvMeasurements.push_back(integrable(acceleration,angVel,dt));

    // Position is updated firstly, as it depends on previously computed velocity and rotation.
    // Velocity is updated secondly, as it depends on previously computed rotation.
    // Rotation is the last to be updated.

    //Matrices to compute covariance
    Eigen::Matrix<float,9,9> A;
    A.setIdentity();
    Eigen::Matrix<float,9,6> B;
    B.setZero();

    Eigen::Vector3f acc, accW;
    acc << acceleration(0)-b.bax, acceleration(1)-b.bay, acceleration(2)-b.baz;
    accW << angVel(0)-b.bwx, angVel(1)-b.bwy, angVel(2)-b.bwz;

    avgA = (dT*avgA + dR*acc*dt)/(dT+dt);
    avgW = (dT*avgW + accW*dt)/(dT+dt);

    // Update delta position dP and velocity dV (rely on no-updated delta rotation)
    dP = dP + dV*dt + 0.5f*dR*acc*dt*dt;
    dV = dV + dR*acc*dt;

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    Eigen::Matrix<float,3,3> Wacc = Sophus::SO3f::hat(acc);

    A.block<3,3>(3,0) = -dR*dt*Wacc;
    A.block<3,3>(6,0) = -0.5f*dR*dt*dt*Wacc;
    A.block<3,3>(6,3) = Eigen::DiagonalMatrix<float,3>(dt, dt, dt);
    B.block<3,3>(3,3) = dR*dt;
    B.block<3,3>(6,3) = 0.5f*dR*dt*dt;


    // Update position and velocity jacobians wrt bias correction
    JPa = JPa + JVa*dt -0.5f*dR*dt*dt;
    JPg = JPg + JVg*dt -0.5f*dR*dt*dt*Wacc*JRg;
    JVa = JVa - dR*dt;
    JVg = JVg - dR*dt*Wacc*JRg;

    // Update delta rotation
    IntegratedRotation dRi(angVel,b,dt);
    dR = NormalizeRotation(dR*dRi.deltaR);

    // Compute rotation parts of matrices A and B
    A.block<3,3>(0,0) = dRi.deltaR.transpose();
    B.block<3,3>(0,0) = dRi.rightJ*dt;

    // Update covariance
    C.block<9,9>(0,0) = A * C.block<9,9>(0,0) * A.transpose() + B*Nga*B.transpose();
    C.block<6,6>(9,9) += NgaWalk;

    // Update rotation jacobian wrt bias correction
    JRg = dRi.deltaR.transpose()*JRg - dRi.rightJ*dt;

    // Total integrated time
    dT += dt;
}

void Preintegrated::MergePrevious(Preintegrated* pPrev)
{
    if (pPrev==this)
        return;

    Bias bav;
    bav.bwx = bu.bwx;
    bav.bwy = bu.bwy;
    bav.bwz = bu.bwz;
    bav.bax = bu.bax;
    bav.bay = bu.bay;
    bav.baz = bu.baz;

    const std::vector<integrable > aux1 = pPrev->mvMeasurements;
    const std::vector<integrable> aux2 = mvMeasurements;

    Initialize(bav);
    for(size_t i=0;i<aux1.size();i++)
        IntegrateNewMeasurement(aux1[i].a,aux1[i].w,aux1[i].t);
    for(size_t i=0;i<aux2.size();i++)
        IntegrateNewMeasurement(aux2[i].a,aux2[i].w,aux2[i].t);

}

void Preintegrated::SetNewBias(const Bias &bu_)
{
    bu = bu_;

    db(0) = bu_.bwx-b.bwx;
    db(1) = bu_.bwy-b.bwy;
    db(2) = bu_.bwz-b.bwz;
    db(3) = bu_.bax-b.bax;
    db(4) = bu_.bay-b.bay;
    db(5) = bu_.baz-b.baz;
}

IMU::Bias Preintegrated::GetDeltaBias(const Bias &b_)
{
    return IMU::Bias(b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz,b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
}


Eigen::Matrix3f Preintegrated::GetDeltaRotation(const Bias &b_)
{
    Eigen::Vector3f dbg;
    dbg << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz;
    if(dbg.array().isNaN()[0]){
        dbg = Eigen::Vector3f(0,0,0);
    }

    return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * dbg).matrix());
}

Eigen::Vector3f Preintegrated::GetDeltaVelocity(const Bias &b_)
{
    Eigen::Vector3f dbg, dba;
    dbg << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz;
    dba << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz;
    return dV + JVg * dbg + JVa * dba;
}

Eigen::Vector3f Preintegrated::GetDeltaPosition(const Bias &b_)
{
    Eigen::Vector3f dbg, dba;
    dbg << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz;
    dba << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz;
    return dP + JPg * dbg + JPa * dba;
}

Eigen::Matrix3f Preintegrated::GetUpdatedDeltaRotation()
{
    return NormalizeRotation(dR * Sophus::SO3f::exp(JRg*db.head(3)).matrix());
}

Eigen::Vector3f Preintegrated::GetUpdatedDeltaVelocity()
{
    return dV + JVg * db.head(3) + JVa * db.tail(3);
}

Eigen::Vector3f Preintegrated::GetUpdatedDeltaPosition()
{
    return dP + JPg*db.head(3) + JPa*db.tail(3);
}

Eigen::Matrix3f Preintegrated::GetOriginalDeltaRotation() {
    return dR;
}

Eigen::Vector3f Preintegrated::GetOriginalDeltaVelocity() {
    return dV;
}

Eigen::Vector3f Preintegrated::GetOriginalDeltaPosition()
{
    return dP;
}

Bias Preintegrated::GetOriginalBias()
{
    return b;
}

Bias Preintegrated::GetUpdatedBias()
{
    return bu;
}

Eigen::Matrix<float,6,1> Preintegrated::GetDeltaBias()
{
    return db;
}

void Bias::CopyFrom(Bias &b)
{
    bax = b.bax;
    bay = b.bay;
    baz = b.baz;
    bwx = b.bwx;
    bwy = b.bwy;
    bwz = b.bwz;
}

std::ostream& operator<< (std::ostream &out, const Bias &b)
{
    if(b.bwx>0)
        out << " ";
    out << b.bwx << ",";
    if(b.bwy>0)
        out << " ";
    out << b.bwy << ",";
    if(b.bwz>0)
        out << " ";
    out << b.bwz << ",";
    if(b.bax>0)
        out << " ";
    out << b.bax << ",";
    if(b.bay>0)
        out << " ";
    out << b.bay << ",";
    if(b.baz>0)
        out << " ";
    out << b.baz;

    return out;
}

void Calib::Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na, const float &ngw, const float &naw) {
    mbIsSet = true;
    const float ng2 = ng*ng;
    const float na2 = na*na;
    const float ngw2 = ngw*ngw;
    const float naw2 = naw*naw;

    // Sophus/Eigen
    mTbc = sophTbc;
    mTcb = mTbc.inverse();
    Cov.diagonal() << ng2, ng2, ng2, na2, na2, na2;
    CovWalk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;
}

Calib::Calib(const Calib &calib)
{
    mbIsSet = calib.mbIsSet;
    // Sophus/Eigen parameters
    mTbc = calib.mTbc;
    mTcb = calib.mTcb;
    Cov = calib.Cov;
    CovWalk = calib.CovWalk;
}

} //namespace IMU

} //namespace ORB_SLAM2
