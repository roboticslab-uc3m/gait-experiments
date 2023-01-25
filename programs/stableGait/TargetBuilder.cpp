// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TargetBuilder.hpp"

#include <algorithm> // std::max

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include <KdlVectorConverter.hpp>

TargetBuilder::TargetBuilder(rl::ICartesianSolver * _solverLeft, rl::ICartesianSolver * _solverRight,
                             const std::vector<double> & _initialJointLeft, const std::vector<double> & _initialJointRight)
    : solverLeft(_solverLeft),
      solverRight(_solverRight),
      initialJointLeft(_initialJointLeft),
      initialJointRight(_initialJointRight),
      tCom(nullptr),
      tLeft(nullptr),
      tRight(nullptr),
      maxTime(0.0)
{}

void TargetBuilder::configure(KDL::Trajectory * _tCom, KDL::Trajectory * _tLeft, KDL::Trajectory * _tRight)
{
    tCom = _tCom;
    tLeft = _tLeft;
    tRight = _tRight;

    maxTime = std::max(tCom->Duration(), std::max(tLeft->Duration(), tRight->Duration()));
}

void TargetBuilder::build(double period, Targets & vLeft, Targets & vRight)
{
    vLeft.clear();
    vRight.clear();

    double t = 0.0;

    while (t < maxTime)
    {
        KDL::Frame H_com = tCom->Pos(t);
        KDL::Frame H_left = tLeft->Pos(t);
        KDL::Frame H_right = tRight->Pos(t);

        KDL::Frame H_com_inv = H_com.Inverse();
        KDL::Frame H_com_left = H_com_inv * H_left;
        KDL::Frame H_com_right = H_com_inv * H_right;

        vLeft.push_back(rl::KdlVectorConverter::frameToVector(H_com_left));
        vRight.push_back(rl::KdlVectorConverter::frameToVector(H_com_right));

        t += period;
    }
}

bool TargetBuilder::validate(const Targets & vLeft, const Targets & vRight)
{
    std::vector<double> qLeft = initialJointLeft;
    std::vector<double> qRight = initialJointRight;

    for (int i = 0; i < vLeft.size(); i++)
    {
        if (!solverLeft->invKin(vLeft[i], qLeft, qLeft))
        {
            yWarning() << "IK failing at left leg:" << vLeft[i];
            return false;
        }

        if (!solverRight->invKin(vRight[i], qRight, qRight))
        {
            yWarning() << "IK failing at right leg:" << vRight[i];
            return false;
        }
    }

    return true;
}
