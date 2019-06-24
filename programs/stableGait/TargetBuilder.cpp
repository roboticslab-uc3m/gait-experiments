// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TargetBuilder.hpp"

#include <cmath>

#include <yarp/os/Time.h>

#include <KdlVectorConverter.hpp>

TargetBuilder::TargetBuilder(rl::ICartesianControl * _iCartLeft, rl::ICartesianControl * _iCartRight)
    : iCartLeft(_iCartLeft),
      iCartRight(_iCartRight),
      tCom(0),
      tLeft(0),
      tRight(0),
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

bool TargetBuilder::validate(Targets & vLeft, Targets & vRight)
{
    std::vector<double> q;

    for (int i = 0; i < vLeft.size(); i++)
    {
        if (!iCartLeft->inv(vLeft[i], q))
        {
            CD_WARNING("IK failing at left leg: %f %f %f %f %f %f\n",
                vLeft[i][0], vLeft[i][1], vLeft[i][2], vLeft[i][3], vLeft[i][4], vLeft[i][5]);
            return false;
        }

        if (!iCartRight->inv(vRight[i], q))
        {
            CD_WARNING("IK failing at right leg: %f %f %f %f %f %f\n",
                vRight[i][0], vRight[i][1], vRight[i][2], vRight[i][3], vRight[i][4], vRight[i][5]);
            return false;
        }
    }

    return true;
}
