// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StepGenerator.hpp"

#include <cmath>

StepGenerator::StepGenerator(FootSpec _footSpec)
    : footSpec(_footSpec),
      squat(0.0),
      step(0.0)
{}

void StepGenerator::configure(double _squat, double _step, const KDL::Frame & _initialPose)
{
    squat = _squat;
    step = _step;
    initialPose = _initialPose;
}

void StepGenerator::generate(double distance, std::vector<KDL::Frame> & steps, std::vector<KDL::Frame> & com)
{
    steps.empty();
    com.empty();

    double initialSep = initialPose.p.y();
    double marginSep = footSpec.margin + footSpec.sep / 2.0;
    double stableSep = footSpec.stable + footSpec.sep / 2.0;
    double stepSep = (footSpec.sep + footSpec.width) / 2.0;

    double initialSquat = -initialPose.p.z();
    double hopSquat = initialSquat - squat + footSpec.hop;
    double maxSquat = initialSquat - squat;

    double marginFwd = footSpec.width / 2.0 - footSpec.margin;
    double stepFwd = footSpec.length - footSpec.margin - footSpec.width / 2.0;

    if (step < footSpec.length)
    {
        marginFwd *= step / footSpec.length;
        stepFwd *= step / footSpec.length;
    }

    KDL::Rotation rot = initialPose.M;

    steps.push_back(KDL::Frame(rot, KDL::Vector(0, -initialSep, 0)));
    steps.push_back(KDL::Frame(rot, KDL::Vector(0, initialSep, 0)));

    com.push_back(KDL::Frame(KDL::Vector(0, 0, initialSquat)));
    com.push_back(KDL::Frame(KDL::Vector(0, 0, hopSquat)));
    com.push_back(KDL::Frame(KDL::Vector(0, stableSep, hopSquat)));

    if (distance <= step)
    {
        stepFwd *= distance / step;

        steps.push_back(KDL::Frame(rot, KDL::Vector(distance, -initialSep, 0)));
        steps.push_back(KDL::Frame(rot, KDL::Vector(distance, initialSep, 0)));

        com.push_back(KDL::Frame(KDL::Vector(stepFwd, marginSep, hopSquat)));
        com.push_back(KDL::Frame(KDL::Vector(distance - marginFwd, -marginSep, maxSquat)));
        com.push_back(KDL::Frame(KDL::Vector(distance, -stableSep, hopSquat)));
        com.push_back(KDL::Frame(KDL::Vector(distance, 0, hopSquat)));
        com.push_back(KDL::Frame(KDL::Vector(distance, 0, initialSquat)));

        return;
    }

    double travelled = 0.0;
    bool movingRightFoot = true;

    do
    {
        double previousTravelled = travelled;
        travelled += step;
        bool isLastStep = travelled >= distance;
        travelled = std::min(travelled, distance);
        marginFwd *= (travelled - previousTravelled) / step;
        stepFwd *= (travelled - previousTravelled) / step;

        if (isLastStep)
        {
            if (movingRightFoot)
            {
                steps.push_back(KDL::Frame(rot, KDL::Vector(distance, -initialSep, 0)));
                steps.push_back(KDL::Frame(rot, KDL::Vector(distance, initialSep, 0)));

                com.push_back(KDL::Frame(KDL::Vector(previousTravelled + stepFwd, marginSep, maxSquat)));
                com.push_back(KDL::Frame(KDL::Vector(travelled - marginFwd, -marginSep, maxSquat)));
                com.push_back(KDL::Frame(KDL::Vector(travelled, -stableSep, hopSquat)));
                com.push_back(KDL::Frame(KDL::Vector(travelled, 0, hopSquat)));
                com.push_back(KDL::Frame(KDL::Vector(travelled, 0, initialSquat)));
            }
            else
            {
                steps.push_back(KDL::Frame(rot, KDL::Vector(distance, initialSep, 0)));
                steps.push_back(KDL::Frame(rot, KDL::Vector(distance, -initialSep, 0)));

                com.push_back(KDL::Frame(KDL::Vector(previousTravelled + stepFwd, -marginSep, maxSquat)));
                com.push_back(KDL::Frame(KDL::Vector(travelled - marginFwd, marginSep, maxSquat)));
                com.push_back(KDL::Frame(KDL::Vector(travelled, stableSep, hopSquat)));
                com.push_back(KDL::Frame(KDL::Vector(travelled, 0, hopSquat)));
                com.push_back(KDL::Frame(KDL::Vector(travelled, 0, initialSquat)));
            }

            break;
        }

        if (movingRightFoot)
        {
            steps.push_back(KDL::Frame(rot, KDL::Vector(travelled, -stepSep, 0)));

            com.push_back(KDL::Frame(KDL::Vector(previousTravelled + stepFwd, marginSep, maxSquat)));
            com.push_back(KDL::Frame(KDL::Vector(travelled - marginFwd, -marginSep, maxSquat)));
            com.push_back(KDL::Frame(KDL::Vector(travelled, -stableSep, hopSquat)));
        }
        else
        {
            steps.push_back(KDL::Frame(rot, KDL::Vector(travelled, stepSep, 0)));

            com.push_back(KDL::Frame(KDL::Vector(previousTravelled + stepFwd, -marginSep, maxSquat)));
            com.push_back(KDL::Frame(KDL::Vector(travelled - marginFwd, marginSep, maxSquat)));
            com.push_back(KDL::Frame(KDL::Vector(travelled, stableSep, hopSquat)));
        }

        movingRightFoot = !movingRightFoot;
    }
    while (true);
}
