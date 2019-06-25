// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StepGenerator.hpp"

#include <cmath>

StepGenerator::StepGenerator(FootSpec _footSpec, const KDL::Frame & _initialPose)
    : footSpec(_footSpec),
      initialPose(_initialPose)
{}

void StepGenerator::configure(GaitSpec _gaitSpec)
{
    gaitSpec = _gaitSpec;
}

void StepGenerator::generate(double distance, std::vector<KDL::Frame> & steps, std::vector<KDL::Frame> & com)
{
    steps.empty();
    com.empty();

    double initialSep = initialPose.p.y();
    double marginSep = footSpec.margin + gaitSpec.sep / 2.0;
    double stableSep = footSpec.stable + gaitSpec.sep / 2.0;
    double stepSep = (gaitSpec.sep + footSpec.width) / 2.0;

    double initialSquat = -initialPose.p.z();
    double hopSquat = initialSquat - gaitSpec.squat + gaitSpec.hop;
    double maxSquat = initialSquat - gaitSpec.squat;

    double marginFwd = footSpec.width / 2.0 - footSpec.margin;
    double stepFwd = footSpec.length - footSpec.margin - footSpec.width / 2.0;

    if (gaitSpec.step < footSpec.length)
    {
        marginFwd *= gaitSpec.step / footSpec.length;
        stepFwd *= gaitSpec.step / footSpec.length;
    }

    KDL::Rotation rot = initialPose.M;

    steps.push_back(KDL::Frame(rot, KDL::Vector(0, -initialSep, 0)));
    steps.push_back(KDL::Frame(rot, KDL::Vector(0, initialSep, 0)));

    com.push_back(KDL::Frame(KDL::Vector(0, 0, initialSquat)));
    com.push_back(KDL::Frame(KDL::Vector(0, 0, hopSquat)));
    com.push_back(KDL::Frame(KDL::Vector(0, stableSep, hopSquat)));

    if (distance <= gaitSpec.step)
    {
        stepFwd *= distance / gaitSpec.step;

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
        travelled += gaitSpec.step;
        bool isLastStep = travelled >= distance;
        travelled = std::min(travelled, distance);
        marginFwd *= (travelled - previousTravelled) / gaitSpec.step;
        stepFwd *= (travelled - previousTravelled) / gaitSpec.step;

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
