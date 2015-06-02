#include "ISIRWholeBodyController/variableTaskScenarios.h"
#include "ISIRWholeBodyController/ocraWbiModel.h"


#include "wocra/Trajectory/wOcraMinimumJerkTrajectory.h"
#include "wocra/Trajectory/wOcraLinearInterpolationTrajectory.h"

#ifndef PI
#define PI 3.1415926
#endif




// 01_Demonstration
void 01_Demonstration::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
{
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);

    // Task Coeffs
    double Kp = 10.0;
    double Kd = 2.0 * sqrt(Kp);

    double Kp_hand = 40.0;
    double Kd_hand = 8.0 ;//* sqrt(Kp_hand);
    double wFullPosture = 0.0001;
    double wPartialPosture = 0.1;
    double wLeftHandTask = 1.0;

    // Full posture task
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getNominalPosture(model, nominal_q);
    fullPostureTask = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp, Kd, wFullPosture, nominal_q);

    // Partial (torso) posture task
    Eigen::VectorXi torso_indices(3);
    Eigen::VectorXd torsoTaskPosDes(3);
    torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
    torsoTaskPosDes << 0.0, 0.0, 0.0;
    torsoPostureTask = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, 6., 2.0 * sqrt(6.), wPartialPosture, torsoTaskPosDes);

    // Hand Tasks
    rHandIndex = model.getSegmentIndex("r_hand");
    lHandIndex = model.getSegmentIndex("l_hand");


    Eigen::Vector3d rHandStartingPos = model.getSegmentPosition(rHandIndex).getTranslation();
    Eigen::Vector3d lHandStartingPos = model.getSegmentPosition(rHandIndex).getTranslation();


    rightHandTask = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "rightHandCartesianTask", "r_hand", ocra::XYZ, Kp_hand, Kd_hand, 1.0, rHandStartingPos);

    leftHandTask      = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask", "l_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, lHandStartingPos);


    //*************** Type of Trajectory ******************//
    bool isLinInterp = false;
    bool isMinJerk = true;
    //*****************************************************//


    Eigen::Vector3d desiredPos, YZ_disp;
    YZ_disp << -0.15, 0.1, 0.2;
    desiredPos  = lHandStartingPos + YZ_disp;

    // multiple position waypoints
    Eigen::MatrixXd waypoints(3,5);
    Eigen::MatrixXd squareDisplacement(3,5);
    waypoints << lHandStartingPos, lHandStartingPos, lHandStartingPos, lHandStartingPos, lHandStartingPos;
    squareDisplacement << 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.2, 0.2, 0.0, 0.0,
                          0.0, 0.0, 0.2, 0.2, 0.0;
    waypoints += squareDisplacement;

    if (isLinInterp)
    {
        /**
        * Linear interpolation trajectory constructor tests:
        */
        leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(startingPos, desiredPos);
        // leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(waypoints);
    }
    }
    else if (isMinJerk)
    {
        /**
        * Minimum jerk trajectory constructor tests:
        */
        leftHandTrajectory = new wocra::wOcraMinimumJerkTrajectory(startingPos, desiredPos);
        // leftHandTrajectory = new wocra::wOcraMinimumJerkTrajectory(waypoints);
    }


    leftHandTrajectory->generateTrajectory(3.0); // set a 4 second duration


}



void 01_Demonstration::doUpdate(double time, wocra::wOcraModel& state, void** args)
{
    Eigen::MatrixXd desiredPosVelAcc = leftHandTrajectory->getDesiredValues(time);
    // Eigen::MatrixXd H_adj = state.getSegmentPosition(lHandIndex).getRotation().inverse().adjoint();
    // H_adj*
    leftHandTask->setState(desiredPosVelAcc.col(0));//,  desiredPosVelAcc.col(1), desiredPosVelAcc.col(2));

    // if(isCartesion){std::cout << "\nFinal desired position: " << desiredPos.transpose() << std::endl;}
    // std::cout << "\nDesired position: " << desiredPosVelAcc.col(0).transpose() << std::endl;
    // std::cout << "Current position: " << state.getSegmentPosition(lHandIndex).getTranslation().transpose()<< std::endl;
    // std::cout << "Error: " << leftHandTask->getTaskError().transpose() << "   norm: " << leftHandTask->getTaskErrorNorm() << std::endl;

}
