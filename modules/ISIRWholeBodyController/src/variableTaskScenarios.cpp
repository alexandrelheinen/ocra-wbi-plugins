#include "ISIRWholeBodyController/variableTaskScenarios.h"
#include "ISIRWholeBodyController/ocraWbiModel.h"


#include "wocra/Trajectory/wOcraMinimumJerkTrajectory.h"
#include "wocra/Trajectory/wOcraLinearInterpolationTrajectory.h"

#ifndef PI
#define PI 3.1415926
#endif

/*
*   Auxilliary Function declarations
*/

// Just sets some of the DoF to certain angles. The home position if you will
void getHomePosture(wocra::wOcraModel &model, VectorXd &q);


/*
*   Demonstrations
*   This set of tasks is used to perform the "demonstrations" of each hand task.
*/
void Demonstrations::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)

{
    ocraWbiModel& wbiModel = dynamic_cast<ocraWbiModel&>(model);
    // Posture task gains and weights
    double Kp = 10.0;
    double Kd = 2.0 * sqrt(Kp);
    double wFullPosture = 0.0001;
    double wPartialPosture = 0.1;

    // Hand task gains and weights
    double Kp_hand = 40.0;
    double Kd_hand = 2.0 * sqrt(Kp_hand);
    double wLeftHandTask = 1.0;
    double wRightHandTask = 1.0;


    /*
    *   Full posture task
    *   This task is used to ensure that the robot stays in some home position in the case that there are no other tasks.
    */
    Eigen::VectorXd nominal_q = Eigen::VectorXd::Zero(model.nbInternalDofs()); //Get a vector of zeros the size of the number of DoF
    getHomePosture(model, nominal_q); // Set a few of those DoF to particular "home" angles.
    // Initialize the full posture task
    fullPostureTask = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, Kp, Kd, wFullPosture, nominal_q);

    /*
    *   Partial (torso) posture task
    *   This task is useful with hand reaching tasks because it manages how much the controller will exploit the torso DoF to perform a reach.
    *   If its weight is too low then the torso may move around dangerously, very high and the torso will remain rigid.
    */

    // set up the DoF index vector, and the desired angles vector.
    Eigen::VectorXi torso_indices(3);
    Eigen::VectorXd torsoTaskPosDes(3);
    torso_indices << wbiModel.getDofIndex("torso_pitch"), wbiModel.getDofIndex("torso_roll"), wbiModel.getDofIndex("torso_yaw");
    torsoTaskPosDes << 0.0, 0.0, 0.0;
    // Initialize the torso posture task
    torsoPostureTask = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, torso_indices, 6., 2.0 * sqrt(6.), wPartialPosture, torsoTaskPosDes);


    /*
    *   Left and Right Hand Tasks
    *   Here we will initialize the left and right hand tasks and their trajectories.
    */

    // Get the right and left hand segement indexes
    rHandIndex = model.getSegmentIndex("r_hand");
    lHandIndex = model.getSegmentIndex("l_hand");

    // Get their starting position
    Eigen::Vector3d rHandStartingPos = model.getSegmentPosition(rHandIndex).getTranslation();
    Eigen::Vector3d lHandStartingPos = model.getSegmentPosition(lHandIndex).getTranslation();

    // Initialize the right hand task and tell it to stay where it started
    rightHandTask = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "rightHandTask", "r_hand", ocra::XYZ, Kp_hand, Kd_hand, wRightHandTask, rHandStartingPos);

    // Initialize the left hand task and tell it to stay where it started
    leftHandTask = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "leftHandTask", "l_hand", ocra::XYZ, Kp_hand, Kd_hand, wLeftHandTask, lHandStartingPos);


    //*************** Type of Trajectory ******************//
    bool isLinInterp = false;
    bool isMinJerk = true;
    //*****************************************************//

    Eigen::Vector3d lHandDesiredPos, YZ_disp;
    YZ_disp << -0.15, 0.1, 0.2;
    lHandDesiredPos  = lHandStartingPos + YZ_disp;

    // multiple position lHandWaypoints
    Eigen::MatrixXd lHandWaypoints(3,5);
    Eigen::MatrixXd squareDisplacement(3,5);
    lHandWaypoints << lHandStartingPos, lHandStartingPos, lHandStartingPos, lHandStartingPos, lHandStartingPos;
    squareDisplacement << 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.2, 0.2, 0.0, 0.0,
                          0.0, 0.0, 0.2, 0.2, 0.0;
    lHandWaypoints += squareDisplacement;

    if (isLinInterp)
    {
        // leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(lHandStartingPos, lHandDesiredPos);
        leftHandTrajectory = new wocra::wOcraLinearInterpolationTrajectory(lHandWaypoints);
    }
    else if (isMinJerk)
    {
        // leftHandTrajectory = new wocra::wOcraMinimumJerkTrajectory(lHandStartingPos, lHandDesiredPos);
        leftHandTrajectory = new wocra::wOcraMinimumJerkTrajectory(lHandWaypoints);
    }


    leftHandTrajectory->generateTrajectory(3.0); // set a 4 second duration


}



void Demonstrations::doUpdate(double time, wocra::wOcraModel& state, void** args)
{

    Eigen::MatrixXd desiredPosVelAcc = leftHandTrajectory->getDesiredValues(time);

    leftHandTask->setState(desiredPosVelAcc.col(0));//,  desiredPosVelAcc.col(1), desiredPosVelAcc.col(2));


}



////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////


/*
* Auxilliary functions
*/

// Get the "home" position angles
void getHomePosture(wocra::wOcraModel& model, VectorXd &q)
{
    q[model.getDofIndex("r_elbow")] = M_PI / 4;
    q[model.getDofIndex("l_elbow")] = M_PI / 4;
    q[model.getDofIndex("l_shoulder_roll")] = M_PI / 6;
    q[model.getDofIndex("r_shoulder_roll")] = M_PI / 6;
    q[model.getDofIndex("l_shoulder_pitch")] = -M_PI / 6;
    q[model.getDofIndex("r_shoulder_pitch")] = -M_PI / 6;

}
