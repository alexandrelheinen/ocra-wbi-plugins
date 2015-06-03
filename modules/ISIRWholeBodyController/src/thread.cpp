/*
* Copyright (C) 2013 ISIR
* Author: Darwin Lau, MingXing Liu, Ryan Lober
* email: lau@isir.upmc.fr
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#include <ISIRWholeBodyController/thread.h>
#include <ISIRWholeBodyController/ocraWbiModel.h>
#include <modHelp/modHelp.h>
#include <iostream>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>


//#include "wocra/Solvers/OneLevelSolver.h"
#include "wocra/Features/wOcraFeature.h"
#include "ocra/control/Feature.h"
#include "ocra/control/FullState.h"
#include "ocra/control/ControlFrame.h"
#include "ocra/control/ControlEnum.h"


using namespace ISIRWholeBodyController;
using namespace yarp::math;
using namespace yarpWbi;

#define ALL_JOINTS -1
#define DIM_DISP 3
#define DIM_TWIST 6
#define TORQUE_MIN -24
#define TORQUE_MAX 24
//#define HAND_FOOT_TASK 1
#define HAND_FOOT_TASK 0
#define TIME_MSEC_TO_SEC 0.001

//*************************************************************************************************************************
ISIRWholeBodyControllerThread::ISIRWholeBodyControllerThread(string _name,
                                                             string _robotName,
                                                             int _period,
                                                             wholeBodyInterface *_wbi,
                                                             yarp::os::Property &_options
                                                            )
    : RateThread(_period), name(_name), robotName(_robotName), robot(_wbi), options(_options)
{
    bool isFreeBase = false;
    ocraModel = new ocraWbiModel(robotName, robot->getDoFs(), robot, isFreeBase);
    bool useReducedProblem = false;
    ctrl = new wocra::wOcraController("icubControl", *ocraModel, internalSolver, useReducedProblem);

    fb_qRad = Eigen::VectorXd::Zero(robot->getDoFs());
    fb_qdRad = Eigen::VectorXd::Zero(robot->getDoFs());

    fb_Hroot = wbi::Frame();
    fb_Troot = Eigen::VectorXd::Zero(DIM_TWIST);

    fb_Hroot_Vector = yarp::sig::Vector(16, 0.0);
    fb_Troot_Vector = yarp::sig::Vector(6, 0.0);

    fb_torque.resize(robot->getDoFs());
    torques_cmd = yarp::sig::Vector(robot->getDoFs(), 0.0);

    time_sim = 0;

    /*
    *   File save related
    *   Make sure to change the baseFilePath to reflect your desired save locations

    *   Note: do NOT use the ~/ operator for home/user/ (it doesn't work with this usage of Boost)
    */

    ////////////////////////////////////////////////////////////////////
    bool saveData = true;
    std::string baseFilePath = "/home/ryan/Desktop/dataTest/"; //CHANGE ME!!!!
    ////////////////////////////////////////////////////////////////////

    leftArmFilePath = baseFilePath+"leftArmData.txt";
    rightArmFilePath = baseFilePath+"rightArmData.txt";


    if (saveData) {

        // Make directory if it doesn't already exist
        if (!boost::filesystem::exists(baseFilePath))
            boost::filesystem::create_directories(baseFilePath);


        // create txt files to save data
        leftArmDataFile.open(leftArmFilePath.c_str());
        leftArmDataFile.close();

        rightArmDataFile.open(rightArmFilePath.c_str());
        rightArmDataFile.close();

    }




}

//*************************************************************************************************************************
bool ISIRWholeBodyControllerThread::threadInit()
{
//    printPeriod = options.check("printPeriod",Value(1000.0),"Print a debug message every printPeriod milliseconds.").asDouble();
    robot->getEstimates(ESTIMATE_JOINT_POS, q_initial.data(), ALL_JOINTS);

    bool res_qrad = robot->getEstimates(ESTIMATE_JOINT_POS, fb_qRad.data(), ALL_JOINTS);
    bool res_qdrad = robot->getEstimates(ESTIMATE_JOINT_VEL, fb_qdRad.data(), ALL_JOINTS);

    if (!ocraModel->hasFixedRoot()){
        // Get root position as a 12x1 vector and get root vel as a 6x1 vector
        bool res_fb_Hroot_Vector = robot->getEstimates(ESTIMATE_BASE_POS, fb_Hroot_Vector.data());
        bool res_fb_Troot = robot->getEstimates(ESTIMATE_BASE_VEL, fb_Troot_Vector.data());
        // Convert to a wbi::Frame and a "fake" Twistd
        wbi::frameFromSerialization(fb_Hroot_Vector.data(), fb_Hroot);
        fb_Troot = Eigen::Twistd(fb_Troot_Vector[0], fb_Troot_Vector[1], fb_Troot_Vector[2], fb_Troot_Vector[3], fb_Troot_Vector[4], fb_Troot_Vector[5]);

        ocraModel->wbiSetState(fb_Hroot, fb_qRad, fb_Troot, fb_qdRad);
    }
    else
        ocraModel->setState(fb_qRad, fb_qdRad);

    // Set all declared joints in module to TORQUE mode
    bool res_setControlMode = robot->setControlMode(CTRL_MODE_TORQUE, 0, ALL_JOINTS);

    //================ SET UP TASK ===================//

    sequence = new Demonstrations();

    sequence->init(*ctrl, *ocraModel);


	return true;
}

//*************************************************************************************************************************
void ISIRWholeBodyControllerThread::run()
{

    bool res_qrad = robot->getEstimates(ESTIMATE_JOINT_POS, fb_qRad.data(), ALL_JOINTS);
    bool res_qdrad = robot->getEstimates(ESTIMATE_JOINT_VEL, fb_qdRad.data(), ALL_JOINTS);
    bool res_torque = robot->getEstimates(ESTIMATE_JOINT_TORQUE, fb_torque.data(), ALL_JOINTS);


    // SET THE STATE (FREE FLYER POSITION/VELOCITY AND Q)
    if (!ocraModel->hasFixedRoot()){
        // Get root position as a 12x1 vector and get root vel as a 6x1 vector
        bool res_fb_Hroot_Vector = robot->getEstimates(ESTIMATE_BASE_POS, fb_Hroot_Vector.data());
        bool res_fb_Troot = robot->getEstimates(ESTIMATE_BASE_VEL, fb_Troot_Vector.data());
        // Convert to a wbi::Frame and a "fake" Twistd
        wbi::frameFromSerialization(fb_Hroot_Vector.data(), fb_Hroot);

        fb_Troot = Eigen::Twistd(fb_Troot_Vector[0], fb_Troot_Vector[1], fb_Troot_Vector[2], fb_Troot_Vector[3], fb_Troot_Vector[4], fb_Troot_Vector[5]);

        ocraModel->wbiSetState(fb_Hroot, fb_qRad, fb_Troot, fb_qdRad);
    }
    else
        ocraModel->setState(fb_qRad, fb_qdRad);

    sequence->update(time_sim, *ocraModel, NULL);


    // compute desired torque by calling the controller
    Eigen::VectorXd eigenTorques = Eigen::VectorXd::Constant(ocraModel->nbInternalDofs(), 0.0);

	ctrl->computeOutput(eigenTorques);



    for(int i = 0; i < eigenTorques.size(); ++i)
    {
      if(eigenTorques(i) < TORQUE_MIN) eigenTorques(i) = TORQUE_MIN;
      else if(eigenTorques(i) > TORQUE_MAX) eigenTorques(i) = TORQUE_MAX;
    }

	modHelp::eigenToYarpVector(eigenTorques, torques_cmd);


    robot->setControlReference(torques_cmd.data());

    printPeriod = 1000;
    printCountdown = (printCountdown>=printPeriod) ? 0 : printCountdown + getRate(); // countdown for next print
    if (printCountdown == 0)
    {
        printData();
        recordData();
    }

    time_sim += TIME_MSEC_TO_SEC * getRate();
}

//*************************************************************************************************************************
void ISIRWholeBodyControllerThread::threadRelease()
{
    // Need to fix q_initial to be q_home: in case the robot starts in some stupid configuration.
    bool res_setControlMode = robot->setControlMode(CTRL_MODE_POS, q_initial.data(), ALL_JOINTS);

}



void ISIRWholeBodyControllerThread::printData()
{
    std::cout << "Current positions at time: "<< time_sim << std::endl;
    std::cout << "Left Hand:" << ocraModel->getSegmentPosition(ocraModel->getSegmentIndex("l_hand")).getTranslation().transpose()<< std::endl;
    std::cout << "Right Hand:" << ocraModel->getSegmentPosition(ocraModel->getSegmentIndex("r_hand")).getTranslation().transpose()<< std::endl;
}

void ISIRWholeBodyControllerThread::recordData()
{

    /*
    *   Save positions of the hands to text files.
    *   Format:
    *   |   relative time   |   X   |   Y   |   Z   |
    */

    std::ostringstream timeStream, lPosStream, rPosStream;
    timeStream << time_sim;
    lPosStream << ocraModel->getSegmentPosition(ocraModel->getSegmentIndex("l_hand")).getTranslation().transpose();
    rPosStream << ocraModel->getSegmentPosition(ocraModel->getSegmentIndex("r_hand")).getTranslation().transpose();
    std::string timeString = timeStream.str();
    std::string lPosString = lPosStream.str();
    std::string rPosString = rPosStream.str();


    leftArmDataFile.open(leftArmFilePath.c_str(), std::ios::app);
    leftArmDataFile << timeString << "\t" << lPosString << std::endl;
    leftArmDataFile.close();

    rightArmDataFile.open(rightArmFilePath.c_str(), std::ios::app);
    rightArmDataFile << timeString << "\t" << rPosString << std::endl;
    rightArmDataFile.close();
}
