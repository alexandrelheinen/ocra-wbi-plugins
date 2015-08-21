#include <taskSequences/sequences/StageTestTasks.h>
// StageTestTasks
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
  void StageTestTasks::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
  {
    kp = 20.0;
    kd = 13.0;
    w  = 1.0;

    // full posture task
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nbInternalDofs());
    getHomePosture(model, q);
    taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, kp, kd, 10*w, q);
    // CoM task
    Eigen::Vector3d posCoM = model.getCoMPosition();
    taskManagers["tmCoM"] = new wocra::wOcraCoMTaskManager(ctrl, model, "CoMTask", kp, kd, 50*w, posCoM);

    // position sequences
    for (unsigned int i = 0; i < NB_POSTURES; ++i)                  // NB_POSTURES elements
    {
      rightHandPos.push_back(Eigen::Vector3d::Zero());
      leftHandPos.push_back(Eigen::Vector3d::Zero());
    }
    positionInit();

    leftHand  = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "leftHandCartesianTask",  "l_hand", ocra::XYZ, kp, kd, 1000*w, leftHandPos.at(0));
    rightHand = new wocra::wOcraSegCartesianTaskManager(ctrl, model, "rightHandCartesianTask", "r_hand", ocra::XYZ, kp, kd, 1000*w, rightHandPos.at(0));
    taskManagers["tmSegCartHandLeft"]  = leftHand;
    taskManagers["tmSegCartHandRight"] = rightHand;
    mode = 0;
  }

  void StageTestTasks::doUpdate(double time, wocra::wOcraModel& state, void** args)
  {
    // posture iteration, assures the sequence of NB_POSTURES postures at the vector q in a period of PERIOD seconds
    time -= PERIOD*floor(time/PERIOD); // normalized time

    if(time >= (mode + 1)*PERIOD/NB_POSTURES + 0.1  || (time < PERIOD/NB_POSTURES && mode == NB_POSTURES - 1))
    {
      std::cout << ":: POSTURE N° "      << mode + 1 << " :: " << std::endl
                << "left "               << std::endl
                << leftHandPos.at(mode)  << std::endl
                << "right "              << std::endl
                << rightHandPos.at(mode) << std::endl;

      leftHand->setState(leftHandPos.at(mode));
      rightHand->setState(rightHandPos.at(mode));
      mode = (mode + 1)%NB_POSTURES;
    }
  }

  void StageTestTasks::positionInit()
  {
    leftHandPos.at(0)  = Eigen::Vector3d(-0.2, -0.1, 0.1);
    leftHandPos.at(1)  = leftHandPos.at(0)  + Eigen::Vector3d(0.0,  0.05, 0.0);
    leftHandPos.at(2)  = leftHandPos.at(1)  + Eigen::Vector3d(0.0, -0.10, 0.0);
    leftHandPos.at(3)  = leftHandPos.at(1)  + Eigen::Vector3d(0.0,  0.10, 0.0);
    leftHandPos.at(4)  = leftHandPos.at(1);
    leftHandPos.at(5)  = leftHandPos.at(0);

    rightHandPos.at(0) = Eigen::Vector3d(-0.2, 0.1, 0.1);
    rightHandPos.at(1) = rightHandPos.at(0) + Eigen::Vector3d(0.0, -0.05, 0.0);
    rightHandPos.at(2) = rightHandPos.at(1) + Eigen::Vector3d(0.0, -0.10, 0.0);
    rightHandPos.at(3) = rightHandPos.at(1) + Eigen::Vector3d(0.0,  0.10, 0.0);
    rightHandPos.at(4) = rightHandPos.at(1);
    rightHandPos.at(5) = rightHandPos.at(0);
  }
