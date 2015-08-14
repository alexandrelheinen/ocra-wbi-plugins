#include <taskSequences/sequences/StageTestTasks.h>
// StageTestTasks
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
  void StageTestTasks::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
  {
    for (unsigned int i = 0; i < NB_POSTURES; ++i)                  // NB_POSTURES elements
      q.push_back(Eigen::VectorXd::Zero(model.nbInternalDofs()));

    for (int k = 0; k < NB_POSTURES; k++)
    {
      if(k > 0)
        q.at(k) = q.at(k-1);
      switch(k)
      {
        case 1: closeArms (model, q.at(k));
                break;
        case 2: moveRight (model, q.at(k));
                break;
        case 3: moveLeft  (model, q.at(k));
                moveLeft  (model, q.at(k));
                break;
        case 4: moveRight (model, q.at(k));
                break;
        case 5: openArms  (model, q.at(k));
                break;
        case 0:
        default: moveCenter(model, q.at(k));
      }
    }

    kp   = 20.0;
    kd = 13.0;
    w   = 1.0;

    posture = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, kp, kd, w, q.at(0));
    taskManagers["tmFull"] = posture;
    mode = 0;
    count = 0;
  }

  void StageTestTasks::doUpdate(double time, wocra::wOcraModel& state, void** args)
  {
    // posture iteration, assures the sequence of NB_POSTURES postures at the vector q in a period of PERIOD seconds
    time -= PERIOD*floor(time/PERIOD); // normalized time

    if(time >= (mode + 1)*PERIOD/NB_POSTURES + 0.1  || (time < PERIOD/NB_POSTURES && mode == NB_POSTURES - 1))
    {
      posture->setPosture(q.at(mode));
      mode = (mode + 1)%NB_POSTURES;
    }
  }

  void StageTestTasks::raiseArm(char s, wocra::wOcraModel &model, VectorXd &q)
  {
    std::string side(&s, 1);
    q[model.getDofIndex(side + "_shoulder_pitch")] = -120.0*DEG_TO_RAD;
    q[model.getDofIndex(side + "_shoulder_roll")]  =  90.0*DEG_TO_RAD;
    q[model.getDofIndex(side + "_shoulder_yaw")]   =  90.0*DEG_TO_RAD;
    q[model.getDofIndex(side + "_elbow")]          =  120.0*DEG_TO_RAD;
  }

  void StageTestTasks::openArms(wocra::wOcraModel &model, VectorXd &q)
  {
    q[model.getDofIndex("l_shoulder_roll")]  += 30.0*DEG_TO_RAD;
    q[model.getDofIndex("l_elbow")]          += 15.0*DEG_TO_RAD;

    q[model.getDofIndex("r_shoulder_roll")]  += 30.0*DEG_TO_RAD;
    q[model.getDofIndex("r_elbow")]          += 15.0*DEG_TO_RAD;
  }

  void StageTestTasks::closeArms(wocra::wOcraModel &model, VectorXd &q)
  {
    q[model.getDofIndex("l_shoulder_roll")]  -= 30.0*DEG_TO_RAD;
    q[model.getDofIndex("l_elbow")]          -= 15.0*DEG_TO_RAD;

    q[model.getDofIndex("r_shoulder_roll")]  -= 30.0*DEG_TO_RAD;
    q[model.getDofIndex("r_elbow")]          -= 15.0*DEG_TO_RAD;
  }

  void StageTestTasks::moveRight(wocra::wOcraModel &model, VectorXd &q)
  {
    q[model.getDofIndex("l_shoulder_roll")] += 30.0*DEG_TO_RAD;
    q[model.getDofIndex("r_shoulder_yaw")]  += 30.0*DEG_TO_RAD;

    q[model.getDofIndex("r_shoulder_roll")] -= 30.0*DEG_TO_RAD;
    q[model.getDofIndex("r_shoulder_yaw")]  -= 30.0*DEG_TO_RAD;
  }

  void StageTestTasks::moveLeft(wocra::wOcraModel &model, VectorXd &q)
  {
    q[model.getDofIndex("l_shoulder_roll")] -= 30.0*DEG_TO_RAD;
    q[model.getDofIndex("r_shoulder_yaw")]  -= 30.0*DEG_TO_RAD;

    q[model.getDofIndex("r_shoulder_roll")] += 30.0*DEG_TO_RAD;
    q[model.getDofIndex("r_shoulder_yaw")]  += 30.0*DEG_TO_RAD;
  }

  void StageTestTasks::moveCenter(wocra::wOcraModel &model, VectorXd &q)
  {
    q[model.getDofIndex("l_shoulder_roll")]  = +20.0*DEG_TO_RAD;
    q[model.getDofIndex("l_shoulder_pitch")] = -60.0*DEG_TO_RAD;
    q[model.getDofIndex("l_shoulder_yaw")]   = +20.0*DEG_TO_RAD;
    q[model.getDofIndex("l_elbow")]          = 40.0*DEG_TO_RAD;

    q[model.getDofIndex("r_shoulder_roll")]  = +20.0*DEG_TO_RAD;
    q[model.getDofIndex("r_shoulder_pitch")] = -60.0*DEG_TO_RAD;
    q[model.getDofIndex("r_shoulder_yaw")]   = +20.0*DEG_TO_RAD;
    q[model.getDofIndex("r_elbow")]          = 40.0*DEG_TO_RAD;
  }

// }
