#include <taskSequences/sequences/StageTestTasks.h>
// StageTestTasks
#include <ocraWbiPlugins/ocraWbiModel.h>

// namespace sequence{
  void StageTestTasks::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
  {
    for (unsigned int i = 0; i < NB_POSTURES; ++i)                  // NB_POSTURES elements
      q.push_back(Eigen::VectorXd::Zero(model.nbInternalDofs()));

    extendArm ('l', model, q.at(0));
    extendArm ('r', model, q.at(0));
    turnArm   ('l', model, q.at(1));
    turnArm   ('r', model, q.at(1));
    bendArm   ('l', model, q.at(2));
    bendArm   ('r', model, q.at(2));
    raiseArm  ('l', model, q.at(3));
    raiseArm  ('r', model, q.at(3));
    downArm   ('l', model, q.at(4));
    downArm   ('r', model, q.at(4));

    p   = 20.0;
    t_d = 13.0;
    w   = 1.0;

    posture = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, p, t_d, w, q.at(0));
    taskManagers["tmFull"] = posture;
    mode = 0;
    count = 0;
  }

  // set functions
  // void setStiffness(double stiffness); = kp
  // void setDamping(double damping);     = td
  // void setWeight(double weight);
  void StageTestTasks::doUpdate(double time, wocra::wOcraModel& state, void** args)
  {
    // parameters iteration
    double t          = time - MAX*TIME_STEP*floor(time/(MAX*TIME_STEP));
    if(t > TIME_STEP*(count+1) || (t < TIME_STEP && count == MAX - 1 ))
    {
      count     = (count + 1)%MAX;
      p         =     VALUE_STEP*count;
      t_d       = 0.2*VALUE_STEP*count;
      posture->setStiffness(p);
      posture->setDamping(t_d);
      posture->setWeight(w);

      std::cout << "Stiffness   = " << p    << ";  \n"
                << "Damping     = " << t_d  << ";  \n"
                << "Task weight = " << w    << ";  \n"
                << "  at "          << time << "s; \n"
                << "...............................\n";
    }

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

  void StageTestTasks::downArm(char s, wocra::wOcraModel &model, VectorXd &q)
  {
    std::string side(&s, 1);
    q[model.getDofIndex(side + "_shoulder_roll")]  = 0.0*DEG_TO_RAD;
    q[model.getDofIndex(side + "_shoulder_pitch")] = 0.0*DEG_TO_RAD;
    q[model.getDofIndex(side + "_shoulder_yaw")]   = 0.0*DEG_TO_RAD;
    q[model.getDofIndex(side + "_elbow")]          = M_PI / 8;
  }

  void StageTestTasks::extendArm(char s, wocra::wOcraModel &model, VectorXd &q)
  {
    std::string side(&s, 1);
    q[model.getDofIndex(side + "_shoulder_pitch")] =  -90.0*DEG_TO_RAD;
    q[model.getDofIndex(side + "_shoulder_roll")]  =  20.0*DEG_TO_RAD;
    q[model.getDofIndex(side + "_shoulder_yaw")]   =  90.0*DEG_TO_RAD;
    q[model.getDofIndex(side + "_elbow")]          =  2.0*DEG_TO_RAD;
  }

  void StageTestTasks::turnArm(char s, wocra::wOcraModel &model, VectorXd &q)
  {
    std::string side(&s, 1);
    extendArm(s, model, q);
    q[model.getDofIndex(side + "_shoulder_yaw")] = -90.0*DEG_TO_RAD;
  }

  void StageTestTasks::bendArm(char s, wocra::wOcraModel &model, VectorXd &q)
  {
    std::string side(&s, 1);
    extendArm(s, model, q);
    turnArm(s, model, q);
    q[model.getDofIndex(side + "_elbow")] = -120.0*DEG_TO_RAD;
  }

// }
