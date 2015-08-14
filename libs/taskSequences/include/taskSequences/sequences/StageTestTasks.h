#ifndef STAGETESTTASKS_H
#define STAGETESTTASKS_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"
#include <math.h>
#include <vector>

#define NB_POSTURES 6
#define PERIOD      30.0

// namespace sequence {

    class StageTestTasks : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
            // tasks postures
            void closeArms (wocra::wOcraModel &model, VectorXd &q);
            void openArms  (wocra::wOcraModel &model, VectorXd &q);
            void moveRight (wocra::wOcraModel &model, VectorXd &q);
            void moveLeft  (wocra::wOcraModel &model, VectorXd &q);
            void moveCenter(wocra::wOcraModel &model, VectorXd &q);
            void raiseArm  (char side, wocra::wOcraModel &model, VectorXd &q);
        private:
            wocra::wOcraFullPostureTaskManager *posture;
            // Full posture task
            int mode; // up down
            std::vector<Eigen::VectorXd> q;
            // PD parameters values
            double kp;
            double kd;
            double w;
            int count;
    };

// }


#endif
