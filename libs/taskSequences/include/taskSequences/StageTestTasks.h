#ifndef STAGETESTTASKS_H
#define STAGETESTTASKS_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "../sequenceTools.h"
#include <math.h>
#include <vector>

#define NB_POSTURES 5
#define PERIOD      10.0
#define TIME_STEP   20.0
#define MAX         10
#define VALUE_STEP  5.0

// namespace sequence {

    class StageTestTasks : public wocra::wOcraTaskSequenceBase
    {
        protected:
            virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
            virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
            // tasks postures
            void raiseArm (char side, wocra::wOcraModel &model, VectorXd &q);
            void downArm  (char side, wocra::wOcraModel &model, VectorXd &q);
            void extendArm(char side, wocra::wOcraModel &model, VectorXd &q);
            void turnArm  (char side, wocra::wOcraModel &model, VectorXd &q);
            void bendArm  (char side, wocra::wOcraModel &model, VectorXd &q);
        private:
            wocra::wOcraFullPostureTaskManager *posture;
            // Full posture task
            int mode; // up down
            std::vector<Eigen::VectorXd> q;
            // PD parameters values
            double p;
            double t_d;
            double w;
            int count;
    };

// }


#endif
