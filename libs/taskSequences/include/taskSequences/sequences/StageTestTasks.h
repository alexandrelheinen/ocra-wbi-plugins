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
        private:
            void positionInit();
            wocra::wOcraSegCartesianTaskManager *leftHand, *rightHand; // posture managers
            std::vector<Eigen::Vector3d> rightHandPos, leftHandPos;   // postures
            int mode;                                                 // posture counter
            double kp;                                                // PD parameters values
            double kd;
            double w;
    };

// }


#endif
