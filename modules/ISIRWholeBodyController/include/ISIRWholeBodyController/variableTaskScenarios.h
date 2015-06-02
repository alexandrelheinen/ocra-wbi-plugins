#ifndef VARIABLETASKSCENARIOS_H
#define VARIABLETASKSCENARIOS_H

#include "wocra/Tasks/wOcraTaskManagerCollectionBase.h"
#include "wocra/Trajectory/wOcraTrajectory.h"

class 01_Demonstration : public wocra::wOcraTaskManagerCollectionBase
{
    protected:
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args);
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            fullPostureTask;
        // Partial posture task
        wocra::wOcraPartialPostureTaskManager*         torsoPostureTask;
        // Segment left hand task
        wocra::wOcraSegCartesianTaskManager*           leftHandTask;

        wocra::wOcraSegCartesianTaskManager*           rightHandTask;

        // trajectory
        wocra::wOcraTrajectory*                        leftHandTrajectory;


        int lHandIndex, rightHandIndex;

};


#endif // VARIABLETASKSCENARIOS_H
