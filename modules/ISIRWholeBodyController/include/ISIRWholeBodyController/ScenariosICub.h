#ifndef SCENARIOSICUB_H
#define SCENARIOSICUB_H

#include "wocra/Tasks/wOcraTaskManagerCollectionBase.h"
#include "wocra/Trajectory/wOcraTrajectory.h"

class ScenarioICub_01_Standing: public wocra::wOcraTaskManagerCollectionBase
{
    public:
        ScenarioICub_01_Standing();
        virtual ~ScenarioICub_01_Standing();
    protected: 
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args); 
/*
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
        // Partial torso posture task
        wocra::wOcraPartialPostureTaskManager*         tmPartialBack;
        // Left foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactLeft;
        // Right foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactRight;
        // Waist task
        wocra::wOcraSegPoseTaskManager*                tmSegPoseWaist;
*/
};


class ScenarioICub_02_VariableWeightHandTasks: public wocra::wOcraTaskManagerCollectionBase
{
    public:
        ScenarioICub_02_VariableWeightHandTasks();
        virtual ~ScenarioICub_02_VariableWeightHandTasks();
    protected: 
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args); 
        wocra::wOcraTrajectory*                        leftHandTrajectory;
        int lHandIndex;

    private:
        wocra::wOcraVariableWeightsTaskManager*           tmLeftHand;
/*
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
        // Partial torso posture task
        wocra::wOcraPartialPostureTaskManager*         tmPartialBack;
        // Left foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactLeft;
        // Right foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactRight;
        // Waist task
        wocra::wOcraSegPoseTaskManager*                tmSegPoseWaist;
*/
};



#endif // TASKSETROMEOBALANCE_H