#ifndef ICUBCARTESIANTASKGENERATOR_H
#define ICUBCARTESIANTASKGENERATOR_H

#include "orc/control/Feature.h"
#include "orc/control/FullState.h"
#include "orc/control/ControlFrame.h"
#include "orc/control/ControlEnum.h"
#include "orcisir/Tasks/ISIRTask.h"
#include "orcisir/Features/ISIRFeature.h"
#include <Eigen/Dense>
#include "orc/control/Model.h"

#include "ISIRCtrlTaskManager.h"



class iCubCartesianTaskGenerator
{
public:

//===========================Constructor/Destructor===========================//
    iCubCartesianTaskGenerator(ISIRCtrlTaskManager& tManager,const std::string& taskName, const std::string& segmentName,orc::ECartesianDof axes, Eigen::Displacementd target, double stiffness, double damping, double weight);
    
    ~iCubCartesianTaskGenerator();

    const std::string getTaskName();
    orc::PositionFeature* getFeature();
    orc::PositionFeature* getFeatureDes();
    orcisir::ISIRTask* getTask();

private:
    orcisir::ISIRTask*          task;
    orc::SegmentFrame*          SF;
    orc::TargetFrame*           TF;
    orc::PositionFeature*       feat;
    orc::PositionFeature*       featDes;
    std::string                 segmentName;
    Eigen::Displacementd        posdes;
    Eigen::Twistd               veldes;
    const Model&                      model;
    ISIRCtrlTaskManager&            taskManager;

};


class iCubPostureTaskGenerator
{
public:

//===========================Constructor/Destructor===========================//
    iCubPostureTaskGenerator(ISIRCtrlTaskManager& tManager,
								const std::string& taskName,
								int state, 
								Eigen::VectorXd& target, 
								double stiffness, 
								double damping, 
								double weight);
    
    // Overloaded constructor for partial state posture tasks (torso stabilization for instance)
    iCubPostureTaskGenerator(ISIRCtrlTaskManager& tManager,
								const std::string& taskName,
								Eigen::VectorXi& selected_dof_indices,
								int state, 
								Eigen::VectorXd& target, 
								double stiffness, 
								double damping, 
								double weight);
    
    
    ~iCubPostureTaskGenerator();

    const std::string getTaskName();
    orc::FullStateFeature* getFeature();
    orc::FullStateFeature* getFeatureDes();
    orcisir::ISIRTask* getTask();

private:
    orcisir::ISIRTask*          task;
    orc::FullModelState*        FMS;
    orc::FullTargetState*       FTS;
    orc::FullStateFeature*      feat;
    orc::FullStateFeature*      featDes;
    
    orcisir::PartialModelState*   PMS;
    orcisir::PartialTargetState*  PTS;
    orcisir::PartialStateFeature* p_feat;
    orcisir::PartialStateFeature* p_featDes;
    
    Eigen::VectorXd             posdes;
    Eigen::Twistd               veldes;
    const Model&                     model;
    ISIRCtrlTaskManager&            taskManager;

};
#endif // ICUBCARTESIANTASKGENERATOR_H