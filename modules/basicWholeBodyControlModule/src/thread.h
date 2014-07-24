/*
* Copyright (C) 2014 ...
* Author: ...
* email: ...
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

#ifndef BASICWHOLEBODYINTERFACE_THREAD
#define BASICWHOLEBODYINTERFACE_THREAD

#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>


#include <wbi/wbi.h>

#include "orcWbiModel.h"

using namespace yarp::os;
using namespace yarp::sig;


using namespace std;

using namespace wbi;


namespace basicWholeBodyControlNamespace
{

class basicWholeBodyControlThread: public RateThread
{
    string name;
    string robotName;
    wholeBodyInterface *robot;
    orcWbiModel *orcModel;
    yarp::os::Property options;

    // Member variables
    double printPeriod;
    double printCountdown;  // every time this is 0 (i.e. every printPeriod ms) print stuff
    yarp::sig::Vector fb_qRad; // vector that contains the encoders read from the robot
    yarp::sig::Vector fb_torque; // vector that contains the torque read from the robot

public:
    basicWholeBodyControlThread(string _name, string _robotName, int _period, wholeBodyInterface *_wbi, yarp::os::Property & _options);

    bool threadInit();
    void run();
    void threadRelease();

    /** Start the controller. */
    void startController();

};

} // end namespace

#endif