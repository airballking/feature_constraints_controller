
#include <ros/ros.h>
#include <tf_conversions/tf_kdl.h>

#include "ROSObject.hpp"

#include <ocl/Component.hpp>

ORO_CREATE_COMPONENT( iTaSC::ROSObject )
;

using namespace KDL;
using namespace RTT;
using namespace OCL;
using namespace Eigen;
namespace iTaSC {
ROSObject::ROSObject(const std::string& name) :
	SubRobot(name, PreOperational), external_pose("Pose"), pose()
{
	nq=0;
}

bool ROSObject::configureHook()
{
    this->ports()->addPort("pose", external_pose);

    ConnPolicy c = ConnPolicy::data(ConnPolicy::LOCK_FREE, true, false);
    c.transport = 3;
    c.name_id = string("/object_pose");
    external_pose.createStream(c);

	return true;
}

bool ROSObject::startHook()
{
	return true;
}



void ROSObject::updateHook()
{
    // read pose from ROS
    geometry_msgs::Pose pose_msg;
	if(NewData == external_pose.read(pose_msg))
        tf::PoseMsgToKDL(pose_msg, pose);

    // write pose to iTaSC system
	T_b_e_port.write(pose);

	// zero twist for now, assuming quasistatic environment
	JuXudot_port.write(KDL::Twist::Zero());
}

}
