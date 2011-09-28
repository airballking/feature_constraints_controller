#ifndef ROSOBJECT_HPP_INCLUDED
#define ROSOBJECT_HPP_INCLUDED

#include "SubRobot.hpp"
#include <geometry_msgs/Pose.h>

namespace iTaSC {
class ROSObject: public SubRobot {
public:
	ROSObject(const std::string& name);
	~ROSObject() {}
	;

    virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook() {}
	virtual void cleanupHook() {}

private:
    RTT::InputPort<geometry_msgs::Pose> external_pose;
	KDL::Frame pose;
};
}

#endif // ROSOBJECT_HPP_INCLUDED
