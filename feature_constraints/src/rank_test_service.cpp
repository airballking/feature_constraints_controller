#include <feature_constraints/FeatureConstraints.h>
#include <feature_constraints/Conversions.h>
#include <feature_constraints/Analysis.h>

#include <constraint_msgs/ConstraintsRank.h>

#include <ros/ros.h>

bool service_callback(constraint_msgs::ConstraintsRank::Request& request, constraint_msgs::ConstraintsRank::Response&  answer)
{
  Analysis analyzer(request.constraints.size());
  answer.rank = analyzer.rank(fromMsg(request.constraints), 0.01, 1e-6);
  return true;
}


int main(int argc, char* argv[])
{
  Constraint::init();

  ros::init(argc, argv, "constraint_rank_test");
  ros::NodeHandle n("~");

  ros::ServiceServer service = n.advertiseService("/constraints_rank",
                                                  service_callback);

  ros::spin();

  return 0;
}

