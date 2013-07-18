#include <gtest/gtest.h>

#include <fccl_controllers/features.h>
#include <kdl/frames.hpp>
#include <string>

TEST(FeatureTests, PointFeature)
{
  PointFeature point_feature1;

  std::string empty_string("");
  EXPECT_STREQ(empty_string.c_str(), point_feature1.name_.c_str());
  EXPECT_STREQ(empty_string.c_str(), point_feature1.frame_id_.c_str());

  PointFeature point_feature2("spatula_front", KDL::Vector(0.0, 0.0, 0.1), "/spatula");
  EXPECT_STREQ("spatula_front", point_feature2.name_.c_str());
  EXPECT_STREQ("/spatula", point_feature2.frame_id_.c_str());

  EXPECT_EQ(0.0, point_feature2.position_.x());
  EXPECT_EQ(0.0, point_feature2.position_.y());
  EXPECT_EQ(0.1, point_feature2.position_.z());
}
