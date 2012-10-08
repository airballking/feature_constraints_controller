
#include <feature_constraints/FeatureConstraints.h>

KDL::JntArray cyl_irpy_chain(const KDL::Frame& frame);

double chain0(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature);
double chain1(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature);
double chain2(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature);
double chain3(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature);
double chain4(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature);
double chain5(const KDL::Frame& frame, const Feature& tool_feature, const Feature& object_feature);

void chain_rpy_init();
