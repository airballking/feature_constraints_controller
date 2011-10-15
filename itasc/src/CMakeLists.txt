
set (CMAKE_CXX_FLAGS "-DEIGEN_USE_NEW_STDVECTOR -DEIGEN2_SUPPORT")

#GLOBAL_ADD_COMPONENT(ublas_toolkit ublas_toolkit.cpp)
orocos_library(eigen_toolkit eigen_toolkit.cpp)
set_target_properties(eigen_toolkit PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/orocos/types)



orocos_component(iTaSC_Scene Scene.cpp )
target_link_libraries(iTaSC_Scene ${RTT_PLUGIN_rtt-marshalling_LIBRARIES})
#component_add_libs(iTaSC_Scene ${KDL_LIBS})

orocos_component(iTaSC_WDLSSolver WDLSVelocitySolver.cpp )
#component_add_libs(iTaSC_WDLSSolver ${KDL_LIBS})

orocos_component(feature_task FeatureTask.cpp)
orocos_component(chain_task ChainTask.cpp)
orocos_component(ros_robot ROSRobot.cpp)
orocos_component(ros_object ROSObject.cpp)
orocos_component(ros_limitavoider ROSLimitAvoider.cpp)

