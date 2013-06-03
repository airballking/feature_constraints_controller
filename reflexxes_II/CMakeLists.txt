cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/ReflexxesAPI.cpp
                                     src/TypeIIRMLPolynomial.cpp src/TypeIIRMLStep2Profiles.cpp
                                     src/TypeIIRMLPosition.cpp src/TypeIIRMLStep2WithoutSynchronization.cpp
                                     src/TypeIIRMLCalculatePositionalExtrems.cpp src/TypeIIRMLQuicksort.cpp                     
                                     src/TypeIIRMLStep3.cpp src/TypeIIRMLDecisions.cpp                       
                                     src/TypeIIRMLSetupModifiedSelectionVector.cpp src/TypeIIRMLVelocityCalculatePositionalExtrems.cpp
                                     src/TypeIIRMLDecisionTree1A.cpp src/TypeIIRMLStep1.cpp                         
                                     src/TypeIIRMLVelocity.cpp src/TypeIIRMLDecisionTree1B.cpp                  
                                     src/TypeIIRMLStep1IntermediateProfiles.cpp src/TypeIIRMLVelocityFallBackStrategy.cpp
                                     src/TypeIIRMLDecisionTree1C.cpp src/TypeIIRMLStep1Profiles.cpp                 
                                     src/TypeIIRMLVelocityIsPhaseSynchronizationPossible.cpp src/TypeIIRMLDecisionTree2.cpp                   
                                     src/TypeIIRMLStep2.cpp src/TypeIIRMLVelocityMethods.cpp
                                     src/TypeIIRMLFallBackStrategy.cpp src/TypeIIRMLStep2IntermediateProfiles.cpp     
                                     src/TypeIIRMLVelocitySetupPhaseSyncSelectionVector.cpp src/TypeIIRMLIsPhaseSynchronizationPossible.cpp  
                                     src/TypeIIRMLStep2PhaseSynchronization.cpp)
