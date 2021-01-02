add_executable(stableGait main.cpp
                          LimitChecker.hpp
                          LimitChecker.cpp
                          StepGenerator.hpp
                          StepGenerator.cpp
                          GaitSpecs.hpp
                          TrajectoryGenerator.hpp
                          TrajectoryGenerator.cpp
                          TargetBuilder.hpp
                          TargetBuilder.cpp)

target_link_libraries(stableGait YARP::YARP_OS
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 ROBOTICSLAB::KdlVectorConverterLib
                                 ROBOTICSLAB::KinematicsDynamicsInterfaces)

target_compile_features(stableGait PRIVATE cxx_lambdas)

include(GNUInstallDirs)

install(TARGETS stableGait
        DESTINATION ${CMAKE_INSTALL_BINDIR})
