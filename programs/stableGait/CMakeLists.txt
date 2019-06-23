add_executable(stableGait main.cpp
                          LimitChecker.hpp
                          LimitChecker.cpp)

target_link_libraries(stableGait YARP::YARP_OS
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 ROBOTICSLAB::KdlVectorConverterLib
                                 ROBOTICSLAB::KinematicsDynamicsInterfaces
                                 ROBOTICSLAB::ColorDebug)

target_compile_features(stableGait PRIVATE cxx_lambdas)

include(GNUInstallDirs)

install(TARGETS stableGait
        DESTINATION ${CMAKE_INSTALL_BINDIR})
