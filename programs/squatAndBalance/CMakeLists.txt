add_executable(squatAndBalance main.cpp)

target_link_libraries(squatAndBalance YARP::YARP_os
                                      YARP::YARP_init
                                      YARP::YARP_dev
                                      orocos-kdl
                                      ROBOTICSLAB::KdlVectorConverterLib
                                      ROBOTICSLAB::KinematicsDynamicsInterfaces)

include(GNUInstallDirs)

install(TARGETS squatAndBalance
        DESTINATION ${CMAKE_INSTALL_BINDIR})
