add_executable(squatAndBalance main.cpp)

target_link_libraries(squatAndBalance YARP::YARP_OS
                                      YARP::YARP_init
                                      YARP::YARP_dev
                                      ROBOTICSLAB::TrajectoryLib
                                      ROBOTICSLAB::KinematicsDynamicsInterfaces)

include(GNUInstallDirs)

install(TARGETS squatAndBalance
        DESTINATION ${CMAKE_INSTALL_BINDIR})
