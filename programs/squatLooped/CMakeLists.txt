add_executable(squatLooped main.cpp)

target_link_libraries(squatLooped YARP::YARP_os
                                  YARP::YARP_init
                                  YARP::YARP_dev
                                  ROBOTICSLAB::KdlVectorConverterLib
                                  ROBOTICSLAB::KinematicsDynamicsInterfaces)

include(GNUInstallDirs)

install(TARGETS squatLooped
        DESTINATION ${CMAKE_INSTALL_BINDIR})
