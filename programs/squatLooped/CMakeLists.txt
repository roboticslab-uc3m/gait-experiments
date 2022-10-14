add_executable(squatLooped main.cpp)

target_link_libraries(squatLooped YARP::YARP_os
                                  YARP::YARP_init
                                  YARP::YARP_dev
                                  ROBOTICSLAB::KdlVectorConverterLib
                                  ROBOTICSLAB::KinematicsDynamicsInterfaces)

install(TARGETS squatLooped)
