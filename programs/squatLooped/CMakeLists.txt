add_executable(squatLooped main.cpp)

target_link_libraries(squatLooped YARP::YARP_OS
                                  YARP::YARP_init
                                  YARP::YARP_dev
                                  ROBOTICSLAB::KdlVectorConverterLib
                                  ROBOTICSLAB::KinematicsDynamicsInterfaces
                                  ROBOTICSLAB::ColorDebug)

target_compile_features(squatLooped PRIVATE cxx_lambdas)

include(GNUInstallDirs)

install(TARGETS squatLooped
        DESTINATION ${CMAKE_INSTALL_BINDIR})
