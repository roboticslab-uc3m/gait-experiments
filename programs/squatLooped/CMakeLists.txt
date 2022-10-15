if(NOT TARGET ROBOTICSLAB::KdlVectorConverterLib AND (NOT DEFINED ENABLE_squatLooped OR ENABLE_squatLooped))
    message(WARNING "KdlVectorConverterLib not found, disabling squatLooped")
endif()

cmake_dependent_option(ENABLE_squatLooped "Enable/disable squatLooped program" ON
                       TARGET ROBOTICSLAB::KdlVectorConverterLib OFF)

if(ENABLE_squatLooped)

    add_executable(squatLooped main.cpp)

    target_link_libraries(squatLooped YARP::YARP_os
                                      YARP::YARP_init
                                      YARP::YARP_dev
                                      ROBOTICSLAB::KdlVectorConverterLib
                                      ROBOTICSLAB::KinematicsDynamicsInterfaces)

    install(TARGETS squatLooped)

else()

    set(ENABLE_squatLooped OFF CACHE BOOL "Enable/disable squatLooped program" FORCE)

endif()
