if(NOT TARGET ROBOTICSLAB::KdlVectorConverterLib AND (NOT DEFINED ENABLE_squatAndBalance OR ENABLE_squatAndBalance))
    message(WARNING "KdlVectorConverterLib not found, disabling squatAndBalance")
endif()

cmake_dependent_option(ENABLE_squatAndBalance "Enable/disable squatAndBalance program" ON
                       TARGET ROBOTICSLAB::KdlVectorConverterLib OFF)

if(ENABLE_squatAndBalance)

    add_executable(squatAndBalance main.cpp)

    target_link_libraries(squatAndBalance YARP::YARP_os
                                          YARP::YARP_init
                                          YARP::YARP_dev
                                          ${orocos_kdl_LIBRARIES}
                                          ROBOTICSLAB::KdlVectorConverterLib
                                          ROBOTICSLAB::KinematicsDynamicsInterfaces)

    target_include_directories(squatAndBalance PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    install(TARGETS squatAndBalance)

else()

    set(ENABLE_squatAndBalance OFF CACHE BOOL "Enable/disable squatAndBalance program" FORCE)

endif()
