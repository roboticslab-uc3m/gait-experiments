option(ENABLE_zmpViewer "Enable/disable zmpViewer program" ON)

if(ENABLE_zmpViewer)

    add_executable(zmpViewer main.cpp
                             ZmpViewer.cpp
                             ZmpViewer.hpp)

    target_link_libraries(zmpViewer YARP::YARP_os
                                    YARP::YARP_init
                                    YARP::YARP_sig)

    install(TARGETS zmpViewer)

endif()
