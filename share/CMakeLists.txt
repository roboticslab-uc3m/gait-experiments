yarp_install(FILES applications/gaitReal.xml
                   applications/gaitSim.xml
                   applications/ymanager.ini
             DESTINATION ${ROBOTICSLAB-GAIT-EXPERIMENTS_APPLICATIONS_INSTALL_DIR})

if(ENABLE_oneFootStand)
      yarp_install(DIRECTORY contexts/oneFootStand
                   DESTINATION ${ROBOTICSLAB-GAIT-EXPERIMENTS_CONTEXTS_INSTALL_DIR})
endif()

if(ENABLE_zmpViewer)
      yarp_install(DIRECTORY contexts/zmpViewer
                   DESTINATION ${ROBOTICSLAB-GAIT-EXPERIMENTS_CONTEXTS_INSTALL_DIR})
endif()
