set(APP_NAME planning)
add_executable(
  ${APP_NAME} 
  trigger/orin_trigger_manager.cc
  trigger/warning_fault_process.cc
  trigger/metric_collect.cc
  localview/middleware_local_view.h
  middleware/common/zmq_debug.cc
  middleware/orin/orin_switch_manager.cc
  middleware/common/cpu_recorder.h
  middleware/orin/main_orin.cpp
  middleware/orin/planning_orin.cpp
)
set_target_properties(${APP_NAME} PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
target_link_libraries(
  ${APP_NAME} 
  ${PROJECT_NAME}
  neta_dc
  neta_phm
  pthread
  )
if(PLATFORM MATCHES "x86")
  target_link_libraries(
    ${APP_NAME}
    neta_adf
    neta_cfg)
else()
  target_link_libraries(
    ${APP_NAME}
    netaos::adf
    netaos::cfg)
endif(PLATFORM MATCHES "x86") 

if(PLATFORM MATCHES "orin")
  target_link_libraries(
    ${APP_NAME}
    ${MIMALLOC_LIBRARIES}
  )
endif()

install(
  TARGETS ${APP_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION runtime_service/${APP_NAME}/bin
  INCLUDES
  DESTINATION .)

install(
  FILES run_planning_orin.sh copy2app_orin.sh record_mcap.sh
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  DESTINATION scripts/test)

# install(FILES conf/orin/MANIFEST.json
#  DESTINATION runtime_service/${APP_NAME}/etc)

install(FILES conf/orin/planning_config.yaml conf/orin/self_planning_config.yaml 
  DESTINATION runtime_service/${APP_NAME}/conf)
