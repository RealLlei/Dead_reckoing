set(APP_NAME planning)
add_executable(
  ${APP_NAME} 
  trigger/adc_trigger_manager.cc
  trigger/warning_fault_process.cc
  trigger/metric_collect.cc
  localview/middleware_local_view.h
  middleware/mdc/switch_manager.cc
  middleware/common/zmq_debug.cc
  middleware/common/pre_process.h
  middleware/common/cpu_recorder.h
  middleware/mdc/planning_mdc.cpp
  middleware/mdc/main_mdc.cpp
  
)
set_target_properties(${APP_NAME} PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
target_link_libraries(${APP_NAME} ${PROJECT_NAME}
  ${MDC_LINK_LIBS} hz_cangen )
install(
  TARGETS ${APP_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION runtime_service/${APP_NAME}/bin
  INCLUDES
  DESTINATION .)

install(
  FILES run_planning_mdc.sh copy2app_mdc.sh
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  DESTINATION scripts/test)

install(DIRECTORY ${CM_CONF}/planning/planningProcess
  DESTINATION runtime_service/${APP_NAME}/etc)

install(FILES ${CM_CONF}/planning/MANIFEST.json
  DESTINATION runtime_service/${APP_NAME}/etc)

install(
FILES start_all.sh start_all_for_dr.sh kill_all.sh record_bag.sh
PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
DESTINATION scripts/test)

install(FILES conf/mdc/planning_config.yaml DESTINATION runtime_service/${APP_NAME}/conf)