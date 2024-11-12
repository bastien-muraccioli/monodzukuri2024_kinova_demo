set(controller_SRC MonodzukuriKinovaDemo.cpp)

set(controller_HDR MonodzukuriKinovaDemo.h)

set(OpenGL_GUI_SRC ui.cpp)

set(OpenGL_GUI_HDR ui.h)# stb_image.h)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR} ${OpenGL_GUI_SRC} ${OpenGL_GUI_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS
                                                 "-DMonodzukuriKinovaDemo_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm
                                            MinimumJerkTask::mc_tasks_MinimumJerkTask
                                            GL glfw glut GLEW)

            

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)