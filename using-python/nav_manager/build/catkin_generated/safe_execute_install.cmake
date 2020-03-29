execute_process(COMMAND "/home/djjin/Work/test_ws/src/nav_manager/build/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/djjin/Work/test_ws/src/nav_manager/build/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
