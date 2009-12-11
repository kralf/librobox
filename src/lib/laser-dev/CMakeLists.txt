remake_find_package(Threads)
if(NOT ${CMAKE_USE_PTHREADS_INIT})
  message(FATAL_ERROR "Missing POSIX thread support!")
endif(NOT ${CMAKE_USE_PTHREADS_INIT})
remake_find_library(carmen-serial carmenserial.h PACKAGE carmen)
remake_find_library(carmen-laser_interface laser_interface.h PACKAGE carmen)

remake_define(LASER_USE_PTHREAD ON)

remake_add_library(
  tcp862_laser_dev
  LINK tcp862_serial ${CARMEN_SERIAL_LIBRARY} ${CARMEN_LASER_INTERFACE_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
)
remake_add_headers()
