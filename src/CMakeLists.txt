remake_find_library(tulibs-timer timer.h PACKAGE tulibs)
remake_find_library(tulibs-thread thread.h PACKAGE tulibs)
remake_find_library(tulibs-config config.h PACKAGE tulibs)

remake_add_library(robox PREFIX OFF LINK ${TULIBS_TIMER_LIBRARY}
  ${TULIBS_THREAD_LIBRARY} ${TULIBS_CONFIG_LIBRARY})
remake_add_headers()
