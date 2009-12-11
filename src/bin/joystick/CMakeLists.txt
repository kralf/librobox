remake_find_library(carmen-joyctrl joyctrl.h PACKAGE carmen)

if(WITH_ERA)
  remake_find_library(carmen-era_interface era_interface.h PACKAGE carmen-era)
endif(WITH_ERA)

remake_add_executables(LINK ${CARMEN_PARAM_INTERFACE_LIBRARY}
  ${CARMEN_JOYCTRL_LIBRARY})
