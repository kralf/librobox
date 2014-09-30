remake_find_package(tulibs CONFIG)
remake_find_library(m math.h PACKAGE libm)

remake_include(${TULIBS_INCLUDE_DIRS})
remake_add_library(robox PREFIX OFF LINK ${TULIBS_LIBRARIES} ${M_LIBRARY})
remake_add_headers()
remake_pkg_config_generate(REQUIRES tulibs)
