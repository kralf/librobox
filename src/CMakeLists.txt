remake_find_package(tulibs CONFIG)

remake_add_library(robox PREFIX OFF LINK ${TULIBS_LIBRARIES})
remake_add_headers()
remake_pkg_config_generate(REQUIRES tulibs)
