cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  librobox
  VERSION 0.1
  RELEASE alpha
  SUMMARY "C-based driver library for the RoboX platform."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/librobox"
  LICENSE "GNU Lesser General Public License (LGPL)"
  INSTALL /usr
  PREFIX robox-
)

remake_doc(
  html
)

remake_pack_deb(
  DEPENDS tulibs
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  DEPENDS librobox tulibs-dev
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS remake tulibs-dev doxygen
  PASS CMAKE_BUILD_TYPE LIBROBOX_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS remake tulibs-dev doxygen
  PASS CMAKE_BUILD_TYPE LIBROBOX_GIT_REVISION
)
