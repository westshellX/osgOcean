SET(HEADER_PATH ${osgOcean_SOURCE_DIR}/include/osgSky )
SET(LIB_HEADER_PATH ${osgOcean_SOURCE_DIR}/include )

SET( LIB_HEADERS
  ${HEADER_PATH}/SkyDome
  ${HEADER_PATH}/SphereSegment
  ${HEADER_PATH}/Export
  ${HEADER_PATH}/Version
)

# Adds headers to the "Header Files" group in visual studio
SOURCE_GROUP( "Header Files" ${HEADER_PATH}/*.* )

ADD_DEFINITIONS(-DOSGSKY_LIBRARY)

INCLUDE_DIRECTORIES (
   ${osgOcean_SOURCE_DIR}/include/
   ${osgOcean_SOURCE_DIR}/include/osgSky/
)

ADD_LIBRARY(
  osgSky
  ${LIB_HEADERS}
  SkyDome.cpp
  SphereSegment.cpp
  Version.cpp
)

SET_TARGET_PROPERTIES(
  osgSky
  PROPERTIES FRAMEWORK TRUE
  PUBLIC_HEADER "${LIB_HEADERS}"
)

TARGET_LINK_LIBRARIES(
  osgSky
  osgOcean

  optimized ${OSG_LIBRARY}
  debug ${OSG_LIBRARY_DEBUG}
)

INSTALL(
  TARGETS osgSky
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include/osgSky
)