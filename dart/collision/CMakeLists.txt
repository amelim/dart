# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Add subdirectories
add_subdirectory(dart)
add_subdirectory(fcl)
add_subdirectory(fcl_mesh)
if(HAVE_BULLET_COLLISION)
  add_subdirectory(bullet)
endif()

# Library
add_library(dart_collision ${hdrs} ${srcs})
target_link_libraries(dart_collision
  dart_collision_dart
  dart_collision_fcl
  dart_collision_fcl_mesh
  ${DART_CORE_DEPENDENCIES}
)
if(HAVE_BULLET_COLLISION)
  target_link_libraries(dart_collision dart_collision_bullet)
endif()

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/collision
  COMPONENT headers
)
install(TARGETS dart_collision EXPORT DARTCoreTargets DESTINATION lib)

