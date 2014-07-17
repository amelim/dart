# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_collision_fcl_mesh ${srcs})
target_link_libraries(dart_collision_fcl_mesh ${DART_CORE_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/collision/fcl_mesh
  COMPONENT headers
)
install(TARGETS dart_collision_fcl_mesh EXPORT DARTCoreTargets DESTINATION lib)
