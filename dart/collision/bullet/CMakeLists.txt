# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_collision_bullet ${srcs})
target_link_libraries(dart_collision_bullet ${DART_CORE_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/collision/bullet
  COMPONENT headers
)
install(TARGETS dart_collision_bullet EXPORT DARTCoreTargets DESTINATION lib)
