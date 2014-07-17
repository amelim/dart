# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_collision_dart ${hdrs} ${srcs})
target_link_libraries(dart_collision_dart ${DART_CORE_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/collision/dart
  COMPONENT headers
)
install(TARGETS dart_collision_dart EXPORT DARTCoreTargets DESTINATION lib)
