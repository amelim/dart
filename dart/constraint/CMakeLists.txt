# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_constraint ${srcs})
target_link_libraries(dart_constraint
  dart_lcpsolver
  dart_collision
  ${DART_CORE_DEPENDENCIES}
)

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/constraint
  COMPONENT headers
)
install(TARGETS dart_constraint EXPORT DARTCoreTargets DESTINATION lib)

