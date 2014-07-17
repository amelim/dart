# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_planning ${hdrs} ${srcs})
target_link_libraries(dart_planning ${DART_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/planning
  COMPONENT headers
)
install(TARGETS dart_planning EXPORT DARTTargets DESTINATION lib)

