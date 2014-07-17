# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_renderer ${hdrs} ${srcs})
target_link_libraries(dart_renderer ${DART_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/renderer
  COMPONENT headers
)
install(TARGETS dart_renderer EXPORT DARTTargets DESTINATION lib)

