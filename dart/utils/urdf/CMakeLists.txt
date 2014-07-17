# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_utils_urdf ${hdrs} ${srcs})
target_link_libraries(dart_utils_urdf ${DART_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/utils/urdf
  COMPONENT headers
)
install(TARGETS dart_utils_urdf EXPORT DARTTargets DESTINATION lib)

