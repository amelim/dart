# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_utils_sdf ${srcs})
target_link_libraries(dart_utils_sdf ${DART_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/utils/sdf
  COMPONENT headers
)
install(TARGETS dart_utils_sdf EXPORT DARTTargets DESTINATION lib)

