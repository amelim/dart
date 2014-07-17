# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Add subdirectories
add_subdirectory(sdf)
add_subdirectory(urdf)

# Library
add_library(dart_utils ${hdrs} ${srcs})
target_link_libraries(dart_utils
  dart_collision
  dart_utils_sdf
  dart_utils_urdf
  ${DART_DEPENDENCIES}
)

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/utils
  COMPONENT headers
)
install(TARGETS dart_utils EXPORT DARTTargets DESTINATION lib)

