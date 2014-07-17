# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_dynamics ${srcs})
target_link_libraries(dart_dynamics
  dart_common
  dart_math
  ${DART_CORE_DEPENDENCIES}
)

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/dynamics
  COMPONENT headers
)
install(TARGETS dart_dynamics EXPORT DARTCoreTargets DESTINATION lib)

