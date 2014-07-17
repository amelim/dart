# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_simulation ${hdrs} ${srcs})
target_link_libraries(dart_simulation
  dart_common
  dart_math
  dart_integration
  dart_constraint
  dart_dynamics
  ${DART_CORE_DEPENDENCIES}
)

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/simulation
  COMPONENT headers
)
install(TARGETS dart_simulation EXPORT DARTCoreTargets DESTINATION lib)

