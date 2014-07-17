# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_lcpsolver ${hdrs} ${srcs})
target_link_libraries(dart_lcpsolver ${DART_CORE_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/lcpsolver
  COMPONENT headers
)
install(TARGETS dart_lcpsolver EXPORT DARTCoreTargets DESTINATION lib)

