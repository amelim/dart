# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_common ${hdrs} ${srcs})
target_link_libraries(dart_common ${DART_CORE_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/common
  COMPONENT headers
)
install(TARGETS dart_common EXPORT DARTCoreTargets DESTINATION lib)

