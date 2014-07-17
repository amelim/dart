# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_integration ${srcs})
target_link_libraries(dart_integration ${DART_CORE_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/integration
  COMPONENT headers
)
install(TARGETS dart_integration EXPORT DARTCoreTargets DESTINATION lib)

