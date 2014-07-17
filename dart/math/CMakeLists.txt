# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_math ${srcs})
target_link_libraries(dart_math ${DART_CORE_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/math
  COMPONENT headers
)
install(TARGETS dart_math EXPORT DARTCoreTargets DESTINATION lib)

