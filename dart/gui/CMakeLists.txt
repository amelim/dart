# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_gui ${hdrs} ${srcs})
target_link_libraries(dart_gui
  dart_simulation
  ${DART_DEPENDENCIES}
)

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/gui
  COMPONENT headers
)
install(TARGETS dart_gui EXPORT DARTTargets DESTINATION lib)

