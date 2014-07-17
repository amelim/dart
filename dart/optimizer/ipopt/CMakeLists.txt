# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_optimizer_ipopt ${srcs})
target_link_libraries(dart_optimizer_ipopt ${DART_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/optimizer/ipopt
  COMPONENT headers
)
install(TARGETS dart_optimizer_ipopt EXPORT DARTTargets DESTINATION lib)

