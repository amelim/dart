# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
add_library(dart_optimizer_snopt ${hdrs} ${srcs})
target_link_libraries(dart_optimizer_snopt ${DART_DEPENDENCIES})

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/optimizer/snopt
  COMPONENT headers
)
install(TARGETS dart_optimizer_snopt EXPORT DARTTargets DESTINATION lib)

