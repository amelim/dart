# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Add subdirectories
if(HAVE_IPOPT)
  add_subdirectory(ipopt)
endif()
if(HAVE_NLOPT)
  add_subdirectory(nlopt)
endif()
if(HAVE_SNOPT)
  add_subdirectory(snopt)
endif()

# Library
add_library(dart_optimizer ${hdrs} ${srcs})
target_link_libraries(dart_optimizer
  dart_common
  ${DART_DEPENDENCIES})
if(HAVE_IPOPT)
  target_link_libraries(dart_optimizer dart_optimizer_ipopt)
endif()
if(HAVE_NLOPT)
  target_link_libraries(dart_optimizer dart_optimizer_nlopt)
endif()
if(HAVE_SNOPT)
  target_link_libraries(dart_optimizer dart_optimizer_snopt)
endif()

# Install
install(FILES ${hdrs}
  DESTINATION include/dart/optimizer
  COMPONENT headers
)
install(TARGETS dart_optimizer EXPORT DARTTargets DESTINATION lib)

