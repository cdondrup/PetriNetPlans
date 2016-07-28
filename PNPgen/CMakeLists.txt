cmake_minimum_required(VERSION 2.6)
project (PNPgen) 

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#search for boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(LibXML++ REQUIRED)
include_directories(${LibXML++_INCLUDE_DIRS})

find_package(Libpcrecpp REQUIRED)
include_directories(${PCRE_INCLUDE_DIRS})


#include directory with the headers of the source files
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

include_directories(${PROJECT_SOURCE_DIR}/include/PNPgen)
include_directories(${PROJECT_SOURCE_DIR}/include/PRUMDP)


#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

ADD_LIBRARY(pnpgen SHARED src/pnpgenerator.cpp)

ADD_LIBRARY(prumdp SHARED src/PRUMDP/MDP.cpp src/PRUMDP/MDPsolve.cpp src/PRUMDP/PRUplus.cpp src/PRUMDP/PRUprint.cpp
    src/PRUMDP/PRU2MDP.cpp src/PRUMDP/PRU2MDPprogress.cpp src/PRUMDP/PRU2MDPstateStore.cpp src/PRUMDP/PRUconstraint.cpp
)

add_executable(pnpgen_linear src/pnpgen_linear.cpp)
add_executable(pnpgen_policy src/pnpgen_policy.cpp)
add_executable(pnpgen_condplan src/pnpgen_condplan.cpp)
add_executable(pnpgen_pru src/pnpgen_pru.cpp)

target_link_libraries(pnpgen boost_regex)
target_link_libraries(prumdp pnpgen boost_regex ${LibXML++_LIBRARIES} ${PCRE_LIBRARIES})
target_link_libraries(pnpgen_linear pnpgen)
target_link_libraries(pnpgen_policy pnpgen)
target_link_libraries(pnpgen_condplan pnpgen)
target_link_libraries(pnpgen_pru prumdp)
 
set(INCLUDE_INSTALL_DIR "${PROJECT_SOURCE_DIR}/include/")
set(LIB_INSTALL_DIR "lib/")
include(CMakePackageConfigHelpers)
export(TARGETS pnpgen prumdp pnpgen_linear pnpgen_policy pnpgen_condplan pnpgen_pru
  FILE "${PROJECT_BINARY_DIR}/PNPgenTargets.cmake")
export(PACKAGE PNPgen)
configure_package_config_file(cmake/PNPgenConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/PNPgenConfig.cmake
                              INSTALL_DESTINATION ${LIB_INSTALL_DIR}/share/${PROJECT_NAME}/cmake
                              PATH_VARS INCLUDE_INSTALL_DIR)

#file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
#   "${INSTALL_INCLUDE_DIR}")
#set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include" "${PROJECT_BINARY_DIR}")
#configure_file(cmake/PNPgenConfig.cmake.in
#  "${PROJECT_BINARY_DIR}/PNPgenConfig.cmake" @ONLY)
#set(CONF_INCLUDE_DIRS "\${PNPGEN_CMAKE_DIR}/include")
#configure_file(cmake/PNPgenConfig.cmake.in
#  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/PNPgenConfig.cmake" @ONLY)

install(TARGETS pnpgen prumdp pnpgen_linear pnpgen_policy pnpgen_pru
  EXPORT PNPgenTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/PNPgenConfig.cmake 
  DESTINATION share/${PROJECT_NAME}/cmake COMPONENT dev)
 
install(EXPORT PNPgenTargets DESTINATION
  share/${PROJECT_NAME}/cmake COMPONENT dev)
