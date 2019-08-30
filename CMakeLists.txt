cmake_minimum_required(VERSION 3.5)
project(simple_kite_benchmark)

add_definitions( -DLAPACK -DSPARSE_MATRIX -DUNIX -DHAVE_CSTDDEF -DUSE_IPOPT)

# set include directories for PSOPT
set(PSOPT_DIR /home/listov/psopt_playground/psopt)
set(PSOPT_INCLUDE_DIR ${PSOPT_DIR}/PSOPT/src)
set(DMATRIX_INCLUDE_DIR ${PSOPT_DIR}/dmatrix/include)
set(CXSPARSE_INCLUDE_DIR ${PSOPT_DIR}/SuiteSparse/CXSparse/Include;${PSOPT_DIR}/SuiteSparse/SuiteSparse_config)
set(LUSOL_INCLUDE_DIR ${PSOPT_DIR}/lusol/csrc)
set(ADOLC_INCLUDE_DIR /usr/include/adolc;/usr/include/adolc/sparse;/usr/include/adolc/drivers;/usr/local/adolc/lie;/usr/local/adolc/internal)
set(IPOPT_INCLUDE_DIR /usr/local/include/coin/;/usr/local/include/coin/ThirdParty)

include_directories(${PSOPT_INCLUDE_DIR} ${DMATRIX_INCLUDE_DIR} ${CXSPARSE_INCLUDE_DIR}
                    ${LUSOL_INCLUDE_DIR} ${ADOLC_INCLUDE_DIR} ${IPOPT_INCLUDE_DIR})
# set PSOPT libraries
set(PSOPT_LIBRARIES ${PSOPT_DIR}/PSOPT/lib/libpsopt.a)
set(ADOLC_LIBRARIES /usr/lib/libadolc.so)
set(DMATRIX_LIBRARIES ${PSOPT_DIR}/dmatrix/lib/libdmatrix.a)
set(LUSOL_LIBRARIES ${LUSOL_INCLUDE_DIR}/liblusol.a)
set(CXSPARSE_LIBRARIES ${PSOPT_DIR}/SuiteSparse/CXSparse/Lib/libcxsparse.a)
set(IPOPT_LIBRARIES /usr/local/lib/libipopt.so;/usr/local/lib/libcoinmetis.so;/usr/local/lib/libcoinmumps.so)

add_executable(psopt_node psopt_node.cpp)
target_link_libraries(psopt_node ${PSOPT_LIBRARIES} ${DMATRIX_LIBRARIES} ${ADOLC_LIBRARIES} ${LUSOL_LIBRARIES}
                                  ${CXSPARSE_LIBRARIES}  ${IPOPT_LIBRARIES} dl lapack)

# build ACADO control node
find_package(ACADO REQUIRED)
include_directories(${ACADO_INCLUDE_DIRS})
set(ACADO_LIBRARIES /usr/local/lib/libacado_toolkit_s.so)
add_executable(acado_codegen acado_codegen.cpp)
target_link_libraries(acado_codegen ${ACADO_LIBRARIES})


# ACADO node
set(GENERATED_MPC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/kite_mpc_export)
include_directories(${GENERATED_MPC_INCLUDE_DIR})
add_custom_target(
   acado_exported_rti
   COMMAND make
   WORKING_DIRECTORY ${GENERATED_MPC_INCLUDE_DIR}
)

add_executable(acado_node acado_node.cpp)
target_link_libraries(acado_node ${ACADO_LIBRARIES} ${GENERATED_MPC_INCLUDE_DIR}/libacado_exported_rti.a)

# ROS ACADO node
# add_executable(acado_node2 acado_node2.cpp acado_node2.hpp)
# target_link_libraries(acado_node2 ${ACADO_LIBRARIES} ${GENERATED_MPC_INCLUDE_DIR}/libacado_exported_rti.a ${catkin_LIBRARIES})
# add_dependencies(acado_node2 openkite_generate_messages_cpp)

