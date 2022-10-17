set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})
configure_file (
  ${PROJECT_SOURCE_DIR}/include/global_defination/global_defination.h.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}/include/global_defination/global_defination.h)
include_directories(${PROJECT_BINARY_DIR}/include)
add_definitions( -DPROJECT_SPACE_PATH=${PROJECT_SOURCE_DIR} )