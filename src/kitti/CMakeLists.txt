

file( GLOB_RECURSE FILE_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/ *.cpp )

message(STATUS "FILE_SRCS:    ${FILE_SRCS}")


set(LIBS ${LIBS} kitti_proto)

add_library(${PROJECT_NAME} SHARED ${FILE_SRCS})
target_link_libraries(${PROJECT_NAME} ${LIBS})
add_library(${PROJECT_NAME}_static STATIC ${FILE_SRCS})
set_target_properties(${PROJECT_NAME} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(${PROJECT_NAME}_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(${PROJECT_NAME}_static PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")


install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(TARGETS ${PROJECT_NAME}_static DESTINATION lib)







