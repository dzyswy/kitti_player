


protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS kitti.proto)

add_library(kitti_proto STATIC ${PROTO_SRCS})  
set_target_properties(kitti_proto PROPERTIES CLEAN_DIRECT_OUTPUT 1) 

install(FILES ${PROTO_HDRS} DESTINATION include)



