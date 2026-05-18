find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROJECT_SOURCE_DIR}/proto/message.proto)

find_path(ZEROMQ_INCLUDE_DIR NAMES zmq.h)
find_library(ZEROMQ_LIBRARY NAMES zmq libzmq)
if(NOT ZEROMQ_INCLUDE_DIR OR NOT ZEROMQ_LIBRARY)
    message(FATAL_ERROR "ZeroMQ 未找到！请安装或指定路径")
endif()

add_executable(pub_sub_pro pub_sub_pro.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(pub_sub_pro PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(pub_sub_pro PRIVATE ${ZEROMQ_INCLUDE_DIR})
target_link_libraries(pub_sub_pro PRIVATE ${ZEROMQ_LIBRARY} ${PROTOBUF_LIBRARIES})