cmake_minimum_required(VERSION 4.3.1)
project(zeromqtest)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(src_pub)
add_subdirectory(src_sub)
add_subdirectory(src_pub_sub)
add_subdirectory(src_pub_sub_pro)
add_subdirectory(src_srv_cli_pro)