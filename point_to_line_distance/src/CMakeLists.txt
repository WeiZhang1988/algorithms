# 收集src目录下的主程序源码
set(MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/point_to_line_distance.cpp)

# 生成.so文件（合并主程序）
add_library(point_to_line_distance SHARED ${MAIN_SRC})

# 指定安装路径（bin）
install(TARGETS point_to_line_distance LIBRARY DESTINATION bin)