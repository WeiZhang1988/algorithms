# 收集test目录下的主程序源码
set(MAIN_TEST ${CMAKE_CURRENT_SOURCE_DIR}/test.cpp)

# 生成可执行文件（合并主程序）
add_executable(test ${MAIN_TEST})

# 指定安装路径（bin）
install(TARGETS test RUNTIME DESTINATION bin)