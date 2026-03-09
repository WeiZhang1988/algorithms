cmake_minimum_required(VERSION 4.2)
project(point_to_line_distance LANGUAGES CXX)

# ȫ��C++��׼
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ��װǰ׺
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE PATH "��װǰ׺" FORCE)

# ȫ�ֱ���ѡ�MinGW/GCC��
if(NOT MSVC)
    add_compile_options(-Wall -Wextra -O2)
endif()

# �����ļ�Ŀ¼����Դ�����ҵ�include�º�src�µ��ļ���
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src)

# ����Դ����Ŀ¼����ִ��src/CMakeLists.txt��
add_subdirectory(src)

# ����ѡ�����Ӳ���Ŀ¼
add_subdirectory(test)