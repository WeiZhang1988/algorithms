# �ռ�testĿ¼�µ�������Դ��
set(MAIN_TEST ${CMAKE_CURRENT_SOURCE_DIR}/test.cpp)

set(OpenCV_DIR "/home/wei-zhang/Software/opencv-4.12.0/install/lib/cmake/opencv4")
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs)

add_executable(test ${MAIN_TEST})
target_link_libraries(test PRIVATE opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs)

# ָ����װ·����bin��
install(TARGETS test RUNTIME DESTINATION bin)