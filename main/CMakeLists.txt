# 主程序
add_executable(smartcar_demo main.cpp)

# 链接 OpenCV 库
target_link_libraries(smartcar_demo
    PRIVATE
        Threads::Threads
        common_lib
        ${OpenCV_LIBS}
)