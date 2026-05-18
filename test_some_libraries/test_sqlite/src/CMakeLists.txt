find_package(SQLite3 REQUIRED)
add_executable(sqlitetest sqlitetest.cpp)
target_include_directories(sqlitetest PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(sqlitetest sqlite3)