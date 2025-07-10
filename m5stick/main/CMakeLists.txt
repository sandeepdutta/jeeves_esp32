set(CMAKE_CXX_STANDARD 17)

idf_component_register(SRCS "m5stick.cpp" "M5StickCPlus2.cpp"
                    INCLUDE_DIRS ".")
