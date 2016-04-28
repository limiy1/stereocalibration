cmake_minimum_required ( VERSION 2.8)
project( stereocalib )

find_package ( OpenCV REQUIRED )

add_executable(stereocalib stereocalib.cpp projectiveDevice.cpp)

target_link_libraries(
    stereocalib
    ${OpenCV_LIBS}
)



