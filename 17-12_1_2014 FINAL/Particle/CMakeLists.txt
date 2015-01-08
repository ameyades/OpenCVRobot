cmake_minimum_required(VERSION 2.8)
project( Particle )
SET(COMPILE_DEFINITIONS -Werror)
#OPENCV
find_package( OpenCV REQUIRED )
add_executable(Particle lab5.cpp Particle.h Util_Map.h Util.h)
target_link_libraries(Particle wiringPi /opt/vc/lib/libmmal_core.so /opt/vc/lib/libmmal_util.so /opt/vc/lib/libmmal_vc_client.so /opt/vc/lib/libvcos.so /opt/vc/lib/libbcm_host.so ${OpenCV_LIBS})
