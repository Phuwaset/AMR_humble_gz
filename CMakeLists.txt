cmake_minimum_required(VERSION 3.5)
project(amr_mtt)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes models urdf worlds rviz map
    DESTINATION share/${PROJECT_NAME}/
)

install( DIRECTORY config/
         DESTINATION share/${PROJECT_NAME}/config)
         
         
install(PROGRAMS scripts/remapper.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
