cmake_minimum_required(VERSION 3.14)
project(rqt_diagnostic_graph_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})
install(PROGRAMS script/rqt_diagnostic_graph_monitor DESTINATION lib/${PROJECT_NAME})
ament_auto_package(INSTALL_TO_SHARE script)
