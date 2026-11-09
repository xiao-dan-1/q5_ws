cmake_minimum_required(VERSION 3.8)
project(q5_description)

find_package(ament_cmake REQUIRED)

# urdf/ 和 meshes/ 由 scripts/import_model.py 从 Q5work 仓库生成（见 README）。
install(DIRECTORY urdf meshes launch rviz worlds config
        DESTINATION share/${PROJECT_NAME}
        OPTIONAL)

# ros2 run 从 lib/<pkg>/ 里找可执行文件，所以自检脚本装到 lib 而不是 share。
install(PROGRAMS scripts/check_description.py scripts/check_tf.py scripts/capture_image.py
        scripts/straight_drive.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
