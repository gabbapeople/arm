# CMake generated Testfile for 
# Source directory: /home/ubuntu/ARM/ros_arm/src/rqt_ez_publisher
# Build directory: /home/ubuntu/ARM/ros_arm/build/rqt_ez_publisher
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rqt_ez_publisher_nosetests_test.function_test.py "/home/ubuntu/ARM/ros_arm/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ubuntu/ARM/ros_arm/build/test_results/rqt_ez_publisher/nosetests-test.function_test.py.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/ubuntu/ARM/ros_arm/build/test_results/rqt_ez_publisher" "/usr/bin/nosetests3 -P --process-timeout=60 /home/ubuntu/ARM/ros_arm/src/rqt_ez_publisher/test/function_test.py --with-xunit --xunit-file=/home/ubuntu/ARM/ros_arm/build/test_results/rqt_ez_publisher/nosetests-test.function_test.py.xml")
set_tests_properties(_ctest_rqt_ez_publisher_nosetests_test.function_test.py PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/ubuntu/ARM/ros_arm/src/rqt_ez_publisher/CMakeLists.txt;11;catkin_add_nosetests;/home/ubuntu/ARM/ros_arm/src/rqt_ez_publisher/CMakeLists.txt;0;")
add_test(_ctest_rqt_ez_publisher_rostest_test_ros.test "/home/ubuntu/ARM/ros_arm/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ubuntu/ARM/ros_arm/build/test_results/rqt_ez_publisher/rostest-test_ros.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/ubuntu/ARM/ros_arm/src/rqt_ez_publisher --package=rqt_ez_publisher --results-filename test_ros.xml --results-base-dir \"/home/ubuntu/ARM/ros_arm/build/test_results\" /home/ubuntu/ARM/ros_arm/src/rqt_ez_publisher/test/ros.test ")
set_tests_properties(_ctest_rqt_ez_publisher_rostest_test_ros.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/ubuntu/ARM/ros_arm/src/rqt_ez_publisher/CMakeLists.txt;12;add_rostest;/home/ubuntu/ARM/ros_arm/src/rqt_ez_publisher/CMakeLists.txt;0;")