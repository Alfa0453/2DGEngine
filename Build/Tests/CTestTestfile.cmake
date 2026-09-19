# CMake generated Testfile for 
# Source directory: D:/C++_Projects/2DGEngine/Tests
# Build directory: D:/C++_Projects/2DGEngine/build/Tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[2DGTests]=] "D:/C++_Projects/2DGEngine/build/Tests/Debug/2DGTests.exe")
  set_tests_properties([=[2DGTests]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/C++_Projects/2DGEngine/Tests/CMakeLists.txt;39;add_test;D:/C++_Projects/2DGEngine/Tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[2DGTests]=] "D:/C++_Projects/2DGEngine/build/Tests/Release/2DGTests.exe")
  set_tests_properties([=[2DGTests]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/C++_Projects/2DGEngine/Tests/CMakeLists.txt;39;add_test;D:/C++_Projects/2DGEngine/Tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test([=[2DGTests]=] "D:/C++_Projects/2DGEngine/build/Tests/MinSizeRel/2DGTests.exe")
  set_tests_properties([=[2DGTests]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/C++_Projects/2DGEngine/Tests/CMakeLists.txt;39;add_test;D:/C++_Projects/2DGEngine/Tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[2DGTests]=] "D:/C++_Projects/2DGEngine/build/Tests/RelWithDebInfo/2DGTests.exe")
  set_tests_properties([=[2DGTests]=] PROPERTIES  _BACKTRACE_TRIPLES "D:/C++_Projects/2DGEngine/Tests/CMakeLists.txt;39;add_test;D:/C++_Projects/2DGEngine/Tests/CMakeLists.txt;0;")
else()
  add_test([=[2DGTests]=] NOT_AVAILABLE)
endif()
