if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(utest utest.cpp)
  catkin_add_nosetests(utest.py)
endif (CATKIN_ENABLE_TESTING)
