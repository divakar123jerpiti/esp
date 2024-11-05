cmake_minimum_required(VERSION 3.16)
set(PARTITION_TABLE ${CMAKE_CURRENT_LIST_DIR}/partitions.csv)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rgb_panel)
