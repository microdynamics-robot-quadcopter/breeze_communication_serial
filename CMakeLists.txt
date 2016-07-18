cmake_minimum_required(VERSION 2.8.3)
project(breeze_communication_serial)

find_package(Boost COMPONENTS system thread)
add_subdirectory(src bin)
