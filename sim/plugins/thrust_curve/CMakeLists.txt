# orca::ThrustCurve（gz-sim 8／Harmonic 的 system 外掛），在 docker/sim.Dockerfile 的 build stage 編。
cmake_minimum_required(VERSION 3.16)
project(orca_thrust_curve LANGUAGES CXX)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(OrcaThrustCurve SHARED ThrustCurve.cc)
set_target_properties(OrcaThrustCurve PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_options(OrcaThrustCurve PRIVATE -Wall -Wextra)
target_link_libraries(OrcaThrustCurve PRIVATE
  gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
