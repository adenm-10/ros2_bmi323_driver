# lib/CMakeLists.txt

# Create a static library named "bmi323sensor"
add_library(bmi323sensor STATIC
  bmi323.c
  bmi3.c
  common.c
)

# Make header files visible to other targets
target_include_directories(bmi323sensor PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
