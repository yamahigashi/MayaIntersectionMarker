cmake_minimum_required(VERSION 3.27)

project(MayaIntersectionMarker
        VERSION 0.0.1
        LANGUAGES CXX CUDA)

# --------------------------------------------------
# User‑configurable options
# --------------------------------------------------
set(MAYA_VERSION "2026" CACHE STRING "Autodesk Maya major version to build against")
set(CUDA_ARCHS "75;86;89" CACHE STRING "List of CUDA Compute Capabilities to target")

# --------------------------------------------------
# Extra module search paths (custom FindXXX.cmake files)
# --------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# --------------------------------------------------
# Find Python for PTX conversion
# --------------------------------------------------
find_package(Python3 COMPONENTS Interpreter REQUIRED)
# if (NOT Python3_EXECUTABLE)
#     message(FATAL_ERROR "Python3 executable not found. Please install Python3.")

# --------------------------------------------------
# Required external packages
# --------------------------------------------------
find_package(Maya        REQUIRED)          # provides Maya::Maya, MAYA_PLUGIN() macro
find_package(OpenMP      REQUIRED)
find_package(embree 4.0  REQUIRED)
find_package(OptiX 9     REQUIRED)          # OptiX::optix & headers ≥ 7.x
find_package(CUDAToolkit REQUIRED)          # CUDA::cudart / CUDA::cuda_driver
find_package(OpenGL      REQUIRED)

# --------------------------------------------------
# Global compile settings
# --------------------------------------------------
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES ${CUDA_ARCHS})
set(CMAKE_CXX_STANDARD 17) # For std::filesystem
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# --------------------------------------------------
# PTX Compilation and Embedding
# --------------------------------------------------
set(PTX_SHADER_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/kernel/MochiKernelShaders.cu")
set(PTX_OUTPUT_NAME "MochiKernelShaders") # Base name for ptx and hpp
set(PTX_FILE "${CMAKE_CURRENT_BINARY_DIR}/${PTX_OUTPUT_NAME}.ptx")
set(PTX_HEADER_FILE "${CMAKE_CURRENT_BINARY_DIR}/${PTX_OUTPUT_NAME}_ptx.h")
set(PTX_TO_HEADER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/ptx_to_header.py") # Path to your script

# Custom command to compile CU to PTX
# Using add_custom_command allows CMake to manage dependencies correctly
add_custom_command(
    OUTPUT ${PTX_FILE}
    COMMAND ${CMAKE_CUDA_COMPILER} ${PTX_SHADER_SOURCE} -ptx -o ${PTX_FILE}
            --use_fast_math
            -arch=sm_86
            # $<$<BOOL:${CMAKE_BUILD_TYPE_INIT_DEBUG}>:-G> # Add -G for debug builds
            # Add CUDA architecture flags automatically
            # ${CMAKE_CUDA_ARCHITECTURES_FLAG}
            # Add include directories needed by the shader .cu file
            -I "${CMAKE_CURRENT_SOURCE_DIR}/srccuda_utils" # For PODs, shared math
            -I "${OptiX_INCLUDE_DIR}"
            -I "${CUDAToolkit_INCLUDE_DIRS}"
            # Add C++ standard if needed by headers included in shader
            -std=c++${CMAKE_CUDA_STANDARD}
    DEPENDS ${PTX_SHADER_SOURCE} # Re-run if shader source changes
    COMMENT "Compiling OptiX shader to PTX: ${PTX_FILE}"
    VERBATIM # Ensure command line arguments are passed correctly
)

# Custom command to convert PTX to C++ Header using the Python script
add_custom_command(
    OUTPUT ${PTX_HEADER_FILE}
    # COMMAND ${Python3_EXECUTABLE} ${PTX_TO_HEADER_SCRIPT} ${PTX_FILE} ${PTX_HEADER_FILE}
    COMMAND uv run --no-project ${PTX_TO_HEADER_SCRIPT} ${PTX_FILE} ${PTX_HEADER_FILE}
    DEPENDS ${PTX_FILE} # Depends on the PTX file being generated first
    COMMENT "Converting PTX to Header: ${PTX_HEADER_FILE}"
    VERBATIM
)

# Custom target to drive the generation (ensures commands run)
add_custom_target(MochiShaders ALL DEPENDS ${PTX_HEADER_FILE})


# --------------------------------------------------
# Sources ─ CU / CPP mixed
#   *The OptiX stub definition must appear exactly once → its own TU*
# --------------------------------------------------
file(GLOB_RECURSE PLUGIN_SRC CONFIGURE_DEPENDS
     # "src/*.cpp"
     # "src/*.cxx"
     # "src/*.cu"
    "src/intersectionMarkerCommand.cpp"
    "src/intersectionMarkerDrawOverride.cpp"
    "src/intersectionMarkerNode.cpp"
    "src/kernel/EmbreeKernel.cpp"
    "src/kernel/KDTreeKernel.cpp"
    "src/kernel/MochiKernel.cpp"
    "src/kernel/MochiKernelImpl.cu"
    "src/kernel/OctreeKernel.cpp"
    "src/main.cpp"
)

# --------------------------------------------------
# Plugin target
# --------------------------------------------------
add_library(${PROJECT_NAME} SHARED ${PLUGIN_SRC})
add_library(${PROJECT_NAME}::plugin ALIAS ${PROJECT_NAME})

# Public / private include paths
# (keep everything PRIVATE except headers meant for consumers)
# --------------------------------------------------
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CUDA_SEPARABLE_COMPILATION ON)

# First‑party includes
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src)

# --------------------------------------------------
# Link dependencies (prefer imported targets)
# --------------------------------------------------
include_directories(${MAYA_INCLUDE_DIR} maya lib
  ${OPENGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${GLM_DIR}
  ${OptiX_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
	${CUDAToolkit_INCLUDE_DIRS}
)

# Maya SDK + CUDA runtime / driver
link_directories(${MAYA_LIBRARY_DIR})
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${MAYA_LIBRARIES}
    ${OPENGL_LIBRARIES}
    CUDA::cudart
    CUDA::cuda_driver
    OptiX::OptiX
    ${OptiX_LIBRARY}
    ${optix_rpath}
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME}
      PRIVATE
        OpenMP::OpenMP_CXX
      )
endif()

if(TARGET embree)
    target_link_libraries(${PROJECT_NAME} PRIVATE embree)
endif()

get_property(dirs TARGET ${PROJECT_NAME} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

# --------------------------------------------------
# Mark as Maya plug‑in & install
# --------------------------------------------------
MAYA_PLUGIN(${PROJECT_NAME})

install(TARGETS      ${PROJECT_NAME}
        LIBRARY DESTINATION "${PROJECT_NAME}/plug-ins/win64/${MAYA_VERSION}")

