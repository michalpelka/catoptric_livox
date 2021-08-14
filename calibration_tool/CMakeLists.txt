cmake_minimum_required(VERSION 3.0.2)

project(catadioptric_livox)

find_package(Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS})

find_package (Eigen3 REQUIRED)
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
message("EIGEN3_INCLUDE_DIR" ${EIGEN3_INCLUDE_DIR})
include_directories(
  include
  3rd/Sophus
)
find_package(TBB COMPONENTS tbbmalloc tbbmalloc_proxy tbb_preview)
find_package(Boost QUIET REQUIRED COMPONENTS program_options)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(GLEW REQUIRED)
if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    link_libraries(${GLEW_LIBRARIES})
endif()

find_package(PCL 1.5 REQUIRED)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR} ${CERES_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)
include_directories(GL 3rd 3rd/imgui 3rd/imgui/backends)
add_library(GL_Codes
        # wrapers
        GL/IndexBuffer.cpp
        GL/Renderer.cpp
        #GL/Texture.cpp
        GL/VertexBuffer.cpp
        GL/Shader.cpp
        GL/VertexArray.cpp
        GL/VertexBufferLayout.cpp
        # imgui
        3rd/imgui/imgui.cpp 3rd/imgui/imgui_demo.cpp 3rd/imgui/imgui.cpp
        3rd/imgui/imgui_draw.cpp 3rd/imgui/imgui_widgets.cpp
        3rd/imgui/backends/imgui_impl_opengl3.cpp 3rd/imgui/backends/imgui_impl_glfw.cpp
        3rd/ImGuizmo/ImGuizmo.cpp
        )


add_executable(calibration_gui calibration_gui.cpp utils.cpp utils_io.cpp)
target_link_libraries(calibration_gui
        ${CERES_LIBRARIES} GL_Codes ${OPENGL_LIBRARIES} glfw ${GLEW_LIBRARIES} ${PCL_LIBRARIES} tbb Boost::program_options)


add_executable(simulate_gui simulation_gui.cpp utils.cpp)
target_link_libraries(simulate_gui
        ${CERES_LIBRARIES} GL_Codes ${OPENGL_LIBRARIES} glfw ${GLEW_LIBRARIES} ${PCL_LIBRARIES} Boost::program_options)