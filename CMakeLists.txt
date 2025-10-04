cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine)


# Set Release as default build type if none specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  message(STATUS "No build type specified. Defaulting to Release.")
endif()

# Optional: make choices visible in tools like cmake-gui
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")

include_directories(C:/sdk/glad/include)
include_directories(C:/sdk/GLFW/include)
include_directories(C:/sdk/ImGuiDocking/backends/)
include_directories(C:/sdk/ImGuiDocking/)
include_directories(C:/sdk/GLM)
include_directories(C:/sdk/fmt/include/fmt)
include_directories(C:/sdk/nlhjson/single_include/nlohmann)
include_directories(C:/sdk/Lua)

link_directories(${CMAKE_SOURCE_DIR}/lib)

add_executable(PhysicsEngine 
src/main.cpp
src/Shader.cpp 
src/GLBuffer.cpp 
src/App.cpp
src/Editor.cpp
src/cRenderer.cpp
src/RenderEngine.cpp
src/cRigidbody.cpp
src/PhysicsEngine.cpp
src/Entity.cpp
src/Engine.cpp
src/Scene.cpp
src/Component.cpp
src/Camera.cpp
C:/sdk/glad/src/glad.c
C:/sdk/ImGuiDocking/imgui.cpp
C:/sdk/ImGuiDocking/imgui_draw.cpp
C:/sdk/ImGuiDocking/backends/imgui_impl_glfw.cpp
C:/sdk/ImGuiDocking/imgui_tables.cpp
C:/sdk/ImGuiDocking/imgui_widgets.cpp
C:/sdk/ImGuiDocking/backends/imgui_impl_opengl3.cpp
C:/sdk/ImGuiDocking/misc/cpp/imgui_stdlib.cpp
)

target_link_libraries(PhysicsEngine PRIVATE libglfw3.a)
target_link_libraries(PhysicsEngine PRIVATE liblua.a)