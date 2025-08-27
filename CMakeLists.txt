cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine)


include_directories(C:/sdk/glad/include)
include_directories(C:/sdk/GLFW/include)
include_directories(C:/sdk/ImGuiDocking/backends/)
include_directories(C:/sdk/ImGuiDocking/)
include_directories(C:/sdk/GLM)

link_directories(${CMAKE_SOURCE_DIR}/lib)

add_executable(PhysicsEngine 
src/main.cpp
src/Shader.cpp 
src/GLBuffer.cpp 
C:/sdk/glad/src/glad.c
src/App.cpp
src/Editor.cpp
src/cRenderer.cpp
src/RenderEngine.cpp
src/cRigidbody.cpp
src/PhysicsEngine.cpp
src/Entity.cpp
src/Engine.cpp
src/Component.cpp
src/Camera.cpp
C:/sdk/ImGuiDocking/imgui.cpp
C:/sdk/ImGuiDocking/imgui_draw.cpp
C:/sdk/ImGuiDocking/backends/imgui_impl_glfw.cpp
C:/sdk/ImGuiDocking/imgui_tables.cpp
C:/sdk/ImGuiDocking/imgui_widgets.cpp
C:/sdk/ImGuiDocking/backends/imgui_impl_opengl3.cpp
)

target_link_libraries(PhysicsEngine PRIVATE libglfw3.a)