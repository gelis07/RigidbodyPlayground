cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine)


include_directories(D:/sdk/glad/include)
include_directories(D:/sdk/GLFW/include)
include_directories(D:/sdk/ImGuiDocking/backends/)
include_directories(D:/sdk/ImGuiDocking/)
include_directories(D:/sdk/GLM)
include_directories(D:/sdk/fmt/include/fmt)
include_directories(D:/sdk/nlhjson/single_include/nlohmann)
include_directories(D:/sdk/Lua)

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
D:/sdk/glad/src/glad.c
D:/sdk/ImGuiDocking/imgui.cpp
D:/sdk/ImGuiDocking/imgui_draw.cpp
D:/sdk/ImGuiDocking/backends/imgui_impl_glfw.cpp
D:/sdk/ImGuiDocking/imgui_tables.cpp
D:/sdk/ImGuiDocking/imgui_widgets.cpp
D:/sdk/ImGuiDocking/backends/imgui_impl_opengl3.cpp
D:/sdk/ImGuiDocking/misc/cpp/imgui_stdlib.cpp
)

target_link_libraries(PhysicsEngine PRIVATE libglfw3.a)
target_link_libraries(PhysicsEngine PRIVATE lua54.lib)