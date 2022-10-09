-- Compute

#version 450 core

layout(local_size_x = 1) in;

layout(std430, binding = 0) readonly buffer IndirectDrawCountBuffer {
    uint drawCount;
};

#ifdef VK_NV_mesh_shader
struct VkDrawMeshTasksIndirectCommandNV {
    uint taskCount;
    uint firstTask;
};
layout(std430, binding = 1) writeonly buffer TasksIndirectCommandBuffer {
    VkDrawMeshTasksIndirectCommandNV commands[];
};
layout(std430, binding = 2) writeonly buffer TasksIndirectCommandsCountBuffer {
    uint indirectCommandsCount;
};
layout(push_constant) uniform PushConstants {
    uint maxDrawMeshTasksCount;
};
#else
struct VkDrawMeshTasksIndirectCommandEXT {
    uint groupCountX;
    uint groupCountY;
    uint groupCountZ;
};
layout(std430, binding = 1) writeonly buffer TasksIndirectCommandBuffer {
    VkDrawMeshTasksIndirectCommandEXT commands[];
};
#endif

void main() {
#ifdef VK_NV_mesh_shader
    uint i = 0u;
    uint firstTask = 0;
    uint taskCount = drawCount;
    VkDrawMeshTasksIndirectCommandNV command;
    command.firstTask = 0;
    while (taskCount > maxDrawMeshTasksCount) {
        command.taskCount = drawCount;
        commands[i] = command;
        firstTask += maxDrawMeshTasksCount;
        taskCount -= maxDrawMeshTasksCount;
        i++;
    }
    command.taskCount = drawCount;
    commands[i] = command;
    indirectCommandsCount = i + (taskCount == 0u ? 0u : 1u);
#else
    VkDrawMeshTasksIndirectCommandEXT command;
    command.groupCountX = drawCount;
    command.groupCountY = 1;
    command.groupCountZ = 1;
    commands[0] = command;
#endif
}
