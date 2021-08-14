#pragma once
#include <vector>
namespace gl_primitives
{
    const std::vector<float> coordinate_system_vertex
    {
        0.0f, 0.0f, 0.0f, 1.0f, 0.f, 0.f,
        1.0f, 0.0f, 0.0f, 1.0f, 0.f, 0.f,

        0.0f, 0.0f, 0.0f, 0.0f, 1.f, 0.f,
        0.0f, 1.0f, 0.0f, 0.0f, 1.f, 0.f,

        0.0f, 0.0f, 0.0f, 0.0f, 0.f, 1.f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.f, 1.f,
    };

    const std::vector<unsigned int> coordinate_system_indices {
        0,1,2,3,4,5
    };

    const std::vector<float> plane_vertex
    {
            -0.5f,-0.5f,0.0f,  1.0f,0.0f,0.0f,
            -0.5f, 0.5f,0.0f,  1.0f,0.0f,0.0f,
             0.5f, 0.5f,0.0f,  1.0f,0.0f,0.0f,
             0.5f,-0.5f,0.0f,  1.0f,0.0f,0.0f,
    };

    const std::vector<unsigned int> plane_idices {
            0,1,2,2,3,0
    };

}
