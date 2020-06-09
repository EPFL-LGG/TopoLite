using namespace metal;
#include <metal_stdlib>
#define eps 1E-6
struct VertexOut {
    float4 position [[position]];
    float3 color;
};

float3x3 get_rotation_matrix_of_axis(float3 u, float theta)
{
    float ct = cos(theta);
    float st = sin(theta);

    float3x3 R;
    R[0][0] = ct + u[0] * u[0] * (1 - ct);
    R[0][1] = u[0] * u[1] * (1 - ct) - u[2] * st;
    R[0][2] = u[0] * u[2] * (1 - ct) + u[1] * st;

    R[1][0] = u[1] * u[0] * (1 - ct) + u[2] * st;
    R[1][1] = ct + u[1] * u[1] * (1 - ct);
    R[1][2] = u[1] * u[2] * (1 - ct)  - u[0] * st;

    R[2][0] = u[2] * u[0] * (1 - ct) - u[1] * st;
    R[2][1] = u[2] * u[1] * (1 - ct) + u[0] * st;
    R[2][2] = ct + u[2] * u[2] * (1 - ct);

    return R;
}

vertex VertexOut polymeshanimation_vertex_main(
                             const device packed_float3 *position,
                             constant float4x4 &mvp,
                             const device packed_float3 *color,
                             const device packed_float3 *translation,
                             const device packed_float3 *rotation,
                             const device packed_float3 *center,
                             const device int    *objectindex,
                             constant float &simtime,
                             uint id [[vertex_id]])
{
    VertexOut vert;

    //translational vector
    float3 trans = float3(translation[objectindex[id]]) * simtime;

    //rotation matrix
    float3 rot_vec = float3(rotation[objectindex[id]]);
    float theta = simtime * length(rot_vec);
    float3x3 R = float3x3(1);
    if(length(rot_vec) > eps)
    {
        R = get_rotation_matrix_of_axis(normalize(rot_vec), theta);
    }

    //center vector
    float3 cent = float3(center[objectindex[id]]);
    
    //color
    vert.color = color[objectindex[id]];

    //vertex coordinates
    float3 pos = float3(position[id]);
    vert.position = mvp * float4(R * (pos - cent) + cent + trans, 1.f);

    return vert;
}
