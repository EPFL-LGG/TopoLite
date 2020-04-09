using namespace metal;
struct VertexOut {
                float4 position [[position]];
                float3 color;
                float3 bary;
};

vertex VertexOut vertex_main(const device packed_float3 *position,
                             constant float4x4 &mvp,
                             const device packed_float3 *barycentric,
                             const device packed_float3 *color,
                             const device packed_float3 *translation,
                             const device int    *objectindex,
                             constant float &simtime,
                             uint id [[vertex_id]])
{
    VertexOut vert;

    //translational vector
    float3 trans = float3(translation[objectindex[id]]) * simtime;

    //color
    vert.color = color[objectindex[id]];

    //vertex coordinates
    float3 pos = float3(position[id]);
    vert.position = mvp * float4(pos + trans, 1.f);

    //barycentric coordinates
    vert.bary = barycentric[id];
    return vert;
}
