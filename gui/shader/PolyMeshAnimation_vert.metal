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
                             constant float &simtime,
                             uint id [[vertex_id]])
{
    VertexOut vert;
    float3 pos = float3(position[id]);
    float3 trans = float3(translation[id]) * simtime;
    vert.position = mvp * float4(pos + trans, 1.f);
    vert.bary = barycentric[id];
    vert.color = color[id];
    return vert;
}
