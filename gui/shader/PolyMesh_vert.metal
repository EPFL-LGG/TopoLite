using namespace metal;
struct VertexOut {
                float4 position [[position]];
                float3 bary;
};

vertex VertexOut vertex_main(const device packed_float3 *position,
                             constant float4x4 &mvp,
                             const device packed_float3 *barycentric,
                             uint id [[vertex_id]])
{
    VertexOut vert;
    vert.position = mvp * float4(position[id], 1.f);
    vert.bary = barycentric[id];
    return vert;
}
