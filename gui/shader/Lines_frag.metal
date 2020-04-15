using namespace metal;

struct VertexOut {
    float4 position [[position]];
    float3 color;
};


fragment float4 fragment_main(VertexOut in [[stage_in]])
{
    return float4(in.color, 1);
}
