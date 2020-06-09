using namespace metal;
#include <metal_graphics>
#include <metal_stdlib>

struct VertexOut {
    float4 position [[position]];
    float3 color;
};

fragment float4 polymeshanimation_frag_main(
        VertexOut in [[stage_in]])
{
    return float4(in.color, 1.0);
}
