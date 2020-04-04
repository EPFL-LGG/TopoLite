using namespace metal;

struct VertexOut {
    float4 position [[position]];
    float3 bary;
};

float edgeFactor(float3 vBC){
    float3 d = fwidth(vBC);
    float3 a3 = smoothstep(float3(0.0), d * 3, vBC);
    return min(min(a3.x, a3.y), a3.z);
}

fragment float4 fragment_main(VertexOut in [[stage_in]])
{
    return float4(mix(float3(0.0), float3(1), edgeFactor(in.bary)), 1.0);
}
