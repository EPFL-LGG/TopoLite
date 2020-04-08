using namespace metal;

struct VertexOut {
    float4 position [[position]];
    float3 color;
    float3 bary;
};

float edgeFactor(float3 vBC){
    float3 d = fwidth(vBC);
    float3 a3 = smoothstep(float3(0.0), d * 3, vBC);
    return min(min(a3.x, a3.y), a3.z);
}

fragment float4 fragment_main(  constant bool &wireframe,
                                VertexOut in [[stage_in]])
{
    if(in.bary[2] < 0.5 && wireframe){
        return float4(mix(float3(0.0), float3(1), edgeFactor(in.bary)), 1.0);
    }
    else{
        return float4(in.color, 1.0);
    }
}
