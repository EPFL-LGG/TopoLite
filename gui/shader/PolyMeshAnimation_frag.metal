using namespace metal;

struct VertexOut {
    float4 position [[position]];
    float3 color;
    float3 bary;
};

float edgeFactor(float3 vBC){
    float3 d = fwidth(vBC);
    float3 a3 = smoothstep(float3(0.0), d * 2, vBC);
    return min(min(a3.x, a3.y), a3.z);
}


fragment float4 fragment_main(  constant bool &show_wireframe,
                                constant bool &show_face,
                                VertexOut in [[stage_in]])
{
    float edge_factor = edgeFactor(in.bary);
    if(edge_factor < 0.6 && show_wireframe){
        return float4(mix(float3(0.0), in.color, edge_factor), 1.0);
    }
    else{
        if(!show_face) discard_fragment();
        return float4(in.color, 1.0);
    }
}
