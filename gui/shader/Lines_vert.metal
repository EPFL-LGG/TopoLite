using namespace metal;
#include <metal_stdlib>

struct VertexOut {
    float4 position [[position]];
    float3 color;
};

struct Line{
    float3 sta;
    float3 dir;
    float3 center;
    float3 rotation;
    float3 translation;
    float simtime;
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

Line get_line_after_animation(Line line){
    
    //translational vector
    float3 trans = line.translation * line.simtime;
    
    //rotation matrix
    float3 rot_vec = line.rotation;
    float theta = line.simtime * length(rot_vec);
    
    float3x3 R = float3x3(1);
    if(length(rot_vec) > 0.0001){
        R = get_rotation_matrix_of_axis(normalize(rot_vec), theta);
    }

    line.dir = R * line.dir;
    line.sta = R * (line.sta - line.center) + line.center + trans;
    
    return line;
}

vertex VertexOut lines_vert_main(
                             const device packed_float3 *position,
                             constant float4x4 &proj,
                             constant float4x4 &mv,
                             const device packed_float3 *linesta,
                             const device packed_float3 *linedrt,
                             const device packed_float3 *color,
                             const device packed_float3 *translation,
                             const device packed_float3 *rotation,
                             const device packed_float3 *center,
                             const device int    *objectindex,
                             constant float &simtime,
                             uint id [[vertex_id]])
{
    VertexOut vert;
    Line line;
    
    //set up line
    line.dir = float3(linedrt[id]);
    line.sta = float3(linesta[id]);
    line.translation = float3(translation[objectindex[id]]);
    line.center = float3(center[objectindex[id]]);
    line.rotation = float3(rotation[objectindex[id]]);
    line.simtime = simtime;
    
    //get animated line
    line = get_line_after_animation(line);
    
    //align the normal of line plane always pointing to your eye
    float3 pos = float3(position[id]);
    float3 lsta = (mv * float4(linesta[id], 1)).xyz;
    float3 x_axis = (mv * float4(linedrt[id], 0)).xyz;
    float3 y_axis = cross(-lsta, x_axis);
    if(length(y_axis) > 0.001){
        y_axis = normalize(y_axis);
    }
    else{
        y_axis = float3(0, 1 ,0);
    }
    //set position
    vert.position = proj * float4(x_axis * pos[0] + y_axis * pos[1] + lsta, 1.0);
    
    //set color
    vert.color = color[objectindex[id]];
    
    return vert;
}
