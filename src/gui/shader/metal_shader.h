//
// Created by ziqwang on 10.06.20.
//

#ifndef TOPOLITE_METAL_SHADER_H
#define TOPOLITE_METAL_SHADER_H
#include <string>

const std::string metal_lines_frag = R"(using namespace metal;

struct VertexOut {
    float4 position [[position]];
    float3 color;
};


fragment float4 lines_frag_main(VertexOut in [[stage_in]])
{
    return float4(in.color, 1);
}
)";

const std::string metal_lines_vert = R"(using namespace metal;
#include <metal_stdlib>
#define eps 1E-6
#define z_fighting 1E-4

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

vertex VertexOut lines_vert_main(
const device packed_float3 *position,
constant float4x4 &mvp,
const device packed_float3 *linep1,
const device packed_float3 *linep2,
const device packed_float3 *color,
const device packed_float3 *translation,
const device packed_float3 *rotation,
const device packed_float3 *center,
const device int    *objectindex,
constant float &simtime,
constant int &type,
constant float &linewidth,
uint id [[vertex_id]])
{
    VertexOut vert;

    /*
     * Create transformation matrix
     */

    float3 trans = float3(translation[objectindex[id]]) * simtime;   //translational vector
    float3 cent = float3(center[objectindex[id]]);                 //center

    float3 rot_vec = float3(rotation[objectindex[id]]);              //rotation
    float theta = simtime * length(rot_vec);       //simulation time

    float3x3 R = float3x3(1);
    if(length(rot_vec) > eps){
        R = get_rotation_matrix_of_axis(normalize(rot_vec), theta); //rotation matrix
    }

    /*
     * Transform the line points
     */

    float3 p1 = float3(linep1[id]);
    float3 p2 = float3(linep2[id]);

    p1 = R * (p1 - cent) + cent + trans;
    p2 = R * (p2 - cent) + cent + trans;

    /*
     * Project to mvp space
     */

    float4 proj_p1 = mvp * float4(p1, 1);
    float4 proj_p2 = mvp * float4(p2, 1);

    p1 = float3(proj_p1.xy / proj_p1.w, 0);
    p2 = float3(proj_p2.xy / proj_p2.w, 0);

    /*
     * Compute offset direction vector
     */

    float3 v12 = (p2 - p1) / length(p2 - p1);

    float3 d = cross(float3(0, 0, 1), v12);
    d = d / length(d) * linewidth;

    /*
     * set up the position
     */

    //align the normal of line plane always pointing to your eye
    float3 pos = float3(position[id]);

    if(type == 0)
    {
        vert.position = pos[0] * (proj_p1 - float4(d, 0) * proj_p1.w) + pos[1] * (proj_p2 - float4(d, 0) * proj_p2.w) + pos[2] * (proj_p1 + float4(d, 0) * proj_p1.w);
    }
    else{
        vert.position = pos[0] * (proj_p1 + float4(d, 0) * proj_p1.w) + pos[1] * (proj_p2 - float4(d, 0) * proj_p2.w) + pos[2] * (proj_p2 + float4(d, 0) * proj_p2.w);
    }
    vert.position -= float4(0, 0, z_fighting * vert.position.w, 0); //solve z-fighting

    /*
    * set the color
    */
    vert.color = color[objectindex[id]];
    return vert;
}
)";

const std::string metal_polymeshes_frag = R"(using namespace metal;
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
)";

const std::string metal_polymeshes_vert = R"(using namespace metal;
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
)";
#endif //TOPOLITE_METAL_SHADER_H
