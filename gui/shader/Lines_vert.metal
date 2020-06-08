using namespace metal;
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

vertex VertexOut lines_vert_main(
const device packed_float3 *position,
constant float4x4 &mvp,
const device packed_float3 *linep1,
const device packed_float3 *linep2,
const device packed_float3 *lineprev,
const device packed_float3 *linenext,
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

    float3 trans = float3(translation[id]) * simtime;   //translational vector
    float3 cent = float3(center[id]);                 //center

    float3 rot_vec = float3(rotation[id]);              //rotation
    float theta = simtime * length(rot_vec);       //simulation time

    float3x3 R = float3x3(1);
    if(length(rot_vec) > eps){
        R = get_rotation_matrix_of_axis(normalize(rot_vec), theta); //rotation matrix
    }

    /*
     * Transform the line points
     */

    float3 prev = float3(lineprev[id]);
    float3 p1 = float3(linep1[id]);
    float3 p2 = float3(linep2[id]);
    float3 next = float3(linenext[id]);

    prev = R * (prev - cent) + cent + trans;
    p1 = R * (p1 - cent) + cent + trans;
    p2 = R * (p2 - cent) + cent + trans;
    next = R * (next - cent) + cent + trans;

    /*
     * Project to mvp space
     */

    float4 proj_prev = mvp * float4(prev, 1);
    float4 proj_p1 = mvp * float4(p1, 1);
    float4 proj_p2 = mvp * float4(p2, 1);
    float4 proj_next = mvp * float4(next, 1);

    prev = float3(proj_prev.xy / proj_prev.w, 0);
    p1 = float3(proj_p1.xy / proj_p1.w, 0);
    p2 = float3(proj_p2.xy / proj_p2.w, 0);
    next = float3(proj_next.xy / proj_next.w, 0);

    /*
     * Compute offset direction vector
     */

    float3 v01 = (prev - p1) / length(prev - p1);
    float3 v12 = (p2 - p1) / length(p2 - p1);
    float3 v23 = (next - p2) / length(next - p2);



    float3 d1, d2;
    if(length(v01 + v12) < eps){
        d1 = cross(float3(0, 0, 1), v12);
        d1 = d1 / length(d1) * linewidth;
    }
    else{
         d1 = (v01 + v12) / length(v01 + v12) * linewidth / max(0.5, abs(sin(0.5 * acos(dot(v01, v12)))));
    }
    if(dot(float3(0, 0, 1), cross(v12, d1)) < 0){
            d1 = -d1;
    }

    if(length(-v12 + v23) < eps){
        d2 = cross(float3(0, 0, 1), v12);
        d2 = d2 / length(d2) * linewidth;
    }
    else{
        d2 = (-v12 + v23) / length(-v12 + v23) * linewidth / max(0.5, abs(sin(0.5 * acos(dot(-v12, v23)))));
    }

    if(dot(float3(0, 0, 1), cross(v12, d2)) < 0){
            d2 = -d2;
    }

    /*
     * set up the position
     */

    //align the normal of line plane always pointing to your eye
    float3 pos = float3(position[id]);

    if(type == 0){
        if(abs(pos[0] - 1) < eps){
            //set position
            vert.position = proj_p2 - float4(d2, 0) * proj_p2.w;
        }
        else if(abs(pos[1] - 1) < eps){
            vert.position = proj_p1 + float4(d1, 0) * proj_p1.w;
        }
        else{
            vert.position = proj_p1 - float4(d1, 0) * proj_p1.w;
        }
    }
    else{
        if(abs(pos[0] - 1) < eps){
            //set position
            vert.position = proj_p2 - float4(d2, 0) * proj_p2.w;
        }
        else if(abs(pos[1] - 1) < eps){
            vert.position = proj_p2 + float4(d2, 0) * proj_p2.w;
        }
        else{
            vert.position = proj_p1 + float4(d1, 0) * proj_p1.w;
        }
    }

    //set color
    vert.color = color[objectindex[id]];

    return vert;
}

