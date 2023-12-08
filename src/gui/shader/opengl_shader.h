//
// Created by ziqwang on 10.06.20.
//

#ifndef TOPOLITE_OPENGL_SHADER_H
#define TOPOLITE_OPENGL_SHADER_H

const std::string opengl_polymeshes_vert = R"(
#version 330
const float eps = 1E-6;
in vec3 position;
in vec3 color;

in vec3 translation;
in vec3 rotation;
in vec3 center;

uniform float simtime;
uniform mat4 mvp;

out vec3 vertex_color;

mat3 get_rotation_matrix_of_axis(vec3 u, float theta){
    float ct = cos(theta);
    float st = sin(theta);

    mat3 R;
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

void main(void)
{
    //rotation matrix
    float theta = simtime * length(rotation);
    mat3 R = mat3(1);
    if(length(rotation) > eps){
        R = get_rotation_matrix_of_axis(normalize(rotation), theta);
    }

    //color
    vertex_color = color;

    //vertex coordinates
    gl_Position = mvp * vec4(R * (position - center) + center + translation * simtime, 1.f);
}
)";

const std::string opengl_polymeshes_frag = R"(
#version 330
in vec3 vertex_color;
out vec4 FragColor;

void main(void)
{
    FragColor =  vec4(vertex_color, 1.0);
}
)";

const std::string opengl_lines_vert = R"(
#version 330
const float eps = 1E-6;
const float z_fighting = 1E-4;
in vec3 position;
in vec3 color;
in vec3 linep1;
in vec3 linep2;

in vec3 translation;
in vec3 rotation;
in vec3 center;

uniform float simtime;
uniform mat4 mvp;
uniform int type;
uniform float linewidth;

out vec3 vertex_color;

mat3 get_rotation_matrix_of_axis(vec3 u, float theta){
    float ct = cos(theta);
    float st = sin(theta);

    mat3 R;
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

void main(void)
{
    /*
     * Create transformation matrix
     */

    vec3 trans = translation * simtime;   //translational vector
    vec3 cent = center;                 //center

    vec3 rot_vec = rotation;              //rotation
    float theta = simtime * length(rot_vec);       //simulation time

    mat3 R = mat3(1);
    if(length(rot_vec) > eps){
        R = get_rotation_matrix_of_axis(normalize(rot_vec), theta); //rotation matrix
    }

    /*
     * Transform the line points
     */

    vec3 p1 = linep1;
    vec3 p2 = linep2;

    p1 = R * (p1 - cent) + cent + trans;
    p2 = R * (p2 - cent) + cent + trans;

    /*
     * Project to mvp space
     */

    vec4 proj_p1 = mvp * vec4(p1, 1);
    vec4 proj_p2 = mvp * vec4(p2, 1);

    p1 = vec3(proj_p1.xy / proj_p1.w, 0);
    p2 = vec3(proj_p2.xy / proj_p2.w, 0);

    /*
     * Compute offset direction vector
     */

    vec3 v12 = (p2 - p1) / length(p2 - p1);

    vec3 d = cross(vec3(0, 0, 1), v12);
    d = d / length(d) * linewidth;

    /*
     * set up the position
     */

    //align the normal of line plane always pointing to your eye
    vec3 pos = position;

    if(type == 0)
    {
        gl_Position = pos[0] * (proj_p1 - vec4(d, 0) * proj_p1.w) + pos[1] * (proj_p2 - vec4(d, 0) * proj_p2.w) + pos[2] * (proj_p1 + vec4(d, 0) * proj_p1.w);
    }
    else{
        gl_Position = pos[0] * (proj_p1 + vec4(d, 0) * proj_p1.w) + pos[1] * (proj_p2 - vec4(d, 0) * proj_p2.w) + pos[2] * (proj_p2 + vec4(d, 0) * proj_p2.w);
    }
    gl_Position -= vec4(0, 0, z_fighting * gl_Position.w, 0); //solve z-fighting

    /*
    * set the color
    */
    vertex_color = color;
}


)";

const std::string opengl_lines_frag = R"(
#version 330
in vec3 vertex_color;
out vec4 FragColor;
void main(void)
{
    FragColor = vec4(vertex_color, 1);
})";

#endif //TOPOLITE_OPENGL_SHADER_H
