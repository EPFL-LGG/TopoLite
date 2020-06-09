#version 330
in vec3 position;
in vec3 barycentric;
in vec3 color;

in vec3 translation;
in vec3 rotation;
in vec3 center;

uniform float simtime;
uniform mat4 mvp;

out vec3 vertex_color;
out vec3 vertex_bary;

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
    if(length(rotation) > 0.0001){
        R = get_rotation_matrix_of_axis(normalize(rotation), theta);
    }

    //color
    vertex_color = color;

    //vertex coordinates
    gl_Position = mvp * vec4(R * (position - center) + center + translation * simtime, 1.f);

    //barycentric coordinates
    vertex_bary = barycentric;
}