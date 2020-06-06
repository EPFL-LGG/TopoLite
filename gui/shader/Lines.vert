#version 330

in vec3 position;
in vec3 color;
in vec3 linesta;
in vec3 linedrt;

in vec3 translation;
in vec3 rotation;
in vec3 center;

uniform float simtime;
uniform mat4 mv;
uniform mat4 proj;

out vec3 vertex_color;

struct Line{
        vec3 sta;
        vec3 dir;
        vec3 center;
        vec3 rotation;
        vec3 translation;
        float simtime;
};

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

Line get_line_after_animation(Line line){
    
    //translational vector
    vec3 trans = line.translation * line.simtime;
    
    //rotation matrix
    vec3 rot_vec = line.rotation;
    float theta = line.simtime * length(rot_vec);
    
    mat3 R = mat3(1);
    if(length(rot_vec) > 0.0001){
        R = get_rotation_matrix_of_axis(normalize(rot_vec), theta);
    }

    line.dir = R * line.dir;
    line.sta = R * (line.sta - line.center) + line.center + trans;
    
    return line;
}

void main(void)
{
    Line line;
    
    //set up line
    line.dir = linedrt;
    line.sta = linesta;
    line.translation = translation;
    line.center = center;
    line.rotation = rotation;
    line.simtime = simtime;
    
    //get animated line
    line = get_line_after_animation(line);
    
    //align the normal of line plane always pointing to your eye
    vec3 pos = position;
    vec3 lsta = (mv * vec4(linesta, 1)).xyz;
    vec3 x_axis = (mv * vec4(linedrt, 0)).xyz;
    vec3 y_axis = cross(-lsta, x_axis);
    if(length(y_axis) > 0.001){
        y_axis = normalize(y_axis);
    }
    else{
        y_axis = vec3(0, 1 ,0);
    }

    //set position
    gl_Position = proj * vec4(x_axis * pos[0] + y_axis * pos[1] + lsta, 1.0);
    
    //set color
    vertex_color = color;
}
