#version 330
in vec3 vertex_color;
in vec3 vertex_bary;
uniform bool show_wireframe;
uniform bool show_face;
uniform vec4 line_color;
out vec4 FragColor;

float edgeFactor(vec3 vBC){
    vec3 d = fwidth(vBC);
    vec3 a3 = smoothstep(vec3(0.0), d * 3, vBC);
    return min(min(a3.x, a3.y), a3.z);
}

void main(void)
{
    float edge_factor = edgeFactor(vertex_bary);
    if(edge_factor < 0.6 && show_wireframe && vertex_bary[2] < 0.05){
        FragColor =  vec4(mix(vec3(line_color), vertex_color, edge_factor), 1.0);
    }
    else{
        if(!show_face) discard;
        FragColor =  vec4(vertex_color, 1.0);
    }
}