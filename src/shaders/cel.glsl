@vs vs_cel
// Vertex shader doesn't need to do much in this case
in vec2 position;
in vec2 texcoord0;

out vec2 texcoord;

void main() {
    gl_Position = vec4(position, 0.0, 1.0);
    texcoord = texcoord0;
}
@end

@fs fs_cel
in vec2 texcoord;

uniform fs_params_cel {
    uniform vec2 iResolution;
};

uniform texture2D tex;
uniform sampler smp;
out vec4 frag_color;

#define SHADES 4

void main() {
    vec2 uv = texcoord;
    vec4 color = texture(sampler2D(tex, smp), uv);

    float brightness = (color.r + color.g + color.b) / 3.0; 
    float shade = floor(brightness * float(SHADES));
    float brightnessOfShade = shade / float(SHADES);
    float factor = brightness / brightnessOfShade;

    color.rgb /= vec3(factor);
    frag_color = color;
}
@end

@program cel vs_cel fs_cel