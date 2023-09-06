@ctype mat4 HMM_Mat4

@vs vs
uniform vs_params {
    mat4 model;
    mat4 view;
    mat4 projection;
    mat4 pose[120];
    mat4 invBindPose[120];
};

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 texCoord;
layout(location = 3) in vec4 weights;
layout(location = 4) in ivec4 joints;

out vec3 norm;
out vec3 fragPos;
out vec2 uv;

void main() {
    // mat4 skin = (pose[floatBitsToInt(joints.x)] *  invBindPose[floatBitsToInt(joints.x)]) * weights.x;
    // skin += (pose[floatBitsToInt(joints.y)] *  invBindPose[floatBitsToInt(joints.y)]) * weights.y;
    // skin += (pose[floatBitsToInt(joints.z)] * invBindPose[floatBitsToInt(joints.z)]) * weights.z;
    // skin += (pose[floatBitsToInt(joints.w)] * invBindPose[floatBitsToInt(joints.w)]) * weights.w;
    mat4 skin = (pose[joints.x] *  invBindPose[joints.x]) * weights.x;
    skin += (pose[joints.y] *  invBindPose[joints.y]) * weights.y;
    skin += (pose[joints.z] * invBindPose[joints.z]) * weights.z;
    skin += (pose[joints.w] * invBindPose[joints.w]) * weights.w;

    gl_Position = projection * view * model * skin * vec4(position, 1.0);
    
    fragPos = vec3(model * skin * vec4(position, 1.0));
    norm = vec3(model * skin * vec4(normal, 0.0f));
    uv = texCoord;
}
@end

@fs fs
in vec3 norm;
in vec3 fragPos;
in vec2 uv;


uniform fs_params {
    uniform vec3 light;
};

uniform texture2D tex;
uniform sampler smp;
out vec4 FragColor;

void main() {
	vec4 diffuseColor = texture(sampler2D(tex, smp), uv);

	vec3 n = normalize(norm);
	vec3 l = normalize(light);
	float diffuseIntensity = clamp(dot(n, l) + 0.1, 0, 1);

	FragColor = diffuseColor * diffuseIntensity;

	FragColor = vec4(1.0, 0.0, 0.0, 1.0);
}
@end

@program bonemesh vs fs