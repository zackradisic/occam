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
layout(location = 4) in uvec4 joints;

out vec3 norm;
out vec3 fragPos;
out vec2 uv;

void main() {
    // The inverse bind pose transforms the position to the space of the bone/joint.
    // The pose matrices are actually the result of
    // `pose_get_global_transform()`, which successively transforms the
    // bone/joint from its local space to the space of its parent ans so until
    // in world space, while also applying the transformations (rotation, etc.)
    // that each joint has.
    // invBindPose :: Space(world) -> Space(bone)
    // pose        :: Space(bone) -> Space(world)
    mat4 skin = (pose[int(joints.x)] *  invBindPose[int(joints.x)]) * weights.x;
    skin += (pose[int(joints.y)] *  invBindPose[int(joints.y)]) * weights.y;
    skin += (pose[int(joints.z)] * invBindPose[int(joints.z)]) * weights.z;
    skin += (pose[int(joints.w)] * invBindPose[int(joints.w)]) * weights.w;

    gl_Position = projection * view * model * skin * vec4(position, 1.0);
    // invert the normal because for some reason it's broken
    norm = vec3(model * skin * vec4(-normal, 0.0f));

    // gl_Position = projection * view * model * vec4(position, 1.0);
    // norm = vec3(model * vec4(-normal, 0.0f));
    
    fragPos = vec3(model * skin * vec4(position, 1.0));
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

	// FragColor = vec4(1.0, 1.0, 1.0, 1.0) * diffuseIntensity;
	// FragColor = vec4(1.0, 0.0, 0.0, 1.0) ;
}
@end

@program bonemesh vs fs