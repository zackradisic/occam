#pragma once
/*
    #version:1# (machine generated, don't edit!)

    Generated by sokol-shdc (https://github.com/floooh/sokol-tools)

    Cmdline: sokol-shdc -i /Users/zackradisic/Code/occam/src/shaders/bonemesh_cpu.glsl -o /Users/zackradisic/Code/occam/src/shaders/bonemesh_cpu.glsl.h -l glsl330:metal_macos:hlsl4 -f sokol

    Overview:

        Shader program 'bonemesh_cpu':
            Get shader desc: bonemesh_cpu_shader_desc(sg_query_backend());
            Vertex shader: vs
                Attribute slots:
                    ATTR_vs_position = 0
                    ATTR_vs_normal = 1
                    ATTR_vs_texCoord = 2
                Uniform block 'vs_params':
                    C struct: vs_params_t
                    Bind slot: SLOT_vs_params = 0
            Fragment shader: fs
                Uniform block 'fs_params':
                    C struct: fs_params_t
                    Bind slot: SLOT_fs_params = 0
                Image 'tex':
                    Type: SG_IMAGETYPE_2D
                    Sample Type: SG_IMAGESAMPLETYPE_FLOAT
                    Bind slot: SLOT_tex = 0
                Sampler 'smp':
                    Type: SG_SAMPLERTYPE_SAMPLE
                    Bind slot: SLOT_smp = 0
                Image Sampler Pair 'tex_smp':
                    Image: tex
                    Sampler: smp


    Shader descriptor structs:

        sg_shader bonemesh_cpu = sg_make_shader(bonemesh_cpu_shader_desc(sg_query_backend()));

    Vertex attribute locations for vertex shader 'vs':

        sg_pipeline pip = sg_make_pipeline(&(sg_pipeline_desc){
            .layout = {
                .attrs = {
                    [ATTR_vs_position] = { ... },
                    [ATTR_vs_normal] = { ... },
                    [ATTR_vs_texCoord] = { ... },
                },
            },
            ...});


    Image bind slots, use as index in sg_bindings.vs.images[] or .fs.images[]

        SLOT_tex = 0;

    Sampler bind slots, use as index in sg_bindings.vs.sampler[] or .fs.samplers[]

        SLOT_smp = 0;

    Bind slot and C-struct for uniform block 'vs_params':

        vs_params_t vs_params = {
            .model = ...;
            .view = ...;
            .projection = ...;
        };
        sg_apply_uniforms(SG_SHADERSTAGE_[VS|FS], SLOT_vs_params, &SG_RANGE(vs_params));

    Bind slot and C-struct for uniform block 'fs_params':

        fs_params_t fs_params = {
            .light = ...;
        };
        sg_apply_uniforms(SG_SHADERSTAGE_[VS|FS], SLOT_fs_params, &SG_RANGE(fs_params));

*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#if !defined(SOKOL_SHDC_ALIGN)
  #if defined(_MSC_VER)
    #define SOKOL_SHDC_ALIGN(a) __declspec(align(a))
  #else
    #define SOKOL_SHDC_ALIGN(a) __attribute__((aligned(a)))
  #endif
#endif
#define ATTR_vs_position (0)
#define ATTR_vs_normal (1)
#define ATTR_vs_texCoord (2)
#define SLOT_tex (0)
#define SLOT_smp (0)
#define SLOT_vs_params (0)
#pragma pack(push,1)
SOKOL_SHDC_ALIGN(16) typedef struct vs_params_t {
    HMM_Mat4 model;
    HMM_Mat4 view;
    HMM_Mat4 projection;
} vs_params_t;
#pragma pack(pop)
#define SLOT_fs_params (0)
#pragma pack(push,1)
SOKOL_SHDC_ALIGN(16) typedef struct fs_params_t {
    float light[3];
    uint8_t _pad_12[4];
} fs_params_t;
#pragma pack(pop)
/*
    #version 330
    
    uniform vec4 vs_params[12];
    layout(location = 0) in vec3 position;
    out vec3 fragPos;
    out vec3 norm;
    layout(location = 1) in vec3 normal;
    out vec2 uv;
    layout(location = 2) in vec2 texCoord;
    
    void main()
    {
        mat4 _29 = mat4(vs_params[0], vs_params[1], vs_params[2], vs_params[3]);
        vec4 _39 = vec4(position, 1.0);
        gl_Position = ((mat4(vs_params[8], vs_params[9], vs_params[10], vs_params[11]) * mat4(vs_params[4], vs_params[5], vs_params[6], vs_params[7])) * _29) * _39;
        fragPos = vec3((_29 * _39).xyz);
        norm = vec3((_29 * vec4(normal, 0.0)).xyz);
        uv = texCoord;
    }
    
*/
static const char vs_source_glsl330[600] = {
    0x23,0x76,0x65,0x72,0x73,0x69,0x6f,0x6e,0x20,0x33,0x33,0x30,0x0a,0x0a,0x75,0x6e,
    0x69,0x66,0x6f,0x72,0x6d,0x20,0x76,0x65,0x63,0x34,0x20,0x76,0x73,0x5f,0x70,0x61,
    0x72,0x61,0x6d,0x73,0x5b,0x31,0x32,0x5d,0x3b,0x0a,0x6c,0x61,0x79,0x6f,0x75,0x74,
    0x28,0x6c,0x6f,0x63,0x61,0x74,0x69,0x6f,0x6e,0x20,0x3d,0x20,0x30,0x29,0x20,0x69,
    0x6e,0x20,0x76,0x65,0x63,0x33,0x20,0x70,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x3b,
    0x0a,0x6f,0x75,0x74,0x20,0x76,0x65,0x63,0x33,0x20,0x66,0x72,0x61,0x67,0x50,0x6f,
    0x73,0x3b,0x0a,0x6f,0x75,0x74,0x20,0x76,0x65,0x63,0x33,0x20,0x6e,0x6f,0x72,0x6d,
    0x3b,0x0a,0x6c,0x61,0x79,0x6f,0x75,0x74,0x28,0x6c,0x6f,0x63,0x61,0x74,0x69,0x6f,
    0x6e,0x20,0x3d,0x20,0x31,0x29,0x20,0x69,0x6e,0x20,0x76,0x65,0x63,0x33,0x20,0x6e,
    0x6f,0x72,0x6d,0x61,0x6c,0x3b,0x0a,0x6f,0x75,0x74,0x20,0x76,0x65,0x63,0x32,0x20,
    0x75,0x76,0x3b,0x0a,0x6c,0x61,0x79,0x6f,0x75,0x74,0x28,0x6c,0x6f,0x63,0x61,0x74,
    0x69,0x6f,0x6e,0x20,0x3d,0x20,0x32,0x29,0x20,0x69,0x6e,0x20,0x76,0x65,0x63,0x32,
    0x20,0x74,0x65,0x78,0x43,0x6f,0x6f,0x72,0x64,0x3b,0x0a,0x0a,0x76,0x6f,0x69,0x64,
    0x20,0x6d,0x61,0x69,0x6e,0x28,0x29,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x6d,0x61,
    0x74,0x34,0x20,0x5f,0x32,0x39,0x20,0x3d,0x20,0x6d,0x61,0x74,0x34,0x28,0x76,0x73,
    0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5b,0x30,0x5d,0x2c,0x20,0x76,0x73,0x5f,0x70,
    0x61,0x72,0x61,0x6d,0x73,0x5b,0x31,0x5d,0x2c,0x20,0x76,0x73,0x5f,0x70,0x61,0x72,
    0x61,0x6d,0x73,0x5b,0x32,0x5d,0x2c,0x20,0x76,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,
    0x73,0x5b,0x33,0x5d,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x76,0x65,0x63,0x34,0x20,
    0x5f,0x33,0x39,0x20,0x3d,0x20,0x76,0x65,0x63,0x34,0x28,0x70,0x6f,0x73,0x69,0x74,
    0x69,0x6f,0x6e,0x2c,0x20,0x31,0x2e,0x30,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x67,
    0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x20,0x3d,0x20,0x28,0x28,0x6d,
    0x61,0x74,0x34,0x28,0x76,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5b,0x38,0x5d,
    0x2c,0x20,0x76,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5b,0x39,0x5d,0x2c,0x20,
    0x76,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5b,0x31,0x30,0x5d,0x2c,0x20,0x76,
    0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5b,0x31,0x31,0x5d,0x29,0x20,0x2a,0x20,
    0x6d,0x61,0x74,0x34,0x28,0x76,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5b,0x34,
    0x5d,0x2c,0x20,0x76,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5b,0x35,0x5d,0x2c,
    0x20,0x76,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5b,0x36,0x5d,0x2c,0x20,0x76,
    0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5b,0x37,0x5d,0x29,0x29,0x20,0x2a,0x20,
    0x5f,0x32,0x39,0x29,0x20,0x2a,0x20,0x5f,0x33,0x39,0x3b,0x0a,0x20,0x20,0x20,0x20,
    0x66,0x72,0x61,0x67,0x50,0x6f,0x73,0x20,0x3d,0x20,0x76,0x65,0x63,0x33,0x28,0x28,
    0x5f,0x32,0x39,0x20,0x2a,0x20,0x5f,0x33,0x39,0x29,0x2e,0x78,0x79,0x7a,0x29,0x3b,
    0x0a,0x20,0x20,0x20,0x20,0x6e,0x6f,0x72,0x6d,0x20,0x3d,0x20,0x76,0x65,0x63,0x33,
    0x28,0x28,0x5f,0x32,0x39,0x20,0x2a,0x20,0x76,0x65,0x63,0x34,0x28,0x6e,0x6f,0x72,
    0x6d,0x61,0x6c,0x2c,0x20,0x30,0x2e,0x30,0x29,0x29,0x2e,0x78,0x79,0x7a,0x29,0x3b,
    0x0a,0x20,0x20,0x20,0x20,0x75,0x76,0x20,0x3d,0x20,0x74,0x65,0x78,0x43,0x6f,0x6f,
    0x72,0x64,0x3b,0x0a,0x7d,0x0a,0x0a,0x00,
};
/*
    #version 330
    
    uniform vec4 fs_params[1];
    uniform sampler2D tex_smp;
    
    in vec2 uv;
    in vec3 norm;
    layout(location = 0) out vec4 FragColor;
    in vec3 fragPos;
    
    void main()
    {
        FragColor = texture(tex_smp, uv) * clamp(dot(normalize(norm), normalize(fs_params[0].xyz)) + 0.100000001490116119384765625, 0.0, 1.0);
        FragColor = vec4(1.0, 0.0, 0.0, 1.0);
    }
    
*/
static const char fs_source_glsl330[353] = {
    0x23,0x76,0x65,0x72,0x73,0x69,0x6f,0x6e,0x20,0x33,0x33,0x30,0x0a,0x0a,0x75,0x6e,
    0x69,0x66,0x6f,0x72,0x6d,0x20,0x76,0x65,0x63,0x34,0x20,0x66,0x73,0x5f,0x70,0x61,
    0x72,0x61,0x6d,0x73,0x5b,0x31,0x5d,0x3b,0x0a,0x75,0x6e,0x69,0x66,0x6f,0x72,0x6d,
    0x20,0x73,0x61,0x6d,0x70,0x6c,0x65,0x72,0x32,0x44,0x20,0x74,0x65,0x78,0x5f,0x73,
    0x6d,0x70,0x3b,0x0a,0x0a,0x69,0x6e,0x20,0x76,0x65,0x63,0x32,0x20,0x75,0x76,0x3b,
    0x0a,0x69,0x6e,0x20,0x76,0x65,0x63,0x33,0x20,0x6e,0x6f,0x72,0x6d,0x3b,0x0a,0x6c,
    0x61,0x79,0x6f,0x75,0x74,0x28,0x6c,0x6f,0x63,0x61,0x74,0x69,0x6f,0x6e,0x20,0x3d,
    0x20,0x30,0x29,0x20,0x6f,0x75,0x74,0x20,0x76,0x65,0x63,0x34,0x20,0x46,0x72,0x61,
    0x67,0x43,0x6f,0x6c,0x6f,0x72,0x3b,0x0a,0x69,0x6e,0x20,0x76,0x65,0x63,0x33,0x20,
    0x66,0x72,0x61,0x67,0x50,0x6f,0x73,0x3b,0x0a,0x0a,0x76,0x6f,0x69,0x64,0x20,0x6d,
    0x61,0x69,0x6e,0x28,0x29,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x46,0x72,0x61,0x67,
    0x43,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,0x74,0x65,0x78,0x74,0x75,0x72,0x65,0x28,
    0x74,0x65,0x78,0x5f,0x73,0x6d,0x70,0x2c,0x20,0x75,0x76,0x29,0x20,0x2a,0x20,0x63,
    0x6c,0x61,0x6d,0x70,0x28,0x64,0x6f,0x74,0x28,0x6e,0x6f,0x72,0x6d,0x61,0x6c,0x69,
    0x7a,0x65,0x28,0x6e,0x6f,0x72,0x6d,0x29,0x2c,0x20,0x6e,0x6f,0x72,0x6d,0x61,0x6c,
    0x69,0x7a,0x65,0x28,0x66,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5b,0x30,0x5d,
    0x2e,0x78,0x79,0x7a,0x29,0x29,0x20,0x2b,0x20,0x30,0x2e,0x31,0x30,0x30,0x30,0x30,
    0x30,0x30,0x30,0x31,0x34,0x39,0x30,0x31,0x31,0x36,0x31,0x31,0x39,0x33,0x38,0x34,
    0x37,0x36,0x35,0x36,0x32,0x35,0x2c,0x20,0x30,0x2e,0x30,0x2c,0x20,0x31,0x2e,0x30,
    0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x46,0x72,0x61,0x67,0x43,0x6f,0x6c,0x6f,0x72,
    0x20,0x3d,0x20,0x76,0x65,0x63,0x34,0x28,0x31,0x2e,0x30,0x2c,0x20,0x30,0x2e,0x30,
    0x2c,0x20,0x30,0x2e,0x30,0x2c,0x20,0x31,0x2e,0x30,0x29,0x3b,0x0a,0x7d,0x0a,0x0a,
    0x00,
};
/*
    cbuffer vs_params : register(b0)
    {
        row_major float4x4 _19_model : packoffset(c0);
        row_major float4x4 _19_view : packoffset(c4);
        row_major float4x4 _19_projection : packoffset(c8);
    };
    
    
    static float4 gl_Position;
    static float3 position;
    static float3 fragPos;
    static float3 norm;
    static float3 normal;
    static float2 uv;
    static float2 texCoord;
    
    struct SPIRV_Cross_Input
    {
        float3 position : TEXCOORD0;
        float3 normal : TEXCOORD1;
        float2 texCoord : TEXCOORD2;
    };
    
    struct SPIRV_Cross_Output
    {
        float3 norm : TEXCOORD0;
        float3 fragPos : TEXCOORD1;
        float2 uv : TEXCOORD2;
        float4 gl_Position : SV_Position;
    };
    
    void vert_main()
    {
        float4 _39 = float4(position, 1.0f);
        gl_Position = mul(_39, mul(_19_model, mul(_19_view, _19_projection)));
        fragPos = float3(mul(_39, _19_model).xyz);
        norm = float3(mul(float4(normal, 0.0f), _19_model).xyz);
        uv = texCoord;
    }
    
    SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
    {
        position = stage_input.position;
        normal = stage_input.normal;
        texCoord = stage_input.texCoord;
        vert_main();
        SPIRV_Cross_Output stage_output;
        stage_output.gl_Position = gl_Position;
        stage_output.fragPos = fragPos;
        stage_output.norm = norm;
        stage_output.uv = uv;
        return stage_output;
    }
*/
static const char vs_source_hlsl4[1289] = {
    0x63,0x62,0x75,0x66,0x66,0x65,0x72,0x20,0x76,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,
    0x73,0x20,0x3a,0x20,0x72,0x65,0x67,0x69,0x73,0x74,0x65,0x72,0x28,0x62,0x30,0x29,
    0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x72,0x6f,0x77,0x5f,0x6d,0x61,0x6a,0x6f,0x72,
    0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x78,0x34,0x20,0x5f,0x31,0x39,0x5f,0x6d,0x6f,
    0x64,0x65,0x6c,0x20,0x3a,0x20,0x70,0x61,0x63,0x6b,0x6f,0x66,0x66,0x73,0x65,0x74,
    0x28,0x63,0x30,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x72,0x6f,0x77,0x5f,0x6d,0x61,
    0x6a,0x6f,0x72,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x78,0x34,0x20,0x5f,0x31,0x39,
    0x5f,0x76,0x69,0x65,0x77,0x20,0x3a,0x20,0x70,0x61,0x63,0x6b,0x6f,0x66,0x66,0x73,
    0x65,0x74,0x28,0x63,0x34,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x72,0x6f,0x77,0x5f,
    0x6d,0x61,0x6a,0x6f,0x72,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x78,0x34,0x20,0x5f,
    0x31,0x39,0x5f,0x70,0x72,0x6f,0x6a,0x65,0x63,0x74,0x69,0x6f,0x6e,0x20,0x3a,0x20,
    0x70,0x61,0x63,0x6b,0x6f,0x66,0x66,0x73,0x65,0x74,0x28,0x63,0x38,0x29,0x3b,0x0a,
    0x7d,0x3b,0x0a,0x0a,0x0a,0x73,0x74,0x61,0x74,0x69,0x63,0x20,0x66,0x6c,0x6f,0x61,
    0x74,0x34,0x20,0x67,0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x3b,0x0a,
    0x73,0x74,0x61,0x74,0x69,0x63,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x70,0x6f,
    0x73,0x69,0x74,0x69,0x6f,0x6e,0x3b,0x0a,0x73,0x74,0x61,0x74,0x69,0x63,0x20,0x66,
    0x6c,0x6f,0x61,0x74,0x33,0x20,0x66,0x72,0x61,0x67,0x50,0x6f,0x73,0x3b,0x0a,0x73,
    0x74,0x61,0x74,0x69,0x63,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x6e,0x6f,0x72,
    0x6d,0x3b,0x0a,0x73,0x74,0x61,0x74,0x69,0x63,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,
    0x20,0x6e,0x6f,0x72,0x6d,0x61,0x6c,0x3b,0x0a,0x73,0x74,0x61,0x74,0x69,0x63,0x20,
    0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,0x75,0x76,0x3b,0x0a,0x73,0x74,0x61,0x74,0x69,
    0x63,0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,0x74,0x65,0x78,0x43,0x6f,0x6f,0x72,
    0x64,0x3b,0x0a,0x0a,0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x53,0x50,0x49,0x52,0x56,
    0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,0x49,0x6e,0x70,0x75,0x74,0x0a,0x7b,0x0a,0x20,
    0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x70,0x6f,0x73,0x69,0x74,0x69,
    0x6f,0x6e,0x20,0x3a,0x20,0x54,0x45,0x58,0x43,0x4f,0x4f,0x52,0x44,0x30,0x3b,0x0a,
    0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x6e,0x6f,0x72,0x6d,0x61,
    0x6c,0x20,0x3a,0x20,0x54,0x45,0x58,0x43,0x4f,0x4f,0x52,0x44,0x31,0x3b,0x0a,0x20,
    0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,0x74,0x65,0x78,0x43,0x6f,0x6f,
    0x72,0x64,0x20,0x3a,0x20,0x54,0x45,0x58,0x43,0x4f,0x4f,0x52,0x44,0x32,0x3b,0x0a,
    0x7d,0x3b,0x0a,0x0a,0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x53,0x50,0x49,0x52,0x56,
    0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,0x4f,0x75,0x74,0x70,0x75,0x74,0x0a,0x7b,0x0a,
    0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x6e,0x6f,0x72,0x6d,0x20,
    0x3a,0x20,0x54,0x45,0x58,0x43,0x4f,0x4f,0x52,0x44,0x30,0x3b,0x0a,0x20,0x20,0x20,
    0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x66,0x72,0x61,0x67,0x50,0x6f,0x73,0x20,
    0x3a,0x20,0x54,0x45,0x58,0x43,0x4f,0x4f,0x52,0x44,0x31,0x3b,0x0a,0x20,0x20,0x20,
    0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,0x75,0x76,0x20,0x3a,0x20,0x54,0x45,0x58,
    0x43,0x4f,0x4f,0x52,0x44,0x32,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,
    0x74,0x34,0x20,0x67,0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x20,0x3a,
    0x20,0x53,0x56,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x3b,0x0a,0x7d,0x3b,
    0x0a,0x0a,0x76,0x6f,0x69,0x64,0x20,0x76,0x65,0x72,0x74,0x5f,0x6d,0x61,0x69,0x6e,
    0x28,0x29,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x20,
    0x5f,0x33,0x39,0x20,0x3d,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x28,0x70,0x6f,0x73,
    0x69,0x74,0x69,0x6f,0x6e,0x2c,0x20,0x31,0x2e,0x30,0x66,0x29,0x3b,0x0a,0x20,0x20,
    0x20,0x20,0x67,0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x20,0x3d,0x20,
    0x6d,0x75,0x6c,0x28,0x5f,0x33,0x39,0x2c,0x20,0x6d,0x75,0x6c,0x28,0x5f,0x31,0x39,
    0x5f,0x6d,0x6f,0x64,0x65,0x6c,0x2c,0x20,0x6d,0x75,0x6c,0x28,0x5f,0x31,0x39,0x5f,
    0x76,0x69,0x65,0x77,0x2c,0x20,0x5f,0x31,0x39,0x5f,0x70,0x72,0x6f,0x6a,0x65,0x63,
    0x74,0x69,0x6f,0x6e,0x29,0x29,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x72,0x61,
    0x67,0x50,0x6f,0x73,0x20,0x3d,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x28,0x6d,0x75,
    0x6c,0x28,0x5f,0x33,0x39,0x2c,0x20,0x5f,0x31,0x39,0x5f,0x6d,0x6f,0x64,0x65,0x6c,
    0x29,0x2e,0x78,0x79,0x7a,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x6e,0x6f,0x72,0x6d,
    0x20,0x3d,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x28,0x6d,0x75,0x6c,0x28,0x66,0x6c,
    0x6f,0x61,0x74,0x34,0x28,0x6e,0x6f,0x72,0x6d,0x61,0x6c,0x2c,0x20,0x30,0x2e,0x30,
    0x66,0x29,0x2c,0x20,0x5f,0x31,0x39,0x5f,0x6d,0x6f,0x64,0x65,0x6c,0x29,0x2e,0x78,
    0x79,0x7a,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x75,0x76,0x20,0x3d,0x20,0x74,0x65,
    0x78,0x43,0x6f,0x6f,0x72,0x64,0x3b,0x0a,0x7d,0x0a,0x0a,0x53,0x50,0x49,0x52,0x56,
    0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,0x4f,0x75,0x74,0x70,0x75,0x74,0x20,0x6d,0x61,
    0x69,0x6e,0x28,0x53,0x50,0x49,0x52,0x56,0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,0x49,
    0x6e,0x70,0x75,0x74,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,0x70,0x75,0x74,
    0x29,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x70,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,
    0x20,0x3d,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,0x70,0x75,0x74,0x2e,0x70,
    0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x3b,0x0a,0x20,0x20,0x20,0x20,0x6e,0x6f,0x72,
    0x6d,0x61,0x6c,0x20,0x3d,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,0x70,0x75,
    0x74,0x2e,0x6e,0x6f,0x72,0x6d,0x61,0x6c,0x3b,0x0a,0x20,0x20,0x20,0x20,0x74,0x65,
    0x78,0x43,0x6f,0x6f,0x72,0x64,0x20,0x3d,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x69,
    0x6e,0x70,0x75,0x74,0x2e,0x74,0x65,0x78,0x43,0x6f,0x6f,0x72,0x64,0x3b,0x0a,0x20,
    0x20,0x20,0x20,0x76,0x65,0x72,0x74,0x5f,0x6d,0x61,0x69,0x6e,0x28,0x29,0x3b,0x0a,
    0x20,0x20,0x20,0x20,0x53,0x50,0x49,0x52,0x56,0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,
    0x4f,0x75,0x74,0x70,0x75,0x74,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x6f,0x75,0x74,
    0x70,0x75,0x74,0x3b,0x0a,0x20,0x20,0x20,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x6f,
    0x75,0x74,0x70,0x75,0x74,0x2e,0x67,0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,
    0x6e,0x20,0x3d,0x20,0x67,0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x3b,
    0x0a,0x20,0x20,0x20,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x6f,0x75,0x74,0x70,0x75,
    0x74,0x2e,0x66,0x72,0x61,0x67,0x50,0x6f,0x73,0x20,0x3d,0x20,0x66,0x72,0x61,0x67,
    0x50,0x6f,0x73,0x3b,0x0a,0x20,0x20,0x20,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x6f,
    0x75,0x74,0x70,0x75,0x74,0x2e,0x6e,0x6f,0x72,0x6d,0x20,0x3d,0x20,0x6e,0x6f,0x72,
    0x6d,0x3b,0x0a,0x20,0x20,0x20,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x6f,0x75,0x74,
    0x70,0x75,0x74,0x2e,0x75,0x76,0x20,0x3d,0x20,0x75,0x76,0x3b,0x0a,0x20,0x20,0x20,
    0x20,0x72,0x65,0x74,0x75,0x72,0x6e,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x6f,0x75,
    0x74,0x70,0x75,0x74,0x3b,0x0a,0x7d,0x0a,0x00,
};
/*
    cbuffer fs_params : register(b0)
    {
        float3 _35_light : packoffset(c0);
    };
    
    Texture2D<float4> tex : register(t0);
    SamplerState smp : register(s0);
    
    static float2 uv;
    static float3 norm;
    static float4 FragColor;
    static float3 fragPos;
    
    struct SPIRV_Cross_Input
    {
        float3 norm : TEXCOORD0;
        float3 fragPos : TEXCOORD1;
        float2 uv : TEXCOORD2;
    };
    
    struct SPIRV_Cross_Output
    {
        float4 FragColor : SV_Target0;
    };
    
    void frag_main()
    {
        FragColor = tex.Sample(smp, uv) * clamp(dot(normalize(norm), normalize(_35_light)) + 0.100000001490116119384765625f, 0.0f, 1.0f);
        FragColor = float4(1.0f, 0.0f, 0.0f, 1.0f);
    }
    
    SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
    {
        uv = stage_input.uv;
        norm = stage_input.norm;
        fragPos = stage_input.fragPos;
        frag_main();
        SPIRV_Cross_Output stage_output;
        stage_output.FragColor = FragColor;
        return stage_output;
    }
*/
static const char fs_source_hlsl4[895] = {
    0x63,0x62,0x75,0x66,0x66,0x65,0x72,0x20,0x66,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,
    0x73,0x20,0x3a,0x20,0x72,0x65,0x67,0x69,0x73,0x74,0x65,0x72,0x28,0x62,0x30,0x29,
    0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x5f,0x33,
    0x35,0x5f,0x6c,0x69,0x67,0x68,0x74,0x20,0x3a,0x20,0x70,0x61,0x63,0x6b,0x6f,0x66,
    0x66,0x73,0x65,0x74,0x28,0x63,0x30,0x29,0x3b,0x0a,0x7d,0x3b,0x0a,0x0a,0x54,0x65,
    0x78,0x74,0x75,0x72,0x65,0x32,0x44,0x3c,0x66,0x6c,0x6f,0x61,0x74,0x34,0x3e,0x20,
    0x74,0x65,0x78,0x20,0x3a,0x20,0x72,0x65,0x67,0x69,0x73,0x74,0x65,0x72,0x28,0x74,
    0x30,0x29,0x3b,0x0a,0x53,0x61,0x6d,0x70,0x6c,0x65,0x72,0x53,0x74,0x61,0x74,0x65,
    0x20,0x73,0x6d,0x70,0x20,0x3a,0x20,0x72,0x65,0x67,0x69,0x73,0x74,0x65,0x72,0x28,
    0x73,0x30,0x29,0x3b,0x0a,0x0a,0x73,0x74,0x61,0x74,0x69,0x63,0x20,0x66,0x6c,0x6f,
    0x61,0x74,0x32,0x20,0x75,0x76,0x3b,0x0a,0x73,0x74,0x61,0x74,0x69,0x63,0x20,0x66,
    0x6c,0x6f,0x61,0x74,0x33,0x20,0x6e,0x6f,0x72,0x6d,0x3b,0x0a,0x73,0x74,0x61,0x74,
    0x69,0x63,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x20,0x46,0x72,0x61,0x67,0x43,0x6f,
    0x6c,0x6f,0x72,0x3b,0x0a,0x73,0x74,0x61,0x74,0x69,0x63,0x20,0x66,0x6c,0x6f,0x61,
    0x74,0x33,0x20,0x66,0x72,0x61,0x67,0x50,0x6f,0x73,0x3b,0x0a,0x0a,0x73,0x74,0x72,
    0x75,0x63,0x74,0x20,0x53,0x50,0x49,0x52,0x56,0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,
    0x49,0x6e,0x70,0x75,0x74,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,
    0x74,0x33,0x20,0x6e,0x6f,0x72,0x6d,0x20,0x3a,0x20,0x54,0x45,0x58,0x43,0x4f,0x4f,
    0x52,0x44,0x30,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,
    0x66,0x72,0x61,0x67,0x50,0x6f,0x73,0x20,0x3a,0x20,0x54,0x45,0x58,0x43,0x4f,0x4f,
    0x52,0x44,0x31,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,
    0x75,0x76,0x20,0x3a,0x20,0x54,0x45,0x58,0x43,0x4f,0x4f,0x52,0x44,0x32,0x3b,0x0a,
    0x7d,0x3b,0x0a,0x0a,0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x53,0x50,0x49,0x52,0x56,
    0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,0x4f,0x75,0x74,0x70,0x75,0x74,0x0a,0x7b,0x0a,
    0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x20,0x46,0x72,0x61,0x67,0x43,
    0x6f,0x6c,0x6f,0x72,0x20,0x3a,0x20,0x53,0x56,0x5f,0x54,0x61,0x72,0x67,0x65,0x74,
    0x30,0x3b,0x0a,0x7d,0x3b,0x0a,0x0a,0x76,0x6f,0x69,0x64,0x20,0x66,0x72,0x61,0x67,
    0x5f,0x6d,0x61,0x69,0x6e,0x28,0x29,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x46,0x72,
    0x61,0x67,0x43,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,0x74,0x65,0x78,0x2e,0x53,0x61,
    0x6d,0x70,0x6c,0x65,0x28,0x73,0x6d,0x70,0x2c,0x20,0x75,0x76,0x29,0x20,0x2a,0x20,
    0x63,0x6c,0x61,0x6d,0x70,0x28,0x64,0x6f,0x74,0x28,0x6e,0x6f,0x72,0x6d,0x61,0x6c,
    0x69,0x7a,0x65,0x28,0x6e,0x6f,0x72,0x6d,0x29,0x2c,0x20,0x6e,0x6f,0x72,0x6d,0x61,
    0x6c,0x69,0x7a,0x65,0x28,0x5f,0x33,0x35,0x5f,0x6c,0x69,0x67,0x68,0x74,0x29,0x29,
    0x20,0x2b,0x20,0x30,0x2e,0x31,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x31,0x34,0x39,
    0x30,0x31,0x31,0x36,0x31,0x31,0x39,0x33,0x38,0x34,0x37,0x36,0x35,0x36,0x32,0x35,
    0x66,0x2c,0x20,0x30,0x2e,0x30,0x66,0x2c,0x20,0x31,0x2e,0x30,0x66,0x29,0x3b,0x0a,
    0x20,0x20,0x20,0x20,0x46,0x72,0x61,0x67,0x43,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,
    0x66,0x6c,0x6f,0x61,0x74,0x34,0x28,0x31,0x2e,0x30,0x66,0x2c,0x20,0x30,0x2e,0x30,
    0x66,0x2c,0x20,0x30,0x2e,0x30,0x66,0x2c,0x20,0x31,0x2e,0x30,0x66,0x29,0x3b,0x0a,
    0x7d,0x0a,0x0a,0x53,0x50,0x49,0x52,0x56,0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,0x4f,
    0x75,0x74,0x70,0x75,0x74,0x20,0x6d,0x61,0x69,0x6e,0x28,0x53,0x50,0x49,0x52,0x56,
    0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,0x49,0x6e,0x70,0x75,0x74,0x20,0x73,0x74,0x61,
    0x67,0x65,0x5f,0x69,0x6e,0x70,0x75,0x74,0x29,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,
    0x75,0x76,0x20,0x3d,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,0x70,0x75,0x74,
    0x2e,0x75,0x76,0x3b,0x0a,0x20,0x20,0x20,0x20,0x6e,0x6f,0x72,0x6d,0x20,0x3d,0x20,
    0x73,0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,0x70,0x75,0x74,0x2e,0x6e,0x6f,0x72,0x6d,
    0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x72,0x61,0x67,0x50,0x6f,0x73,0x20,0x3d,0x20,
    0x73,0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,0x70,0x75,0x74,0x2e,0x66,0x72,0x61,0x67,
    0x50,0x6f,0x73,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x72,0x61,0x67,0x5f,0x6d,0x61,
    0x69,0x6e,0x28,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x53,0x50,0x49,0x52,0x56,0x5f,
    0x43,0x72,0x6f,0x73,0x73,0x5f,0x4f,0x75,0x74,0x70,0x75,0x74,0x20,0x73,0x74,0x61,
    0x67,0x65,0x5f,0x6f,0x75,0x74,0x70,0x75,0x74,0x3b,0x0a,0x20,0x20,0x20,0x20,0x73,
    0x74,0x61,0x67,0x65,0x5f,0x6f,0x75,0x74,0x70,0x75,0x74,0x2e,0x46,0x72,0x61,0x67,
    0x43,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,0x46,0x72,0x61,0x67,0x43,0x6f,0x6c,0x6f,
    0x72,0x3b,0x0a,0x20,0x20,0x20,0x20,0x72,0x65,0x74,0x75,0x72,0x6e,0x20,0x73,0x74,
    0x61,0x67,0x65,0x5f,0x6f,0x75,0x74,0x70,0x75,0x74,0x3b,0x0a,0x7d,0x0a,0x00,
};
/*
    #include <metal_stdlib>
    #include <simd/simd.h>
    
    using namespace metal;
    
    struct vs_params
    {
        float4x4 model;
        float4x4 view;
        float4x4 projection;
    };
    
    struct main0_out
    {
        float3 norm [[user(locn0)]];
        float3 fragPos [[user(locn1)]];
        float2 uv [[user(locn2)]];
        float4 gl_Position [[position]];
    };
    
    struct main0_in
    {
        float3 position [[attribute(0)]];
        float3 normal [[attribute(1)]];
        float2 texCoord [[attribute(2)]];
    };
    
    vertex main0_out main0(main0_in in [[stage_in]], constant vs_params& _19 [[buffer(0)]])
    {
        main0_out out = {};
        float4 _39 = float4(in.position, 1.0);
        out.gl_Position = ((_19.projection * _19.view) * _19.model) * _39;
        out.fragPos = float3((_19.model * _39).xyz);
        out.norm = float3((_19.model * float4(in.normal, 0.0)).xyz);
        out.uv = in.texCoord;
        return out;
    }
    
*/
static const char vs_source_metal_macos[841] = {
    0x23,0x69,0x6e,0x63,0x6c,0x75,0x64,0x65,0x20,0x3c,0x6d,0x65,0x74,0x61,0x6c,0x5f,
    0x73,0x74,0x64,0x6c,0x69,0x62,0x3e,0x0a,0x23,0x69,0x6e,0x63,0x6c,0x75,0x64,0x65,
    0x20,0x3c,0x73,0x69,0x6d,0x64,0x2f,0x73,0x69,0x6d,0x64,0x2e,0x68,0x3e,0x0a,0x0a,
    0x75,0x73,0x69,0x6e,0x67,0x20,0x6e,0x61,0x6d,0x65,0x73,0x70,0x61,0x63,0x65,0x20,
    0x6d,0x65,0x74,0x61,0x6c,0x3b,0x0a,0x0a,0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x76,
    0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,
    0x6c,0x6f,0x61,0x74,0x34,0x78,0x34,0x20,0x6d,0x6f,0x64,0x65,0x6c,0x3b,0x0a,0x20,
    0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x78,0x34,0x20,0x76,0x69,0x65,0x77,
    0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x78,0x34,0x20,0x70,
    0x72,0x6f,0x6a,0x65,0x63,0x74,0x69,0x6f,0x6e,0x3b,0x0a,0x7d,0x3b,0x0a,0x0a,0x73,
    0x74,0x72,0x75,0x63,0x74,0x20,0x6d,0x61,0x69,0x6e,0x30,0x5f,0x6f,0x75,0x74,0x0a,
    0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x6e,0x6f,0x72,
    0x6d,0x20,0x5b,0x5b,0x75,0x73,0x65,0x72,0x28,0x6c,0x6f,0x63,0x6e,0x30,0x29,0x5d,
    0x5d,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x66,0x72,
    0x61,0x67,0x50,0x6f,0x73,0x20,0x5b,0x5b,0x75,0x73,0x65,0x72,0x28,0x6c,0x6f,0x63,
    0x6e,0x31,0x29,0x5d,0x5d,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,
    0x32,0x20,0x75,0x76,0x20,0x5b,0x5b,0x75,0x73,0x65,0x72,0x28,0x6c,0x6f,0x63,0x6e,
    0x32,0x29,0x5d,0x5d,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,
    0x20,0x67,0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x20,0x5b,0x5b,0x70,
    0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x5d,0x5d,0x3b,0x0a,0x7d,0x3b,0x0a,0x0a,0x73,
    0x74,0x72,0x75,0x63,0x74,0x20,0x6d,0x61,0x69,0x6e,0x30,0x5f,0x69,0x6e,0x0a,0x7b,
    0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x70,0x6f,0x73,0x69,
    0x74,0x69,0x6f,0x6e,0x20,0x5b,0x5b,0x61,0x74,0x74,0x72,0x69,0x62,0x75,0x74,0x65,
    0x28,0x30,0x29,0x5d,0x5d,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,
    0x33,0x20,0x6e,0x6f,0x72,0x6d,0x61,0x6c,0x20,0x5b,0x5b,0x61,0x74,0x74,0x72,0x69,
    0x62,0x75,0x74,0x65,0x28,0x31,0x29,0x5d,0x5d,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,
    0x6c,0x6f,0x61,0x74,0x32,0x20,0x74,0x65,0x78,0x43,0x6f,0x6f,0x72,0x64,0x20,0x5b,
    0x5b,0x61,0x74,0x74,0x72,0x69,0x62,0x75,0x74,0x65,0x28,0x32,0x29,0x5d,0x5d,0x3b,
    0x0a,0x7d,0x3b,0x0a,0x0a,0x76,0x65,0x72,0x74,0x65,0x78,0x20,0x6d,0x61,0x69,0x6e,
    0x30,0x5f,0x6f,0x75,0x74,0x20,0x6d,0x61,0x69,0x6e,0x30,0x28,0x6d,0x61,0x69,0x6e,
    0x30,0x5f,0x69,0x6e,0x20,0x69,0x6e,0x20,0x5b,0x5b,0x73,0x74,0x61,0x67,0x65,0x5f,
    0x69,0x6e,0x5d,0x5d,0x2c,0x20,0x63,0x6f,0x6e,0x73,0x74,0x61,0x6e,0x74,0x20,0x76,
    0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x26,0x20,0x5f,0x31,0x39,0x20,0x5b,0x5b,
    0x62,0x75,0x66,0x66,0x65,0x72,0x28,0x30,0x29,0x5d,0x5d,0x29,0x0a,0x7b,0x0a,0x20,
    0x20,0x20,0x20,0x6d,0x61,0x69,0x6e,0x30,0x5f,0x6f,0x75,0x74,0x20,0x6f,0x75,0x74,
    0x20,0x3d,0x20,0x7b,0x7d,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,
    0x34,0x20,0x5f,0x33,0x39,0x20,0x3d,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x28,0x69,
    0x6e,0x2e,0x70,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x2c,0x20,0x31,0x2e,0x30,0x29,
    0x3b,0x0a,0x20,0x20,0x20,0x20,0x6f,0x75,0x74,0x2e,0x67,0x6c,0x5f,0x50,0x6f,0x73,
    0x69,0x74,0x69,0x6f,0x6e,0x20,0x3d,0x20,0x28,0x28,0x5f,0x31,0x39,0x2e,0x70,0x72,
    0x6f,0x6a,0x65,0x63,0x74,0x69,0x6f,0x6e,0x20,0x2a,0x20,0x5f,0x31,0x39,0x2e,0x76,
    0x69,0x65,0x77,0x29,0x20,0x2a,0x20,0x5f,0x31,0x39,0x2e,0x6d,0x6f,0x64,0x65,0x6c,
    0x29,0x20,0x2a,0x20,0x5f,0x33,0x39,0x3b,0x0a,0x20,0x20,0x20,0x20,0x6f,0x75,0x74,
    0x2e,0x66,0x72,0x61,0x67,0x50,0x6f,0x73,0x20,0x3d,0x20,0x66,0x6c,0x6f,0x61,0x74,
    0x33,0x28,0x28,0x5f,0x31,0x39,0x2e,0x6d,0x6f,0x64,0x65,0x6c,0x20,0x2a,0x20,0x5f,
    0x33,0x39,0x29,0x2e,0x78,0x79,0x7a,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x6f,0x75,
    0x74,0x2e,0x6e,0x6f,0x72,0x6d,0x20,0x3d,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x28,
    0x28,0x5f,0x31,0x39,0x2e,0x6d,0x6f,0x64,0x65,0x6c,0x20,0x2a,0x20,0x66,0x6c,0x6f,
    0x61,0x74,0x34,0x28,0x69,0x6e,0x2e,0x6e,0x6f,0x72,0x6d,0x61,0x6c,0x2c,0x20,0x30,
    0x2e,0x30,0x29,0x29,0x2e,0x78,0x79,0x7a,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x6f,
    0x75,0x74,0x2e,0x75,0x76,0x20,0x3d,0x20,0x69,0x6e,0x2e,0x74,0x65,0x78,0x43,0x6f,
    0x6f,0x72,0x64,0x3b,0x0a,0x20,0x20,0x20,0x20,0x72,0x65,0x74,0x75,0x72,0x6e,0x20,
    0x6f,0x75,0x74,0x3b,0x0a,0x7d,0x0a,0x0a,0x00,
};
/*
    #include <metal_stdlib>
    #include <simd/simd.h>
    
    using namespace metal;
    
    struct fs_params
    {
        float3 light;
    };
    
    struct main0_out
    {
        float4 FragColor [[color(0)]];
    };
    
    struct main0_in
    {
        float3 norm [[user(locn0)]];
        float2 uv [[user(locn2)]];
    };
    
    fragment main0_out main0(main0_in in [[stage_in]], constant fs_params& _35 [[buffer(0)]], texture2d<float> tex [[texture(0)]], sampler smp [[sampler(0)]])
    {
        main0_out out = {};
        out.FragColor = tex.sample(smp, in.uv) * fast::clamp(dot(fast::normalize(in.norm), fast::normalize(_35.light)) + 0.100000001490116119384765625, 0.0, 1.0);
        out.FragColor = float4(1.0, 0.0, 0.0, 1.0);
        return out;
    }
    
*/
static const char fs_source_metal_macos[665] = {
    0x23,0x69,0x6e,0x63,0x6c,0x75,0x64,0x65,0x20,0x3c,0x6d,0x65,0x74,0x61,0x6c,0x5f,
    0x73,0x74,0x64,0x6c,0x69,0x62,0x3e,0x0a,0x23,0x69,0x6e,0x63,0x6c,0x75,0x64,0x65,
    0x20,0x3c,0x73,0x69,0x6d,0x64,0x2f,0x73,0x69,0x6d,0x64,0x2e,0x68,0x3e,0x0a,0x0a,
    0x75,0x73,0x69,0x6e,0x67,0x20,0x6e,0x61,0x6d,0x65,0x73,0x70,0x61,0x63,0x65,0x20,
    0x6d,0x65,0x74,0x61,0x6c,0x3b,0x0a,0x0a,0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x66,
    0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,
    0x6c,0x6f,0x61,0x74,0x33,0x20,0x6c,0x69,0x67,0x68,0x74,0x3b,0x0a,0x7d,0x3b,0x0a,
    0x0a,0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x6d,0x61,0x69,0x6e,0x30,0x5f,0x6f,0x75,
    0x74,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x20,0x46,
    0x72,0x61,0x67,0x43,0x6f,0x6c,0x6f,0x72,0x20,0x5b,0x5b,0x63,0x6f,0x6c,0x6f,0x72,
    0x28,0x30,0x29,0x5d,0x5d,0x3b,0x0a,0x7d,0x3b,0x0a,0x0a,0x73,0x74,0x72,0x75,0x63,
    0x74,0x20,0x6d,0x61,0x69,0x6e,0x30,0x5f,0x69,0x6e,0x0a,0x7b,0x0a,0x20,0x20,0x20,
    0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x6e,0x6f,0x72,0x6d,0x20,0x5b,0x5b,0x75,
    0x73,0x65,0x72,0x28,0x6c,0x6f,0x63,0x6e,0x30,0x29,0x5d,0x5d,0x3b,0x0a,0x20,0x20,
    0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,0x75,0x76,0x20,0x5b,0x5b,0x75,0x73,
    0x65,0x72,0x28,0x6c,0x6f,0x63,0x6e,0x32,0x29,0x5d,0x5d,0x3b,0x0a,0x7d,0x3b,0x0a,
    0x0a,0x66,0x72,0x61,0x67,0x6d,0x65,0x6e,0x74,0x20,0x6d,0x61,0x69,0x6e,0x30,0x5f,
    0x6f,0x75,0x74,0x20,0x6d,0x61,0x69,0x6e,0x30,0x28,0x6d,0x61,0x69,0x6e,0x30,0x5f,
    0x69,0x6e,0x20,0x69,0x6e,0x20,0x5b,0x5b,0x73,0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,
    0x5d,0x5d,0x2c,0x20,0x63,0x6f,0x6e,0x73,0x74,0x61,0x6e,0x74,0x20,0x66,0x73,0x5f,
    0x70,0x61,0x72,0x61,0x6d,0x73,0x26,0x20,0x5f,0x33,0x35,0x20,0x5b,0x5b,0x62,0x75,
    0x66,0x66,0x65,0x72,0x28,0x30,0x29,0x5d,0x5d,0x2c,0x20,0x74,0x65,0x78,0x74,0x75,
    0x72,0x65,0x32,0x64,0x3c,0x66,0x6c,0x6f,0x61,0x74,0x3e,0x20,0x74,0x65,0x78,0x20,
    0x5b,0x5b,0x74,0x65,0x78,0x74,0x75,0x72,0x65,0x28,0x30,0x29,0x5d,0x5d,0x2c,0x20,
    0x73,0x61,0x6d,0x70,0x6c,0x65,0x72,0x20,0x73,0x6d,0x70,0x20,0x5b,0x5b,0x73,0x61,
    0x6d,0x70,0x6c,0x65,0x72,0x28,0x30,0x29,0x5d,0x5d,0x29,0x0a,0x7b,0x0a,0x20,0x20,
    0x20,0x20,0x6d,0x61,0x69,0x6e,0x30,0x5f,0x6f,0x75,0x74,0x20,0x6f,0x75,0x74,0x20,
    0x3d,0x20,0x7b,0x7d,0x3b,0x0a,0x20,0x20,0x20,0x20,0x6f,0x75,0x74,0x2e,0x46,0x72,
    0x61,0x67,0x43,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,0x74,0x65,0x78,0x2e,0x73,0x61,
    0x6d,0x70,0x6c,0x65,0x28,0x73,0x6d,0x70,0x2c,0x20,0x69,0x6e,0x2e,0x75,0x76,0x29,
    0x20,0x2a,0x20,0x66,0x61,0x73,0x74,0x3a,0x3a,0x63,0x6c,0x61,0x6d,0x70,0x28,0x64,
    0x6f,0x74,0x28,0x66,0x61,0x73,0x74,0x3a,0x3a,0x6e,0x6f,0x72,0x6d,0x61,0x6c,0x69,
    0x7a,0x65,0x28,0x69,0x6e,0x2e,0x6e,0x6f,0x72,0x6d,0x29,0x2c,0x20,0x66,0x61,0x73,
    0x74,0x3a,0x3a,0x6e,0x6f,0x72,0x6d,0x61,0x6c,0x69,0x7a,0x65,0x28,0x5f,0x33,0x35,
    0x2e,0x6c,0x69,0x67,0x68,0x74,0x29,0x29,0x20,0x2b,0x20,0x30,0x2e,0x31,0x30,0x30,
    0x30,0x30,0x30,0x30,0x30,0x31,0x34,0x39,0x30,0x31,0x31,0x36,0x31,0x31,0x39,0x33,
    0x38,0x34,0x37,0x36,0x35,0x36,0x32,0x35,0x2c,0x20,0x30,0x2e,0x30,0x2c,0x20,0x31,
    0x2e,0x30,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x6f,0x75,0x74,0x2e,0x46,0x72,0x61,
    0x67,0x43,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x28,
    0x31,0x2e,0x30,0x2c,0x20,0x30,0x2e,0x30,0x2c,0x20,0x30,0x2e,0x30,0x2c,0x20,0x31,
    0x2e,0x30,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x72,0x65,0x74,0x75,0x72,0x6e,0x20,
    0x6f,0x75,0x74,0x3b,0x0a,0x7d,0x0a,0x0a,0x00,
};
#if !defined(SOKOL_GFX_INCLUDED)
  #error "Please include sokol_gfx.h before bonemesh_cpu.glsl.h"
#endif
static inline const sg_shader_desc* bonemesh_cpu_shader_desc(sg_backend backend) {
  if (backend == SG_BACKEND_GLCORE33) {
    static sg_shader_desc desc;
    static bool valid;
    if (!valid) {
      valid = true;
      desc.attrs[0].name = "position";
      desc.attrs[1].name = "normal";
      desc.attrs[2].name = "texCoord";
      desc.vs.source = vs_source_glsl330;
      desc.vs.entry = "main";
      desc.vs.uniform_blocks[0].size = 192;
      desc.vs.uniform_blocks[0].layout = SG_UNIFORMLAYOUT_STD140;
      desc.vs.uniform_blocks[0].uniforms[0].name = "vs_params";
      desc.vs.uniform_blocks[0].uniforms[0].type = SG_UNIFORMTYPE_FLOAT4;
      desc.vs.uniform_blocks[0].uniforms[0].array_count = 12;
      desc.fs.source = fs_source_glsl330;
      desc.fs.entry = "main";
      desc.fs.uniform_blocks[0].size = 16;
      desc.fs.uniform_blocks[0].layout = SG_UNIFORMLAYOUT_STD140;
      desc.fs.uniform_blocks[0].uniforms[0].name = "fs_params";
      desc.fs.uniform_blocks[0].uniforms[0].type = SG_UNIFORMTYPE_FLOAT4;
      desc.fs.uniform_blocks[0].uniforms[0].array_count = 1;
      desc.fs.images[0].used = true;
      desc.fs.images[0].multisampled = false;
      desc.fs.images[0].image_type = SG_IMAGETYPE_2D;
      desc.fs.images[0].sample_type = SG_IMAGESAMPLETYPE_FLOAT;
      desc.fs.samplers[0].used = true;
      desc.fs.samplers[0].sampler_type = SG_SAMPLERTYPE_SAMPLE;
      desc.fs.image_sampler_pairs[0].used = true;
      desc.fs.image_sampler_pairs[0].image_slot = 0;
      desc.fs.image_sampler_pairs[0].sampler_slot = 0;
      desc.fs.image_sampler_pairs[0].glsl_name = "tex_smp";
      desc.label = "bonemesh_cpu_shader";
    }
    return &desc;
  }
  if (backend == SG_BACKEND_D3D11) {
    static sg_shader_desc desc;
    static bool valid;
    if (!valid) {
      valid = true;
      desc.attrs[0].sem_name = "TEXCOORD";
      desc.attrs[0].sem_index = 0;
      desc.attrs[1].sem_name = "TEXCOORD";
      desc.attrs[1].sem_index = 1;
      desc.attrs[2].sem_name = "TEXCOORD";
      desc.attrs[2].sem_index = 2;
      desc.vs.source = vs_source_hlsl4;
      desc.vs.d3d11_target = "vs_4_0";
      desc.vs.entry = "main";
      desc.vs.uniform_blocks[0].size = 192;
      desc.vs.uniform_blocks[0].layout = SG_UNIFORMLAYOUT_STD140;
      desc.fs.source = fs_source_hlsl4;
      desc.fs.d3d11_target = "ps_4_0";
      desc.fs.entry = "main";
      desc.fs.uniform_blocks[0].size = 16;
      desc.fs.uniform_blocks[0].layout = SG_UNIFORMLAYOUT_STD140;
      desc.fs.images[0].used = true;
      desc.fs.images[0].multisampled = false;
      desc.fs.images[0].image_type = SG_IMAGETYPE_2D;
      desc.fs.images[0].sample_type = SG_IMAGESAMPLETYPE_FLOAT;
      desc.fs.samplers[0].used = true;
      desc.fs.samplers[0].sampler_type = SG_SAMPLERTYPE_SAMPLE;
      desc.fs.image_sampler_pairs[0].used = true;
      desc.fs.image_sampler_pairs[0].image_slot = 0;
      desc.fs.image_sampler_pairs[0].sampler_slot = 0;
      desc.label = "bonemesh_cpu_shader";
    }
    return &desc;
  }
  if (backend == SG_BACKEND_METAL_MACOS) {
    static sg_shader_desc desc;
    static bool valid;
    if (!valid) {
      valid = true;
      desc.vs.source = vs_source_metal_macos;
      desc.vs.entry = "main0";
      desc.vs.uniform_blocks[0].size = 192;
      desc.vs.uniform_blocks[0].layout = SG_UNIFORMLAYOUT_STD140;
      desc.fs.source = fs_source_metal_macos;
      desc.fs.entry = "main0";
      desc.fs.uniform_blocks[0].size = 16;
      desc.fs.uniform_blocks[0].layout = SG_UNIFORMLAYOUT_STD140;
      desc.fs.images[0].used = true;
      desc.fs.images[0].multisampled = false;
      desc.fs.images[0].image_type = SG_IMAGETYPE_2D;
      desc.fs.images[0].sample_type = SG_IMAGESAMPLETYPE_FLOAT;
      desc.fs.samplers[0].used = true;
      desc.fs.samplers[0].sampler_type = SG_SAMPLERTYPE_SAMPLE;
      desc.fs.image_sampler_pairs[0].used = true;
      desc.fs.image_sampler_pairs[0].image_slot = 0;
      desc.fs.image_sampler_pairs[0].sampler_slot = 0;
      desc.label = "bonemesh_cpu_shader";
    }
    return &desc;
  }
  return 0;
}
