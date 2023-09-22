#pragma once
/*
    #version:1# (machine generated, don't edit!)

    Generated by sokol-shdc (https://github.com/floooh/sokol-tools)

    Cmdline: sokol-shdc -i /Users/zackradisic/Code/occam/src/shaders/cel.glsl -o /Users/zackradisic/Code/occam/src/shaders/cel.glsl.h -l glsl330:metal_macos:hlsl4 -f sokol

    Overview:

        Shader program 'cel':
            Get shader desc: cel_shader_desc(sg_query_backend());
            Vertex shader: vs_cel
                Attribute slots:
                    ATTR_vs_cel_position = 0
                    ATTR_vs_cel_texcoord0 = 1
            Fragment shader: fs_cel
                Uniform block 'fs_params_cel':
                    C struct: fs_params_cel_t
                    Bind slot: SLOT_fs_params_cel = 0
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

        sg_shader cel = sg_make_shader(cel_shader_desc(sg_query_backend()));

    Vertex attribute locations for vertex shader 'vs_cel':

        sg_pipeline pip = sg_make_pipeline(&(sg_pipeline_desc){
            .layout = {
                .attrs = {
                    [ATTR_vs_cel_position] = { ... },
                    [ATTR_vs_cel_texcoord0] = { ... },
                },
            },
            ...});


    Image bind slots, use as index in sg_bindings.vs.images[] or .fs.images[]

        SLOT_tex = 0;

    Sampler bind slots, use as index in sg_bindings.vs.sampler[] or .fs.samplers[]

        SLOT_smp = 0;

    Bind slot and C-struct for uniform block 'fs_params_cel':

        fs_params_cel_t fs_params_cel = {
            .iResolution = ...;
            .cell_shading = ...;
        };
        sg_apply_uniforms(SG_SHADERSTAGE_[VS|FS], SLOT_fs_params_cel, &SG_RANGE(fs_params_cel));

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
#define ATTR_vs_cel_position (0)
#define ATTR_vs_cel_texcoord0 (1)
#define SLOT_tex (0)
#define SLOT_smp (0)
#define SLOT_fs_params_cel (0)
#pragma pack(push,1)
SOKOL_SHDC_ALIGN(16) typedef struct fs_params_cel_t {
    float iResolution[2];
    int cell_shading;
    uint8_t _pad_12[4];
} fs_params_cel_t;
#pragma pack(pop)
/*
    #version 330
    
    layout(location = 0) in vec3 position;
    out vec2 texcoord;
    layout(location = 1) in vec2 texcoord0;
    
    void main()
    {
        gl_Position = vec4(position, 1.0);
        texcoord = texcoord0;
    }
    
*/
static const char vs_cel_source_glsl330[196] = {
    0x23,0x76,0x65,0x72,0x73,0x69,0x6f,0x6e,0x20,0x33,0x33,0x30,0x0a,0x0a,0x6c,0x61,
    0x79,0x6f,0x75,0x74,0x28,0x6c,0x6f,0x63,0x61,0x74,0x69,0x6f,0x6e,0x20,0x3d,0x20,
    0x30,0x29,0x20,0x69,0x6e,0x20,0x76,0x65,0x63,0x33,0x20,0x70,0x6f,0x73,0x69,0x74,
    0x69,0x6f,0x6e,0x3b,0x0a,0x6f,0x75,0x74,0x20,0x76,0x65,0x63,0x32,0x20,0x74,0x65,
    0x78,0x63,0x6f,0x6f,0x72,0x64,0x3b,0x0a,0x6c,0x61,0x79,0x6f,0x75,0x74,0x28,0x6c,
    0x6f,0x63,0x61,0x74,0x69,0x6f,0x6e,0x20,0x3d,0x20,0x31,0x29,0x20,0x69,0x6e,0x20,
    0x76,0x65,0x63,0x32,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,0x72,0x64,0x30,0x3b,0x0a,
    0x0a,0x76,0x6f,0x69,0x64,0x20,0x6d,0x61,0x69,0x6e,0x28,0x29,0x0a,0x7b,0x0a,0x20,
    0x20,0x20,0x20,0x67,0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x20,0x3d,
    0x20,0x76,0x65,0x63,0x34,0x28,0x70,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x2c,0x20,
    0x31,0x2e,0x30,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,
    0x72,0x64,0x20,0x3d,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,0x72,0x64,0x30,0x3b,0x0a,
    0x7d,0x0a,0x0a,0x00,
};
/*
    #version 330
    
    struct fs_params_cel
    {
        vec2 iResolution;
        int cell_shading;
    };
    
    uniform fs_params_cel _31;
    
    uniform sampler2D tex_smp;
    
    in vec2 texcoord;
    layout(location = 0) out vec4 frag_color;
    
    void main()
    {
        vec4 color = texture(tex_smp, texcoord);
        if (_31.cell_shading == 1)
        {
            vec4 _83 = color;
            float _53 = (_83.x + _83.y) + _83.z;
            vec3 _73 = _83.xyz / vec3((_53 * 0.3333333432674407958984375) / (floor(_53 * 5.333333492279052734375) * 0.0625));
            vec4 _87 = _83;
            _87.x = _73.x;
            _87.y = _73.y;
            _87.z = _73.z;
            color = _87;
        }
        frag_color = color;
    }
    
*/
static const char fs_cel_source_glsl330[639] = {
    0x23,0x76,0x65,0x72,0x73,0x69,0x6f,0x6e,0x20,0x33,0x33,0x30,0x0a,0x0a,0x73,0x74,
    0x72,0x75,0x63,0x74,0x20,0x66,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5f,0x63,
    0x65,0x6c,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x76,0x65,0x63,0x32,0x20,0x69,0x52,
    0x65,0x73,0x6f,0x6c,0x75,0x74,0x69,0x6f,0x6e,0x3b,0x0a,0x20,0x20,0x20,0x20,0x69,
    0x6e,0x74,0x20,0x63,0x65,0x6c,0x6c,0x5f,0x73,0x68,0x61,0x64,0x69,0x6e,0x67,0x3b,
    0x0a,0x7d,0x3b,0x0a,0x0a,0x75,0x6e,0x69,0x66,0x6f,0x72,0x6d,0x20,0x66,0x73,0x5f,
    0x70,0x61,0x72,0x61,0x6d,0x73,0x5f,0x63,0x65,0x6c,0x20,0x5f,0x33,0x31,0x3b,0x0a,
    0x0a,0x75,0x6e,0x69,0x66,0x6f,0x72,0x6d,0x20,0x73,0x61,0x6d,0x70,0x6c,0x65,0x72,
    0x32,0x44,0x20,0x74,0x65,0x78,0x5f,0x73,0x6d,0x70,0x3b,0x0a,0x0a,0x69,0x6e,0x20,
    0x76,0x65,0x63,0x32,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,0x72,0x64,0x3b,0x0a,0x6c,
    0x61,0x79,0x6f,0x75,0x74,0x28,0x6c,0x6f,0x63,0x61,0x74,0x69,0x6f,0x6e,0x20,0x3d,
    0x20,0x30,0x29,0x20,0x6f,0x75,0x74,0x20,0x76,0x65,0x63,0x34,0x20,0x66,0x72,0x61,
    0x67,0x5f,0x63,0x6f,0x6c,0x6f,0x72,0x3b,0x0a,0x0a,0x76,0x6f,0x69,0x64,0x20,0x6d,
    0x61,0x69,0x6e,0x28,0x29,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x76,0x65,0x63,0x34,
    0x20,0x63,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,0x74,0x65,0x78,0x74,0x75,0x72,0x65,
    0x28,0x74,0x65,0x78,0x5f,0x73,0x6d,0x70,0x2c,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,
    0x72,0x64,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x69,0x66,0x20,0x28,0x5f,0x33,0x31,
    0x2e,0x63,0x65,0x6c,0x6c,0x5f,0x73,0x68,0x61,0x64,0x69,0x6e,0x67,0x20,0x3d,0x3d,
    0x20,0x31,0x29,0x0a,0x20,0x20,0x20,0x20,0x7b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,
    0x20,0x20,0x76,0x65,0x63,0x34,0x20,0x5f,0x38,0x33,0x20,0x3d,0x20,0x63,0x6f,0x6c,
    0x6f,0x72,0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,
    0x74,0x20,0x5f,0x35,0x33,0x20,0x3d,0x20,0x28,0x5f,0x38,0x33,0x2e,0x78,0x20,0x2b,
    0x20,0x5f,0x38,0x33,0x2e,0x79,0x29,0x20,0x2b,0x20,0x5f,0x38,0x33,0x2e,0x7a,0x3b,
    0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x76,0x65,0x63,0x33,0x20,0x5f,0x37,
    0x33,0x20,0x3d,0x20,0x5f,0x38,0x33,0x2e,0x78,0x79,0x7a,0x20,0x2f,0x20,0x76,0x65,
    0x63,0x33,0x28,0x28,0x5f,0x35,0x33,0x20,0x2a,0x20,0x30,0x2e,0x33,0x33,0x33,0x33,
    0x33,0x33,0x33,0x34,0x33,0x32,0x36,0x37,0x34,0x34,0x30,0x37,0x39,0x35,0x38,0x39,
    0x38,0x34,0x33,0x37,0x35,0x29,0x20,0x2f,0x20,0x28,0x66,0x6c,0x6f,0x6f,0x72,0x28,
    0x5f,0x35,0x33,0x20,0x2a,0x20,0x35,0x2e,0x33,0x33,0x33,0x33,0x33,0x33,0x34,0x39,
    0x32,0x32,0x37,0x39,0x30,0x35,0x32,0x37,0x33,0x34,0x33,0x37,0x35,0x29,0x20,0x2a,
    0x20,0x30,0x2e,0x30,0x36,0x32,0x35,0x29,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,
    0x20,0x20,0x20,0x76,0x65,0x63,0x34,0x20,0x5f,0x38,0x37,0x20,0x3d,0x20,0x5f,0x38,
    0x33,0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x5f,0x38,0x37,0x2e,0x78,
    0x20,0x3d,0x20,0x5f,0x37,0x33,0x2e,0x78,0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,
    0x20,0x20,0x5f,0x38,0x37,0x2e,0x79,0x20,0x3d,0x20,0x5f,0x37,0x33,0x2e,0x79,0x3b,
    0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x5f,0x38,0x37,0x2e,0x7a,0x20,0x3d,
    0x20,0x5f,0x37,0x33,0x2e,0x7a,0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
    0x63,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,0x5f,0x38,0x37,0x3b,0x0a,0x20,0x20,0x20,
    0x20,0x7d,0x0a,0x20,0x20,0x20,0x20,0x66,0x72,0x61,0x67,0x5f,0x63,0x6f,0x6c,0x6f,
    0x72,0x20,0x3d,0x20,0x63,0x6f,0x6c,0x6f,0x72,0x3b,0x0a,0x7d,0x0a,0x0a,0x00,
};
/*
    static float4 gl_Position;
    static float3 position;
    static float2 texcoord;
    static float2 texcoord0;
    
    struct SPIRV_Cross_Input
    {
        float3 position : TEXCOORD0;
        float2 texcoord0 : TEXCOORD1;
    };
    
    struct SPIRV_Cross_Output
    {
        float2 texcoord : TEXCOORD0;
        float4 gl_Position : SV_Position;
    };
    
    void vert_main()
    {
        gl_Position = float4(position, 1.0f);
        texcoord = texcoord0;
    }
    
    SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
    {
        position = stage_input.position;
        texcoord0 = stage_input.texcoord0;
        vert_main();
        SPIRV_Cross_Output stage_output;
        stage_output.gl_Position = gl_Position;
        stage_output.texcoord = texcoord;
        return stage_output;
    }
*/
static const char vs_cel_source_hlsl4[689] = {
    0x73,0x74,0x61,0x74,0x69,0x63,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x20,0x67,0x6c,
    0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x3b,0x0a,0x73,0x74,0x61,0x74,0x69,
    0x63,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x70,0x6f,0x73,0x69,0x74,0x69,0x6f,
    0x6e,0x3b,0x0a,0x73,0x74,0x61,0x74,0x69,0x63,0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,
    0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,0x72,0x64,0x3b,0x0a,0x73,0x74,0x61,0x74,0x69,
    0x63,0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,0x72,
    0x64,0x30,0x3b,0x0a,0x0a,0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x53,0x50,0x49,0x52,
    0x56,0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,0x49,0x6e,0x70,0x75,0x74,0x0a,0x7b,0x0a,
    0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x70,0x6f,0x73,0x69,0x74,
    0x69,0x6f,0x6e,0x20,0x3a,0x20,0x54,0x45,0x58,0x43,0x4f,0x4f,0x52,0x44,0x30,0x3b,
    0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,0x74,0x65,0x78,0x63,
    0x6f,0x6f,0x72,0x64,0x30,0x20,0x3a,0x20,0x54,0x45,0x58,0x43,0x4f,0x4f,0x52,0x44,
    0x31,0x3b,0x0a,0x7d,0x3b,0x0a,0x0a,0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x53,0x50,
    0x49,0x52,0x56,0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,0x4f,0x75,0x74,0x70,0x75,0x74,
    0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,0x74,0x65,
    0x78,0x63,0x6f,0x6f,0x72,0x64,0x20,0x3a,0x20,0x54,0x45,0x58,0x43,0x4f,0x4f,0x52,
    0x44,0x30,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x20,0x67,
    0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x20,0x3a,0x20,0x53,0x56,0x5f,
    0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x3b,0x0a,0x7d,0x3b,0x0a,0x0a,0x76,0x6f,
    0x69,0x64,0x20,0x76,0x65,0x72,0x74,0x5f,0x6d,0x61,0x69,0x6e,0x28,0x29,0x0a,0x7b,
    0x0a,0x20,0x20,0x20,0x20,0x67,0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,
    0x20,0x3d,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x28,0x70,0x6f,0x73,0x69,0x74,0x69,
    0x6f,0x6e,0x2c,0x20,0x31,0x2e,0x30,0x66,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x74,
    0x65,0x78,0x63,0x6f,0x6f,0x72,0x64,0x20,0x3d,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,
    0x72,0x64,0x30,0x3b,0x0a,0x7d,0x0a,0x0a,0x53,0x50,0x49,0x52,0x56,0x5f,0x43,0x72,
    0x6f,0x73,0x73,0x5f,0x4f,0x75,0x74,0x70,0x75,0x74,0x20,0x6d,0x61,0x69,0x6e,0x28,
    0x53,0x50,0x49,0x52,0x56,0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,0x49,0x6e,0x70,0x75,
    0x74,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,0x70,0x75,0x74,0x29,0x0a,0x7b,
    0x0a,0x20,0x20,0x20,0x20,0x70,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x20,0x3d,0x20,
    0x73,0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,0x70,0x75,0x74,0x2e,0x70,0x6f,0x73,0x69,
    0x74,0x69,0x6f,0x6e,0x3b,0x0a,0x20,0x20,0x20,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,
    0x72,0x64,0x30,0x20,0x3d,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,0x70,0x75,
    0x74,0x2e,0x74,0x65,0x78,0x63,0x6f,0x6f,0x72,0x64,0x30,0x3b,0x0a,0x20,0x20,0x20,
    0x20,0x76,0x65,0x72,0x74,0x5f,0x6d,0x61,0x69,0x6e,0x28,0x29,0x3b,0x0a,0x20,0x20,
    0x20,0x20,0x53,0x50,0x49,0x52,0x56,0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,0x4f,0x75,
    0x74,0x70,0x75,0x74,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x6f,0x75,0x74,0x70,0x75,
    0x74,0x3b,0x0a,0x20,0x20,0x20,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x6f,0x75,0x74,
    0x70,0x75,0x74,0x2e,0x67,0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x20,
    0x3d,0x20,0x67,0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x3b,0x0a,0x20,
    0x20,0x20,0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x6f,0x75,0x74,0x70,0x75,0x74,0x2e,
    0x74,0x65,0x78,0x63,0x6f,0x6f,0x72,0x64,0x20,0x3d,0x20,0x74,0x65,0x78,0x63,0x6f,
    0x6f,0x72,0x64,0x3b,0x0a,0x20,0x20,0x20,0x20,0x72,0x65,0x74,0x75,0x72,0x6e,0x20,
    0x73,0x74,0x61,0x67,0x65,0x5f,0x6f,0x75,0x74,0x70,0x75,0x74,0x3b,0x0a,0x7d,0x0a,
    0x00,
};
/*
    cbuffer fs_params_cel : register(b0)
    {
        float2 _31_iResolution : packoffset(c0);
        int _31_cell_shading : packoffset(c0.z);
    };
    
    Texture2D<float4> tex : register(t0);
    SamplerState smp : register(s0);
    
    static float2 texcoord;
    static float4 frag_color;
    
    struct SPIRV_Cross_Input
    {
        float2 texcoord : TEXCOORD0;
    };
    
    struct SPIRV_Cross_Output
    {
        float4 frag_color : SV_Target0;
    };
    
    void frag_main()
    {
        float4 color = tex.Sample(smp, texcoord);
        if (_31_cell_shading == 1)
        {
            float4 _83 = color;
            float _53 = (_83.x + _83.y) + _83.z;
            float3 _73 = _83.xyz / ((_53 * 0.3333333432674407958984375f) / (floor(_53 * 5.333333492279052734375f) * 0.0625f)).xxx;
            float4 _87 = _83;
            _87.x = _73.x;
            _87.y = _73.y;
            _87.z = _73.z;
            color = _87;
        }
        frag_color = color;
    }
    
    SPIRV_Cross_Output main(SPIRV_Cross_Input stage_input)
    {
        texcoord = stage_input.texcoord;
        frag_main();
        SPIRV_Cross_Output stage_output;
        stage_output.frag_color = frag_color;
        return stage_output;
    }
*/
static const char fs_cel_source_hlsl4[1057] = {
    0x63,0x62,0x75,0x66,0x66,0x65,0x72,0x20,0x66,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,
    0x73,0x5f,0x63,0x65,0x6c,0x20,0x3a,0x20,0x72,0x65,0x67,0x69,0x73,0x74,0x65,0x72,
    0x28,0x62,0x30,0x29,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,
    0x32,0x20,0x5f,0x33,0x31,0x5f,0x69,0x52,0x65,0x73,0x6f,0x6c,0x75,0x74,0x69,0x6f,
    0x6e,0x20,0x3a,0x20,0x70,0x61,0x63,0x6b,0x6f,0x66,0x66,0x73,0x65,0x74,0x28,0x63,
    0x30,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x69,0x6e,0x74,0x20,0x5f,0x33,0x31,0x5f,
    0x63,0x65,0x6c,0x6c,0x5f,0x73,0x68,0x61,0x64,0x69,0x6e,0x67,0x20,0x3a,0x20,0x70,
    0x61,0x63,0x6b,0x6f,0x66,0x66,0x73,0x65,0x74,0x28,0x63,0x30,0x2e,0x7a,0x29,0x3b,
    0x0a,0x7d,0x3b,0x0a,0x0a,0x54,0x65,0x78,0x74,0x75,0x72,0x65,0x32,0x44,0x3c,0x66,
    0x6c,0x6f,0x61,0x74,0x34,0x3e,0x20,0x74,0x65,0x78,0x20,0x3a,0x20,0x72,0x65,0x67,
    0x69,0x73,0x74,0x65,0x72,0x28,0x74,0x30,0x29,0x3b,0x0a,0x53,0x61,0x6d,0x70,0x6c,
    0x65,0x72,0x53,0x74,0x61,0x74,0x65,0x20,0x73,0x6d,0x70,0x20,0x3a,0x20,0x72,0x65,
    0x67,0x69,0x73,0x74,0x65,0x72,0x28,0x73,0x30,0x29,0x3b,0x0a,0x0a,0x73,0x74,0x61,
    0x74,0x69,0x63,0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,0x74,0x65,0x78,0x63,0x6f,
    0x6f,0x72,0x64,0x3b,0x0a,0x73,0x74,0x61,0x74,0x69,0x63,0x20,0x66,0x6c,0x6f,0x61,
    0x74,0x34,0x20,0x66,0x72,0x61,0x67,0x5f,0x63,0x6f,0x6c,0x6f,0x72,0x3b,0x0a,0x0a,
    0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x53,0x50,0x49,0x52,0x56,0x5f,0x43,0x72,0x6f,
    0x73,0x73,0x5f,0x49,0x6e,0x70,0x75,0x74,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,
    0x6c,0x6f,0x61,0x74,0x32,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,0x72,0x64,0x20,0x3a,
    0x20,0x54,0x45,0x58,0x43,0x4f,0x4f,0x52,0x44,0x30,0x3b,0x0a,0x7d,0x3b,0x0a,0x0a,
    0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x53,0x50,0x49,0x52,0x56,0x5f,0x43,0x72,0x6f,
    0x73,0x73,0x5f,0x4f,0x75,0x74,0x70,0x75,0x74,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,
    0x66,0x6c,0x6f,0x61,0x74,0x34,0x20,0x66,0x72,0x61,0x67,0x5f,0x63,0x6f,0x6c,0x6f,
    0x72,0x20,0x3a,0x20,0x53,0x56,0x5f,0x54,0x61,0x72,0x67,0x65,0x74,0x30,0x3b,0x0a,
    0x7d,0x3b,0x0a,0x0a,0x76,0x6f,0x69,0x64,0x20,0x66,0x72,0x61,0x67,0x5f,0x6d,0x61,
    0x69,0x6e,0x28,0x29,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,
    0x34,0x20,0x63,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,0x74,0x65,0x78,0x2e,0x53,0x61,
    0x6d,0x70,0x6c,0x65,0x28,0x73,0x6d,0x70,0x2c,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,
    0x72,0x64,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x69,0x66,0x20,0x28,0x5f,0x33,0x31,
    0x5f,0x63,0x65,0x6c,0x6c,0x5f,0x73,0x68,0x61,0x64,0x69,0x6e,0x67,0x20,0x3d,0x3d,
    0x20,0x31,0x29,0x0a,0x20,0x20,0x20,0x20,0x7b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,
    0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x20,0x5f,0x38,0x33,0x20,0x3d,0x20,0x63,
    0x6f,0x6c,0x6f,0x72,0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x66,0x6c,
    0x6f,0x61,0x74,0x20,0x5f,0x35,0x33,0x20,0x3d,0x20,0x28,0x5f,0x38,0x33,0x2e,0x78,
    0x20,0x2b,0x20,0x5f,0x38,0x33,0x2e,0x79,0x29,0x20,0x2b,0x20,0x5f,0x38,0x33,0x2e,
    0x7a,0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,
    0x33,0x20,0x5f,0x37,0x33,0x20,0x3d,0x20,0x5f,0x38,0x33,0x2e,0x78,0x79,0x7a,0x20,
    0x2f,0x20,0x28,0x28,0x5f,0x35,0x33,0x20,0x2a,0x20,0x30,0x2e,0x33,0x33,0x33,0x33,
    0x33,0x33,0x33,0x34,0x33,0x32,0x36,0x37,0x34,0x34,0x30,0x37,0x39,0x35,0x38,0x39,
    0x38,0x34,0x33,0x37,0x35,0x66,0x29,0x20,0x2f,0x20,0x28,0x66,0x6c,0x6f,0x6f,0x72,
    0x28,0x5f,0x35,0x33,0x20,0x2a,0x20,0x35,0x2e,0x33,0x33,0x33,0x33,0x33,0x33,0x34,
    0x39,0x32,0x32,0x37,0x39,0x30,0x35,0x32,0x37,0x33,0x34,0x33,0x37,0x35,0x66,0x29,
    0x20,0x2a,0x20,0x30,0x2e,0x30,0x36,0x32,0x35,0x66,0x29,0x29,0x2e,0x78,0x78,0x78,
    0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,
    0x20,0x5f,0x38,0x37,0x20,0x3d,0x20,0x5f,0x38,0x33,0x3b,0x0a,0x20,0x20,0x20,0x20,
    0x20,0x20,0x20,0x20,0x5f,0x38,0x37,0x2e,0x78,0x20,0x3d,0x20,0x5f,0x37,0x33,0x2e,
    0x78,0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x5f,0x38,0x37,0x2e,0x79,
    0x20,0x3d,0x20,0x5f,0x37,0x33,0x2e,0x79,0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,
    0x20,0x20,0x5f,0x38,0x37,0x2e,0x7a,0x20,0x3d,0x20,0x5f,0x37,0x33,0x2e,0x7a,0x3b,
    0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x63,0x6f,0x6c,0x6f,0x72,0x20,0x3d,
    0x20,0x5f,0x38,0x37,0x3b,0x0a,0x20,0x20,0x20,0x20,0x7d,0x0a,0x20,0x20,0x20,0x20,
    0x66,0x72,0x61,0x67,0x5f,0x63,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,0x63,0x6f,0x6c,
    0x6f,0x72,0x3b,0x0a,0x7d,0x0a,0x0a,0x53,0x50,0x49,0x52,0x56,0x5f,0x43,0x72,0x6f,
    0x73,0x73,0x5f,0x4f,0x75,0x74,0x70,0x75,0x74,0x20,0x6d,0x61,0x69,0x6e,0x28,0x53,
    0x50,0x49,0x52,0x56,0x5f,0x43,0x72,0x6f,0x73,0x73,0x5f,0x49,0x6e,0x70,0x75,0x74,
    0x20,0x73,0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,0x70,0x75,0x74,0x29,0x0a,0x7b,0x0a,
    0x20,0x20,0x20,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,0x72,0x64,0x20,0x3d,0x20,0x73,
    0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,0x70,0x75,0x74,0x2e,0x74,0x65,0x78,0x63,0x6f,
    0x6f,0x72,0x64,0x3b,0x0a,0x20,0x20,0x20,0x20,0x66,0x72,0x61,0x67,0x5f,0x6d,0x61,
    0x69,0x6e,0x28,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x53,0x50,0x49,0x52,0x56,0x5f,
    0x43,0x72,0x6f,0x73,0x73,0x5f,0x4f,0x75,0x74,0x70,0x75,0x74,0x20,0x73,0x74,0x61,
    0x67,0x65,0x5f,0x6f,0x75,0x74,0x70,0x75,0x74,0x3b,0x0a,0x20,0x20,0x20,0x20,0x73,
    0x74,0x61,0x67,0x65,0x5f,0x6f,0x75,0x74,0x70,0x75,0x74,0x2e,0x66,0x72,0x61,0x67,
    0x5f,0x63,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,0x66,0x72,0x61,0x67,0x5f,0x63,0x6f,
    0x6c,0x6f,0x72,0x3b,0x0a,0x20,0x20,0x20,0x20,0x72,0x65,0x74,0x75,0x72,0x6e,0x20,
    0x73,0x74,0x61,0x67,0x65,0x5f,0x6f,0x75,0x74,0x70,0x75,0x74,0x3b,0x0a,0x7d,0x0a,
    0x00,
};
/*
    #include <metal_stdlib>
    #include <simd/simd.h>
    
    using namespace metal;
    
    struct main0_out
    {
        float2 texcoord [[user(locn0)]];
        float4 gl_Position [[position]];
    };
    
    struct main0_in
    {
        float3 position [[attribute(0)]];
        float2 texcoord0 [[attribute(1)]];
    };
    
    vertex main0_out main0(main0_in in [[stage_in]])
    {
        main0_out out = {};
        out.gl_Position = float4(in.position, 1.0);
        out.texcoord = in.texcoord0;
        return out;
    }
    
*/
static const char vs_cel_source_metal_macos[444] = {
    0x23,0x69,0x6e,0x63,0x6c,0x75,0x64,0x65,0x20,0x3c,0x6d,0x65,0x74,0x61,0x6c,0x5f,
    0x73,0x74,0x64,0x6c,0x69,0x62,0x3e,0x0a,0x23,0x69,0x6e,0x63,0x6c,0x75,0x64,0x65,
    0x20,0x3c,0x73,0x69,0x6d,0x64,0x2f,0x73,0x69,0x6d,0x64,0x2e,0x68,0x3e,0x0a,0x0a,
    0x75,0x73,0x69,0x6e,0x67,0x20,0x6e,0x61,0x6d,0x65,0x73,0x70,0x61,0x63,0x65,0x20,
    0x6d,0x65,0x74,0x61,0x6c,0x3b,0x0a,0x0a,0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x6d,
    0x61,0x69,0x6e,0x30,0x5f,0x6f,0x75,0x74,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,
    0x6c,0x6f,0x61,0x74,0x32,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,0x72,0x64,0x20,0x5b,
    0x5b,0x75,0x73,0x65,0x72,0x28,0x6c,0x6f,0x63,0x6e,0x30,0x29,0x5d,0x5d,0x3b,0x0a,
    0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x20,0x67,0x6c,0x5f,0x50,0x6f,
    0x73,0x69,0x74,0x69,0x6f,0x6e,0x20,0x5b,0x5b,0x70,0x6f,0x73,0x69,0x74,0x69,0x6f,
    0x6e,0x5d,0x5d,0x3b,0x0a,0x7d,0x3b,0x0a,0x0a,0x73,0x74,0x72,0x75,0x63,0x74,0x20,
    0x6d,0x61,0x69,0x6e,0x30,0x5f,0x69,0x6e,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,
    0x6c,0x6f,0x61,0x74,0x33,0x20,0x70,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x20,0x5b,
    0x5b,0x61,0x74,0x74,0x72,0x69,0x62,0x75,0x74,0x65,0x28,0x30,0x29,0x5d,0x5d,0x3b,
    0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,0x74,0x65,0x78,0x63,
    0x6f,0x6f,0x72,0x64,0x30,0x20,0x5b,0x5b,0x61,0x74,0x74,0x72,0x69,0x62,0x75,0x74,
    0x65,0x28,0x31,0x29,0x5d,0x5d,0x3b,0x0a,0x7d,0x3b,0x0a,0x0a,0x76,0x65,0x72,0x74,
    0x65,0x78,0x20,0x6d,0x61,0x69,0x6e,0x30,0x5f,0x6f,0x75,0x74,0x20,0x6d,0x61,0x69,
    0x6e,0x30,0x28,0x6d,0x61,0x69,0x6e,0x30,0x5f,0x69,0x6e,0x20,0x69,0x6e,0x20,0x5b,
    0x5b,0x73,0x74,0x61,0x67,0x65,0x5f,0x69,0x6e,0x5d,0x5d,0x29,0x0a,0x7b,0x0a,0x20,
    0x20,0x20,0x20,0x6d,0x61,0x69,0x6e,0x30,0x5f,0x6f,0x75,0x74,0x20,0x6f,0x75,0x74,
    0x20,0x3d,0x20,0x7b,0x7d,0x3b,0x0a,0x20,0x20,0x20,0x20,0x6f,0x75,0x74,0x2e,0x67,
    0x6c,0x5f,0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x20,0x3d,0x20,0x66,0x6c,0x6f,
    0x61,0x74,0x34,0x28,0x69,0x6e,0x2e,0x70,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x2c,
    0x20,0x31,0x2e,0x30,0x29,0x3b,0x0a,0x20,0x20,0x20,0x20,0x6f,0x75,0x74,0x2e,0x74,
    0x65,0x78,0x63,0x6f,0x6f,0x72,0x64,0x20,0x3d,0x20,0x69,0x6e,0x2e,0x74,0x65,0x78,
    0x63,0x6f,0x6f,0x72,0x64,0x30,0x3b,0x0a,0x20,0x20,0x20,0x20,0x72,0x65,0x74,0x75,
    0x72,0x6e,0x20,0x6f,0x75,0x74,0x3b,0x0a,0x7d,0x0a,0x0a,0x00,
};
/*
    #include <metal_stdlib>
    #include <simd/simd.h>
    
    using namespace metal;
    
    struct fs_params_cel
    {
        float2 iResolution;
        int cell_shading;
    };
    
    struct main0_out
    {
        float4 frag_color [[color(0)]];
    };
    
    struct main0_in
    {
        float2 texcoord [[user(locn0)]];
    };
    
    fragment main0_out main0(main0_in in [[stage_in]], constant fs_params_cel& _31 [[buffer(0)]], texture2d<float> tex [[texture(0)]], sampler smp [[sampler(0)]])
    {
        main0_out out = {};
        float4 color = tex.sample(smp, in.texcoord);
        if (_31.cell_shading == 1)
        {
            float4 _83 = color;
            float _53 = (_83.x + _83.y) + _83.z;
            float3 _73 = _83.xyz / float3((_53 * 0.3333333432674407958984375) / (floor(_53 * 5.333333492279052734375) * 0.0625));
            float4 _87 = _83;
            _87.x = _73.x;
            _87.y = _73.y;
            _87.z = _73.z;
            color = _87;
        }
        out.frag_color = color;
        return out;
    }
    
*/
static const char fs_cel_source_metal_macos[903] = {
    0x23,0x69,0x6e,0x63,0x6c,0x75,0x64,0x65,0x20,0x3c,0x6d,0x65,0x74,0x61,0x6c,0x5f,
    0x73,0x74,0x64,0x6c,0x69,0x62,0x3e,0x0a,0x23,0x69,0x6e,0x63,0x6c,0x75,0x64,0x65,
    0x20,0x3c,0x73,0x69,0x6d,0x64,0x2f,0x73,0x69,0x6d,0x64,0x2e,0x68,0x3e,0x0a,0x0a,
    0x75,0x73,0x69,0x6e,0x67,0x20,0x6e,0x61,0x6d,0x65,0x73,0x70,0x61,0x63,0x65,0x20,
    0x6d,0x65,0x74,0x61,0x6c,0x3b,0x0a,0x0a,0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x66,
    0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5f,0x63,0x65,0x6c,0x0a,0x7b,0x0a,0x20,
    0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,0x69,0x52,0x65,0x73,0x6f,0x6c,
    0x75,0x74,0x69,0x6f,0x6e,0x3b,0x0a,0x20,0x20,0x20,0x20,0x69,0x6e,0x74,0x20,0x63,
    0x65,0x6c,0x6c,0x5f,0x73,0x68,0x61,0x64,0x69,0x6e,0x67,0x3b,0x0a,0x7d,0x3b,0x0a,
    0x0a,0x73,0x74,0x72,0x75,0x63,0x74,0x20,0x6d,0x61,0x69,0x6e,0x30,0x5f,0x6f,0x75,
    0x74,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x20,0x66,
    0x72,0x61,0x67,0x5f,0x63,0x6f,0x6c,0x6f,0x72,0x20,0x5b,0x5b,0x63,0x6f,0x6c,0x6f,
    0x72,0x28,0x30,0x29,0x5d,0x5d,0x3b,0x0a,0x7d,0x3b,0x0a,0x0a,0x73,0x74,0x72,0x75,
    0x63,0x74,0x20,0x6d,0x61,0x69,0x6e,0x30,0x5f,0x69,0x6e,0x0a,0x7b,0x0a,0x20,0x20,
    0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x32,0x20,0x74,0x65,0x78,0x63,0x6f,0x6f,0x72,
    0x64,0x20,0x5b,0x5b,0x75,0x73,0x65,0x72,0x28,0x6c,0x6f,0x63,0x6e,0x30,0x29,0x5d,
    0x5d,0x3b,0x0a,0x7d,0x3b,0x0a,0x0a,0x66,0x72,0x61,0x67,0x6d,0x65,0x6e,0x74,0x20,
    0x6d,0x61,0x69,0x6e,0x30,0x5f,0x6f,0x75,0x74,0x20,0x6d,0x61,0x69,0x6e,0x30,0x28,
    0x6d,0x61,0x69,0x6e,0x30,0x5f,0x69,0x6e,0x20,0x69,0x6e,0x20,0x5b,0x5b,0x73,0x74,
    0x61,0x67,0x65,0x5f,0x69,0x6e,0x5d,0x5d,0x2c,0x20,0x63,0x6f,0x6e,0x73,0x74,0x61,
    0x6e,0x74,0x20,0x66,0x73,0x5f,0x70,0x61,0x72,0x61,0x6d,0x73,0x5f,0x63,0x65,0x6c,
    0x26,0x20,0x5f,0x33,0x31,0x20,0x5b,0x5b,0x62,0x75,0x66,0x66,0x65,0x72,0x28,0x30,
    0x29,0x5d,0x5d,0x2c,0x20,0x74,0x65,0x78,0x74,0x75,0x72,0x65,0x32,0x64,0x3c,0x66,
    0x6c,0x6f,0x61,0x74,0x3e,0x20,0x74,0x65,0x78,0x20,0x5b,0x5b,0x74,0x65,0x78,0x74,
    0x75,0x72,0x65,0x28,0x30,0x29,0x5d,0x5d,0x2c,0x20,0x73,0x61,0x6d,0x70,0x6c,0x65,
    0x72,0x20,0x73,0x6d,0x70,0x20,0x5b,0x5b,0x73,0x61,0x6d,0x70,0x6c,0x65,0x72,0x28,
    0x30,0x29,0x5d,0x5d,0x29,0x0a,0x7b,0x0a,0x20,0x20,0x20,0x20,0x6d,0x61,0x69,0x6e,
    0x30,0x5f,0x6f,0x75,0x74,0x20,0x6f,0x75,0x74,0x20,0x3d,0x20,0x7b,0x7d,0x3b,0x0a,
    0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x20,0x63,0x6f,0x6c,0x6f,0x72,
    0x20,0x3d,0x20,0x74,0x65,0x78,0x2e,0x73,0x61,0x6d,0x70,0x6c,0x65,0x28,0x73,0x6d,
    0x70,0x2c,0x20,0x69,0x6e,0x2e,0x74,0x65,0x78,0x63,0x6f,0x6f,0x72,0x64,0x29,0x3b,
    0x0a,0x20,0x20,0x20,0x20,0x69,0x66,0x20,0x28,0x5f,0x33,0x31,0x2e,0x63,0x65,0x6c,
    0x6c,0x5f,0x73,0x68,0x61,0x64,0x69,0x6e,0x67,0x20,0x3d,0x3d,0x20,0x31,0x29,0x0a,
    0x20,0x20,0x20,0x20,0x7b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x66,0x6c,
    0x6f,0x61,0x74,0x34,0x20,0x5f,0x38,0x33,0x20,0x3d,0x20,0x63,0x6f,0x6c,0x6f,0x72,
    0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x20,
    0x5f,0x35,0x33,0x20,0x3d,0x20,0x28,0x5f,0x38,0x33,0x2e,0x78,0x20,0x2b,0x20,0x5f,
    0x38,0x33,0x2e,0x79,0x29,0x20,0x2b,0x20,0x5f,0x38,0x33,0x2e,0x7a,0x3b,0x0a,0x20,
    0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x33,0x20,0x5f,0x37,
    0x33,0x20,0x3d,0x20,0x5f,0x38,0x33,0x2e,0x78,0x79,0x7a,0x20,0x2f,0x20,0x66,0x6c,
    0x6f,0x61,0x74,0x33,0x28,0x28,0x5f,0x35,0x33,0x20,0x2a,0x20,0x30,0x2e,0x33,0x33,
    0x33,0x33,0x33,0x33,0x33,0x34,0x33,0x32,0x36,0x37,0x34,0x34,0x30,0x37,0x39,0x35,
    0x38,0x39,0x38,0x34,0x33,0x37,0x35,0x29,0x20,0x2f,0x20,0x28,0x66,0x6c,0x6f,0x6f,
    0x72,0x28,0x5f,0x35,0x33,0x20,0x2a,0x20,0x35,0x2e,0x33,0x33,0x33,0x33,0x33,0x33,
    0x34,0x39,0x32,0x32,0x37,0x39,0x30,0x35,0x32,0x37,0x33,0x34,0x33,0x37,0x35,0x29,
    0x20,0x2a,0x20,0x30,0x2e,0x30,0x36,0x32,0x35,0x29,0x29,0x3b,0x0a,0x20,0x20,0x20,
    0x20,0x20,0x20,0x20,0x20,0x66,0x6c,0x6f,0x61,0x74,0x34,0x20,0x5f,0x38,0x37,0x20,
    0x3d,0x20,0x5f,0x38,0x33,0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x5f,
    0x38,0x37,0x2e,0x78,0x20,0x3d,0x20,0x5f,0x37,0x33,0x2e,0x78,0x3b,0x0a,0x20,0x20,
    0x20,0x20,0x20,0x20,0x20,0x20,0x5f,0x38,0x37,0x2e,0x79,0x20,0x3d,0x20,0x5f,0x37,
    0x33,0x2e,0x79,0x3b,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x5f,0x38,0x37,
    0x2e,0x7a,0x20,0x3d,0x20,0x5f,0x37,0x33,0x2e,0x7a,0x3b,0x0a,0x20,0x20,0x20,0x20,
    0x20,0x20,0x20,0x20,0x63,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,0x5f,0x38,0x37,0x3b,
    0x0a,0x20,0x20,0x20,0x20,0x7d,0x0a,0x20,0x20,0x20,0x20,0x6f,0x75,0x74,0x2e,0x66,
    0x72,0x61,0x67,0x5f,0x63,0x6f,0x6c,0x6f,0x72,0x20,0x3d,0x20,0x63,0x6f,0x6c,0x6f,
    0x72,0x3b,0x0a,0x20,0x20,0x20,0x20,0x72,0x65,0x74,0x75,0x72,0x6e,0x20,0x6f,0x75,
    0x74,0x3b,0x0a,0x7d,0x0a,0x0a,0x00,
};
#if !defined(SOKOL_GFX_INCLUDED)
  #error "Please include sokol_gfx.h before cel.glsl.h"
#endif
static inline const sg_shader_desc* cel_shader_desc(sg_backend backend) {
  if (backend == SG_BACKEND_GLCORE33) {
    static sg_shader_desc desc;
    static bool valid;
    if (!valid) {
      valid = true;
      desc.attrs[0].name = "position";
      desc.attrs[1].name = "texcoord0";
      desc.vs.source = vs_cel_source_glsl330;
      desc.vs.entry = "main";
      desc.fs.source = fs_cel_source_glsl330;
      desc.fs.entry = "main";
      desc.fs.uniform_blocks[0].size = 16;
      desc.fs.uniform_blocks[0].layout = SG_UNIFORMLAYOUT_STD140;
      desc.fs.uniform_blocks[0].uniforms[0].name = "_31.iResolution";
      desc.fs.uniform_blocks[0].uniforms[0].type = SG_UNIFORMTYPE_FLOAT2;
      desc.fs.uniform_blocks[0].uniforms[0].array_count = 1;
      desc.fs.uniform_blocks[0].uniforms[1].name = "_31.cell_shading";
      desc.fs.uniform_blocks[0].uniforms[1].type = SG_UNIFORMTYPE_INT;
      desc.fs.uniform_blocks[0].uniforms[1].array_count = 1;
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
      desc.label = "cel_shader";
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
      desc.vs.source = vs_cel_source_hlsl4;
      desc.vs.d3d11_target = "vs_4_0";
      desc.vs.entry = "main";
      desc.fs.source = fs_cel_source_hlsl4;
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
      desc.label = "cel_shader";
    }
    return &desc;
  }
  if (backend == SG_BACKEND_METAL_MACOS) {
    static sg_shader_desc desc;
    static bool valid;
    if (!valid) {
      valid = true;
      desc.vs.source = vs_cel_source_metal_macos;
      desc.vs.entry = "main0";
      desc.fs.source = fs_cel_source_metal_macos;
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
      desc.label = "cel_shader";
    }
    return &desc;
  }
  return 0;
}