// For wasm i think turn off?
#define HANDMADE_MATH_NO_SSE
#define HANDMADE_MATH_USE_DEGREES
#define STB_IMAGE_IMPLEMENTATION

#define DEBUG 1

// #define DISABLE_GUI

// #define CPU_SKIN

#define SOKOL_IMPL
#include <HandmadeMath.h>
#include "common.h"
#include <sokol/sokol_glue.h>
#include "sokol.h"

#include <stb_image.h>

#ifdef CPU_SKIN
#include "shaders/bonemesh_cpu.glsl.h"
#else
#include "shaders/bonemesh.glsl.h"
#endif
#include "shaders/cel.glsl.h"

#include "stdio.h"
#include "arena.h"
#include <sokol/sokol_glue.h>
#include <sokol/sokol_time.h>
#define CGLTF_IMPLEMENTATION
#include <cgltf.h>
#include "array.h"

#ifndef DISABLE_GUI
#define _CRT_SECURE_NO_WARNINGS (1)
// include nuklear.h before the sokol_nuklear.h implementation
#define NK_IMPLEMENTATION
#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_STANDARD_BOOL
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#define NK_INCLUDE_STANDARD_VARARGS
#include <nuklear.h>
#define SOKOL_NUKLEAR_IMPL
#include <sokol/util/sokol_nuklear.h>
#endif // DEBUG

#if defined(SOKOL_GLCORE33) || defined(SOKOL_GLES2) || defined(SOKOL_GLES3)
#define USING_OPENGL_BACKEND
#else
#endif

#if defined(USING_OPENGL_BACKEND)
#else
#define LEFT_HANDED
#endif

// Coordinate-system-handedness generic
static inline HMM_Quat HMM_M4ToQ(HMM_Mat4 M) {
#ifdef LEFT_HANDED
  return HMM_M4ToQ_LH(M);
#else
  return HMM_M4ToQ_RH(M);
#endif
}

static inline HMM_Mat4 HMM_Orthographic(float Left, float Right, float Bottom,
                                        float Top, float Near, float Far) {
#ifdef LEFT_HANDED
  return HMM_Orthographic_LH_ZO(Left, Right, Bottom, Top, Near, Far);
#else
  return HMM_Orthographic_RH_NO(Left, Right, Bottom, Top, Near, Far);
#endif
}

static inline HMM_Mat4 HMM_Perspective(float FOV, float AspectRatio, float Near,
                                       float Far) {
#ifdef LEFT_HANDED
  // WHY DOES THIS WORK
  // return HMM_Perspective_LH_ZO(-FOV, AspectRatio, Near, Far);
  return HMM_Perspective_LH_ZO(FOV, AspectRatio, Near, Far);
#else
  return HMM_Perspective_RH_NO(FOV, AspectRatio, Near, Far);
#endif
}

static inline HMM_Mat4 HMM_Rotate(float Angle, HMM_Vec3 Axis) {
#ifdef LEFT_HANDED
  return HMM_Rotate_LH(Angle, Axis);
#else
  return HMM_Rotate_RH(Angle, Axis);
#endif
}

static inline HMM_Mat4 HMM_LookAt(HMM_Vec3 Eye, HMM_Vec3 Center, HMM_Vec3 Up) {
#ifdef LEFT_HANDED
  return HMM_LookAt_LH(Eye, Center, Up);
#else
  return HMM_LookAt_RH(Eye, Center, Up);
#endif
}

ASSUME_NONNULL_BEGIN

#define QUAT_EPSILON 0.000001f
#define VEC3_EPSILON 0.000001f

#define SG_RANGE_ARR(T, a)                                                     \
  (sg_range) { .ptr = (a).ptr, .size = sizeof(T) * (a).len }

typedef HMM_Vec2 vec2;
typedef HMM_Vec3 vec3;
typedef HMM_Vec4 vec4;
typedef HMM_Quat quat;
typedef HMM_Mat3 mat3;
typedef HMM_Mat4 mat4;

static inline float interpolate_scalar(float a, float b, float t);
static inline vec3 interpolate_vec3(vec3 a, vec3 b, float t);
static inline quat interpolate_quat(quat a, quat b, float t);

typedef array_type(mat4) mat4array_t;
typedef array_type(float) floatarray_t;
typedef array_type(u32) u32array_t;

typedef array_type(const char *) strarray_t;

static char **JOINT_NAMES = NULL;

int cgltf_get_node_index(cgltf_node *target, cgltf_node *nodes, u32 nodes_len);
int cgltf_get_node_index_by_name(const char *name, cgltf_node *nodes,
                                 cgltf_size nodes_len);
void cgltf_get_scalar_values(floatarray_t *out, u32 comp_count,
                             const cgltf_accessor *in_accessor);
char **cgltf_load_joint_names(cgltf_data *data);

const mat4 OPENGL_TO_METAL_MATRIX = (mat4){
    .Elements = {{1.0f, 0.0f, 0.0f, 0.0f},
                 {0.0f, 1.0f, 0.0f, 0.0f},
                 {0.0f, 0.0f, 0.5f, 0.0f},
                 {0.0f, 0.0f, 0.5f, 1.0f}},
};

typedef struct {
  vec3 pos;
  vec3 norm;
  vec2 texcord;
  float weights[4];
  int influences[4];
} Vertex;

typedef union {
  struct {
    u8 x, y, z, w;
  };

  u8 v[4];

} u8vec4;

#define M4V4D(m, mRow, x, y, z, w)                                             \
  x *m.Elements[0][mRow] + y *m.Elements[1][mRow] + z *m.Elements[2][mRow] +   \
      w *m.Elements[3][mRow]

vec3 mat4_mul_v3(mat4 m, vec3 v) {
  return HMM_V3(M4V4D(m, 0, v.X, v.Y, v.Z, 0.0f),
                M4V4D(m, 1, v.X, v.Y, v.Z, 0.0f),
                M4V4D(m, 2, v.X, v.Y, v.Z, 0.0f));
}
vec3 mat4_mul_p(mat4 m, vec3 v) {
  return HMM_V3(M4V4D(m, 0, v.X, v.Y, v.Z, 1.0f),
                M4V4D(m, 1, v.X, v.Y, v.Z, 1.0f),
                M4V4D(m, 2, v.X, v.Y, v.Z, 1.0f));
}

const HMM_Mat4 MAT4_CONVERT_HAND = {.Elements = {{1.0f, 0.0f, 0.0f, 0.0f},
                                                 {0.0f, 1.0f, 0.0f, 0.0f},
                                                 {0.0f, 0.0f, -1.0f, 0.0f},
                                                 {0.0f, 0.0f, 0.0f, 1.0f}}};

static inline vec3 vec3_convert_handedness(bool is_point, vec3 in) {
#ifdef LEFT_HANDED
  if (is_point)
    return mat4_mul_p(MAT4_CONVERT_HAND, in);
  return mat4_mul_v3(MAT4_CONVERT_HAND, in);

  return in;
#endif
  return in;
}

static inline vec3 vec3_new(float a, float b, float c) {
  return (vec3){a, b, c};
}
static inline vec3 vec3_default() { return (vec3){0, 0, 0}; }
static inline vec4 vec4_default() { return (vec4){0, 0, 0, 0}; }

static inline mat4 mat4_default() {
  return (mat4){.Elements = {{1.0f, 0.0f, 0.0f, 0.0f},
                             {0.0f, 1.0f, 0.0f, 0.0f},
                             {0.0f, 0.0f, 1.0f, 0.0f},
                             {0.0f, 0.0f, 0.0f, 1.0f}}};
}
mat4 mat4_new(float _00, float _01, float _02, float _03, float _10, float _11,
              float _12, float _13, float _20, float _21, float _22, float _23,
              float _30, float _31, float _32, float _33) {
  return (mat4){.Elements = {{_00, _01, _02, _03},
                             {_10, _11, _12, _13},
                             {_20, _21, _22, _23},
                             {_30, _31, _32, _33}}};
}

mat4 mat4_new_from_arr(float *arr) {
  return mat4_new(arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6],
                  arr[7], arr[8], arr[9], arr[10], arr[11], arr[12], arr[13],
                  arr[14], arr[15]);
}

void convertRHtoLH(float mat[4][4]) {
  // Swap rows 2 and 3
  for (int i = 0; i < 4; ++i) {
    float temp = mat[1][i];
    mat[1][i] = mat[2][i];
    mat[2][i] = temp;
  }

  // Swap columns 2 and 3
  for (int i = 0; i < 4; ++i) {
    float temp = mat[i][1];
    mat[i][1] = mat[i][2];
    mat[i][2] = temp;
  }

  // Negate all values in row 3
  for (int i = 0; i < 4; ++i) {
    mat[2][i] = -mat[2][i];
  }

  // Negate all values in column 3
  for (int i = 0; i < 4; ++i) {
    mat[i][2] = -mat[i][2];
  }
}

static inline quat quat_new(float X, float Y, float Z, float W) {
  return HMM_Q(X, Y, Z, W);
}

#if 0
vec3 quat_mul_v3(quat q, vec3 v) {
  // 	return    q.vector * 2.0f * dot(q.vector, v) +
  //   v *(q.scalar * q.scalar - dot(q.vector, q.vector)) +
  //       cross(q.vector, v) * 2.0f * q.scalar;
  vec3 out;

  // out = HMM_MulV3F(HMM_MulV3F(q.XYZ, 2.0), HMM_DotV3(q.XYZ, v));
  // out = HMM_AddV3(out, HMM_MulV3F(v, q.W * q.W - HMM_DotV3(q.XYZ, q.XYZ)));
  // out = HMM_AddV3(out, HMM_MulV3F(HMM_MulV3F(HMM_Cross(q.XYZ, v), 2.0),
  // q.W));

  // t = 2.0f * dot(q.vector, v)
  float t = 2.0f * HMM_DotV3(q.XYZ, v);

  // p = v * (q.scalar^2 - dot(q.vector, q.vector))
  vec3 p = HMM_MulV3F(v, q.W * q.W - HMM_DotV3(q.XYZ, q.XYZ));

  // r = 2.0f * q.scalar * cross(q.vector, v)
  vec3 r = HMM_MulV3F(HMM_Cross(q.XYZ, v), 2.0f * q.W);

  // out = t * q.vector + p + r
  out = HMM_AddV3(HMM_AddV3(HMM_MulV3F(q.XYZ, t), p), r);

  return out;
}
#else
HMM_Vec3 quat_mul_v3(quat q, HMM_Vec3 v) {
  float dotVector = HMM_DotV3(q.XYZ, v);
  HMM_Vec3 term1 = HMM_MulV3F(q.XYZ, 2.0f * dotVector);
  HMM_Vec3 term2 = HMM_MulV3F(v, (q.W * q.W - HMM_DotV3(q.XYZ, q.XYZ)));
  HMM_Vec3 term3 = HMM_MulV3F(HMM_Cross(q.XYZ, v), 2.0f * q.W);

  HMM_Vec3 result = HMM_AddV3(HMM_AddV3(term1, term2), term3);
  return result;
}
// vec3 quat_mul_v3(quat q, vec3 v) {
//   // 	return    q.vector * 2.0f * dot(q.vector, v) +
//   //   v *(q.scalar * q.scalar - dot(q.vector, q.vector)) +
//   //       cross(q.vector, v) * 2.0f * q.scalar;
//   vec3 out;

//   out = HMM_MulV3F(HMM_MulV3F(q.XYZ, 2.0), HMM_DotV3(q.XYZ, v));
//   out = HMM_AddV3(out, HMM_MulV3F(v, q.W * q.W - HMM_DotV3(q.XYZ, q.XYZ)));
//   out = HMM_AddV3(out, HMM_MulV3F(HMM_MulV3F(HMM_Cross(q.XYZ, v), 2.0),
//   q.W)); return out;
// }
#endif

#define NO_HANDMADE_QUAT_IMPL
#ifdef NO_HANDMADE_QUAT_IMPL
quat quat_default() { return HMM_Q(0, 0, 0, 1); }

float quat_dot(quat a, quat b) {
  return a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W;
}

quat quat_mul_quat(quat Q1, quat Q2) {
  return quat_new(Q2.X * Q1.W + Q2.Y * Q1.Z - Q2.Z * Q1.Y + Q2.W * Q1.X,
                  -Q2.X * Q1.Z + Q2.Y * Q1.W + Q2.Z * Q1.X + Q2.W * Q1.Y,
                  Q2.X * Q1.Y - Q2.Y * Q1.X + Q2.Z * Q1.W + Q2.W * Q1.Z,
                  -Q2.X * Q1.X - Q2.Y * Q1.Y - Q2.Z * Q1.Z + Q2.W * Q1.W);
}

quat quat_norm(quat q) {
  float lenSq = q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W;
  if (lenSq < QUAT_EPSILON) {
    return q;
  }
  float i_len = 1.0f / sqrtf(lenSq);

  q.X *= i_len;
  q.Y *= i_len;
  q.Z *= i_len;
  q.W *= i_len;
  return q;
}

quat quat_inverse(quat q) {
  float lenSq = q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W;
  if (lenSq < QUAT_EPSILON) {
    return quat_default();
  }
  float recip = 1.0f / lenSq;

  // conjugate / norm
  return HMM_Q(-q.X * recip, -q.Y * recip, -q.Z * recip, q.W * recip);
}

quat quat_from_to(vec3 from, vec3 to) {
  HMM_Vec3 f = HMM_NormV3(from);
  HMM_Vec3 t = HMM_NormV3(to);

  if (HMM_LenSqrV3(HMM_SubV3(f, t)) < 1e-6) {
    return HMM_Q(0.0f, 0.0f, 0.0f, 1.0f);
  } else if (HMM_LenSqrV3(HMM_SubV3(f, HMM_MulV3F(t, -1.0f))) < 1e-6) {
    HMM_Vec3 ortho = HMM_V3(1.0f, 0.0f, 0.0f);

    if (fabsf(f.Y) < fabsf(f.X)) {
      ortho = HMM_V3(0.0f, 1.0f, 0.0f);
    }
    if (fabsf(f.Z) < fabs(f.Y) && fabs(f.Z) < fabsf(f.X)) {
      ortho = HMM_V3(0.0f, 0.0f, 1.0f);
    }

    HMM_Vec3 axis = HMM_NormV3(HMM_Cross(f, ortho));
    return HMM_Q(axis.X, axis.Y, axis.Z, 0.0f);
  }

  HMM_Vec3 half = HMM_NormV3(HMM_AddV3(f, t));
  HMM_Vec3 axis = HMM_Cross(f, half);

  return HMM_Q(axis.X, axis.Y, axis.Z, HMM_DotV3(f, half));
}

quat quat_look_rotation(vec3 dir, vec3 up) {
  // Normalize the input vectors
  HMM_Vec3 f = HMM_NormV3(dir);
  HMM_Vec3 u = HMM_NormV3(up);

  // Find orthonormal basis vectors
  HMM_Vec3 r = HMM_Cross(u, f);
  u = HMM_Cross(f, r);

  // From world forward to object forward
  HMM_Quat f2d = quat_from_to(HMM_V3(0, 0, 1), f);

  // What direction is the new object up?
  HMM_Vec3 objectUp = quat_mul_v3(f2d, HMM_V3(0, 1, 0));

  // From object up to desired up
  HMM_Quat u2u = quat_from_to(objectUp, u);

  // Rotate to forward direction first, then twist to correct up
  HMM_Quat result = quat_mul_quat(f2d, u2u);

  // Don't forget to normalize the result
  result = HMM_NormQ(result);

  return result;
}
mat4 quat_to_mat4(quat q) {
  // For some reason this causes a flickering bug on metal, but it's not
  // necessary, but leaving it here #ifdef LEFT_HANDED return (mat4){
  //     .Elements = {
  //         {1 - 2 * q.Y * q.Y - 2 * q.Z * q.Z, 2 * q.X * q.Y + 2 * q.W * q.Z,
  //          2 * q.X * q.Z - 2 * q.W * q.Y, 0},
  //         {2 * q.X * q.Y - 2 * q.W * q.Z, 1 - 2 * q.X * q.X - 2 * q.Z * q.Z,
  //          2 * q.Y * q.Z + 2 * q.W * q.X, 0},
  //         {2 * q.X * q.Z + 2 * q.W * q.Y, 2 * q.Y * q.Z - 2 * q.W * q.X,
  //          1 - 2 * q.X * q.X - 2 * q.Y * q.Y, 0},
  //         {0, 0, 0, 1}}};
  // #else
  vec3 r = quat_mul_v3(q, HMM_V3(1, 0, 0));
  vec3 u = quat_mul_v3(q, HMM_V3(0, 1, 0));
  vec3 f = quat_mul_v3(q, HMM_V3(0, 0, 1));

  return mat4_new(r.X, r.Y, r.Z, 0, u.X, u.Y, u.Z, 0, f.X, f.Y, f.Z, 0, 0, 0, 0,
                  1);
  // #endif
}
quat quat_from_mat4_rh(mat4 m) {
  vec3 up = HMM_NormV3(HMM_V3(m.Columns[1].X, m.Columns[1].Y, m.Columns[1].Z));
  vec3 forward =
      HMM_NormV3(HMM_V3(m.Columns[2].X, m.Columns[2].Y, m.Columns[2].Z));
  vec3 right = HMM_Cross(up, forward);
  up = HMM_Cross(forward, right);

  return quat_look_rotation(forward, up);
}
#else

quat quat_mul_quat(quat Q1, quat Q2) { return HMM_MulQ(Q2, Q1); }
quat quat_inverse(quat q) { return HMM_InvQ(q); }

mat4 quat_to_mat4(quat q) { return HMM_QToM4(q); }
// quat quat_from_mat4_rh(mat4 m) { return HMM_M4ToQ_RH(m); }

quat quat_from_to(vec3 from, vec3 to) {
  HMM_Vec3 f = HMM_NormV3(from);
  HMM_Vec3 t = HMM_NormV3(to);

  if (HMM_LenSqrV3(HMM_SubV3(f, t)) < 1e-6) {
    return HMM_Q(0.0f, 0.0f, 0.0f, 1.0f);
  } else if (HMM_LenSqrV3(HMM_SubV3(f, HMM_MulV3F(t, -1.0f))) < 1e-6) {
    HMM_Vec3 ortho = HMM_V3(1.0f, 0.0f, 0.0f);

    if (fabsf(f.Y) < fabsf(f.X)) {
      ortho = HMM_V3(0.0f, 1.0f, 0.0f);
    }
    if (fabsf(f.Z) < fabs(f.Y) && fabs(f.Z) < fabsf(f.X)) {
      ortho = HMM_V3(0.0f, 0.0f, 1.0f);
    }

    HMM_Vec3 axis = HMM_NormV3(HMM_Cross(f, ortho));
    return HMM_Q(axis.X, axis.Y, axis.Z, 0.0f);
  }

  HMM_Vec3 half = HMM_NormV3(HMM_AddV3(f, t));
  HMM_Vec3 axis = HMM_Cross(f, half);

  return HMM_Q(axis.X, axis.Y, axis.Z, HMM_DotV3(f, half));
}

quat quat_look_rotation(vec3 dir, vec3 up) {
  // Normalize the input vectors
  HMM_Vec3 f = HMM_NormV3(dir);
  HMM_Vec3 u = HMM_NormV3(up);

  // Find orthonormal basis vectors
  HMM_Vec3 r = HMM_Cross(u, f);
  u = HMM_Cross(f, r);

  // From world forward to object forward
  HMM_Quat f2d = quat_from_to(HMM_V3(0, 0, 1), f);

  // What direction is the new object up?
  HMM_Vec3 objectUp = quat_mul_v3(f2d, HMM_V3(0, 1, 0));

  // From object up to desired up
  HMM_Quat u2u = quat_from_to(objectUp, u);

  // Rotate to forward direction first, then twist to correct up
  HMM_Quat result = quat_mul_quat(f2d, u2u);

  // Don't forget to normalize the result
  result = HMM_NormQ(result);

  return result;
}

quat quat_from_mat4_rh(mat4 m) {
  // return HMM_M4ToQ_RH(m);
  vec3 up = HMM_NormV3(HMM_V3(m.Columns[1].X, m.Columns[1].Y, m.Columns[1].Z));
  vec3 forward =
      HMM_NormV3(HMM_V3(m.Columns[2].X, m.Columns[2].Y, m.Columns[2].Z));
  vec3 right = HMM_Cross(up, forward);
  up = HMM_Cross(forward, right);

  return quat_look_rotation(forward, up);
}
#endif

bool quat_eq(quat left, quat right) {
  return (fabsf(left.X - right.X) <= QUAT_EPSILON &&
          fabsf(left.Y - right.Y) <= QUAT_EPSILON &&
          fabsf(left.Z - right.Z) <= QUAT_EPSILON &&
          fabsf(left.W - left.W) <= QUAT_EPSILON);
}

vec4 vec3_to_vec4_point(vec3 p) {
  return (vec4){
      .X = p.X,
      .Y = p.Y,
      .Z = p.Z,
      .W = 1.0,
  };
}

vec4 vec3_to_vec4_vec(vec3 p) {
  return (vec4){
      .X = p.X,
      .Y = p.Y,
      .Z = p.Z,
      .W = 0.0,
  };
}

vec3 vec4_truncate(vec4 p) {
  return (vec3){
      .X = p.X,
      .Y = p.Y,
      .Z = p.Z,
  };
}

// #define TRANSFORM_USE_MATRICES
typedef struct {
#ifdef TRANSFORM_USE_MATRICES
  mat4 m;
#else
  vec3 position;
  quat rotation;
  vec3 scale;
#endif
} Transform;

Transform cgltf_get_local_transform(cgltf_node *n);
// To convert:
// you could also look at the matrix math, in a right handed coordinate system
// you get `posWorld = matToWorld*matAnim*vertModel` but instead you have a
// `matToWorld` in left handed and want the result in left handed. So you need
// a `posWorld = matToWorld*matToLeft*matAnim*vertModel` but you want that
// `matToLeft` to be integrated in both the `matAnim` and `vertModel` to that
// end you can do `matAnimLeftHanded = matToLeft*matAnim*matToRight` and have
// `vertModelLeftHanded = matToLeft*vertModel` and because
// `matToRight*matToLeft
// == matIdentity` that will cancel out and make `posWorld =
// matToWorld*matAnimeftHanded*vertModelLeftHanded` end up as `posWorld =
// matToWorld*matToLeft*matAnim*vertModel` from:
// https://discord.com/channels/239737791225790464/1084530676059082832/1084537739405434892
// `matToLeft` and `matToRight` are MAT4_CONVERT_HAND
// This function should be called whenever we create a transform from glTF
// values, this is because glTF uses a right handed coordinate system.
void transform_convert_handedness(Transform *t);

Transform transform_mix(const Transform *a, const Transform *b, float t) {
  quat brot = b->rotation;
  if (flte_zero(quat_dot(a->rotation, brot))) {
    brot = HMM_MulQF(brot, -1);
  }

  return (Transform){
      .position = interpolate_vec3(a->position, b->position, t),
      .rotation = interpolate_quat(a->rotation, brot, t),
      .scale = interpolate_vec3(a->scale, b->scale, t),
  };
}

void transform_convert_handedness(Transform *t) {
#ifdef LEFT_HANDED
#ifdef TRANSFORM_USE_MATRICES
  t->m = HMM_MulM4(MAT4_CONVERT_HAND, HMM_MulM4(t->m, MAT4_CONVERT_HAND));
#else
  mat4 m = quat_to_mat4(t->rotation);
  m = HMM_MulM4(MAT4_CONVERT_HAND, HMM_MulM4(m, MAT4_CONVERT_HAND));

  t->rotation = quat_from_mat4_rh(m);
  t->position.Z *= -1;

#endif
#else
  // Assuming it's already in a right-handed coordinate system, so do nothing.
#endif
}

Transform transform_default() {
  Transform out;
#ifdef TRANSFORM_USE_MATRICES
  out.m = HMM_M4D(1.0);
#else
  out.position = HMM_V3(0, 0, 0);
  out.rotation = quat_new(0, 0, 0, 1);
  out.scale = HMM_V3(1, 1, 1);
#endif
  return out;
}

bool transform_print(Transform *t, const char *name, bool print_non_nan) {
  if (isnan(t->position.X) || isnan(t->position.Y) || isnan(t->position.Z) ||
      isnan(t->rotation.X) || isnan(t->rotation.Y) || isnan(t->rotation.Z) ||
      isnan(t->rotation.W) || isnan(t->scale.X) || isnan(t->scale.Y) ||
      isnan(t->scale.Z) || isinf(t->position.X) || isinf(t->position.Y) ||
      isinf(t->position.Z) || isinf(t->rotation.X) || isinf(t->rotation.Y) ||
      isinf(t->rotation.Z) || isinf(t->rotation.W) || isinf(t->scale.X) ||
      isinf(t->scale.Y) || isinf(t->scale.Z)) {

#ifdef TR_PRINT_DEFAULT
    t->position = HMM_V3(0, 0, 0);
    t->rotation = quat_new(0, 0, 0, 1);
    t->scale = HMM_V3(1, 1, 1);
#endif

    printf("(%s) One or more transform values contain NaN\n", name);
    printf("Transform {\n");
    printf("  position: vec3(%f, %f, %f),\n", t->position.X, t->position.Y,
           t->position.Z);
    printf("  scale: vec3(%f, %f, %f),\n", t->scale.X, t->scale.Y, t->scale.Z);
    printf("  rotation: quat(%f, %f, %f, %f),\n", t->rotation.X, t->rotation.Y,
           t->rotation.Z, t->rotation.W);
    printf("}\n");
    return true;
  }
  if (print_non_nan) {

    printf("(%s) No NaNs!\n", name);
    printf("Transform {\n");
    printf("  position: vec3(%f, %f, %f),\n", t->position.X, t->position.Y,
           t->position.Z);
    printf("  scale: vec3(%f, %f, %f),\n", t->scale.X, t->scale.Y, t->scale.Z);
    printf("  rotation: quat(%f, %f, %f, %f),\n", t->rotation.X, t->rotation.Y,
           t->rotation.Z, t->rotation.W);
    printf("}\n");
  }
  return false;
}

// Order is right -> left
Transform transform_combine(Transform a, Transform b) {
#ifdef TRANSFORM_USE_MATRICES
  return (Transform){.m = HMM_MulM4(a.m, b.m)};
#else
  Transform out = transform_default();
  out.scale = HMM_MulV3(a.scale, b.scale);
  out.rotation = quat_mul_quat(b.rotation, a.rotation);
  out.position = quat_mul_v3(a.rotation, HMM_MulV3(a.scale, b.position));
  out.position = HMM_AddV3(a.position, out.position);
  return out;
#endif
}

Transform transform_inverse(Transform t) {
#ifdef TRANSFORM_USE_MATRICES
  return (Transform){.m = HMM_InvGeneralM4(t.m)};
#else
  Transform inv = transform_default();

  inv.rotation = quat_inverse(t.rotation);

  inv.scale.X = fabs(t.scale.X) < VEC3_EPSILON ? 0.0f : 1.0f / t.scale.X;
  inv.scale.Y = fabs(t.scale.Y) < VEC3_EPSILON ? 0.0f : 1.0f / t.scale.Y;
  inv.scale.Z = fabs(t.scale.Z) < VEC3_EPSILON ? 0.0f : 1.0f / t.scale.Z;

  vec3 inv_translation = HMM_MulV3F(t.position, -1.0f);
  inv.position =
      quat_mul_v3(inv.rotation, HMM_MulV3(inv.scale, inv_translation));

  return inv;
#endif
}

// bool transform_eq(const Transform *a, const Transform *b) {
//   return HMM_EqV3(a->position, b->position) && HMM_EqV3(a->scale, b->scale)
//   &&
//          quat_eq(a->rotation, b->rotation);
// }

vec3 transform_point(const Transform *t, vec3 p) {
#ifdef TRANSFORM_USE_MATRICES
  return mat4_mul_p(t->m, p);
#else
  vec3 out = vec3_default();
  out = quat_mul_v3(t->rotation, HMM_MulV3(t->scale, p));
  out = HMM_AddV3(t->position, out);
  return out;
#endif
}

vec3 transform_vector(const Transform *t, vec3 v) {
#ifdef TRANSFORM_USE_MATRICES
  return mat4_mul_v3(t->m, v);
#else
  vec3 out = vec3_default();
  out = quat_mul_v3(t->rotation, HMM_MulV3(t->scale, v));
  return out;
#endif
}

Transform transform_from_mat4_rh(const mat4 *m) {
#ifdef TRANSFORM_USE_MATRICES
  return (Transform){.m = *m};
#else
  Transform out = transform_default();

  out.position = HMM_V3(m->Columns[3].X, m->Columns[3].Y, m->Columns[3].Z);
  // out.rotation = HMM_M4ToQ_RH(*m);
  out.rotation = quat_from_mat4_rh(*m);

  mat4 rot_scale_mat =
      mat4_new(m->Columns[0].X, m->Columns[0].Y, m->Columns[0].Z, 0, // col 1
               m->Columns[1].X, m->Columns[1].Y, m->Columns[1].Z, 0, //
               m->Columns[2].X, m->Columns[2].Y, m->Columns[2].Z, 0, //
               0, 0, 0, 1);

  mat4 inv_rot_mat = quat_to_mat4(quat_inverse(out.rotation));
  mat4 scale_skew_mat = HMM_MulM4(rot_scale_mat, inv_rot_mat);

  out.scale = (vec3){
      scale_skew_mat.Columns[0].X,
      scale_skew_mat.Columns[1].Y,
      scale_skew_mat.Columns[2].Z,
  };

  return out;
#endif
}

mat4 transform_to_mat4(Transform t) {
#ifdef TRANSFORM_USE_MATRICES
  return t.m;
#else
  mat4 out;
  out = HMM_Scale(t.scale);
  out = HMM_MulM4(quat_to_mat4(t.rotation), out);
  out = HMM_MulM4(HMM_Translate(t.position), out);
  return out;
#endif
}

typedef enum {
  INTERP_CONSTANT,
  INTERP_LINEAR,
  INTERP_CUBIC,
} Interpolation;

typedef enum {
  FRAME_SCALAR,
  FRAME_VEC3,
  FRAME_QUAT,
} FrameType;

u8 frame_type_float_count(FrameType ft) {
  switch (ft) {
  case FRAME_SCALAR:
    // return components(float, float);
    return 1;
  case FRAME_VEC3:
    // return components(vec3, float);
    return 3;
  case FRAME_QUAT:
    // return components(quat, float);
    return 4;
  }
}

static inline quat quat_mix(quat from, quat to, float t) {
  return HMM_AddQ(HMM_MulQF(from, 1.0f - t), HMM_MulQF(to, t));
}

static inline float interpolate_scalar(float a, float b, float t) {
  return a + (b - a) * t;
}

static inline vec3 interpolate_vec3(vec3 a, vec3 b, float t) {
  return vec3_new(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t,
                  a.Z + (b.Z - a.Z) * t);
}

static inline quat interpolate_quat(quat a, quat b, float t) {
  quat result = quat_mix(a, b, t);
  if (quat_dot(a, b) < 0) {
    result = quat_mix(a, HMM_MulQF(b, -1), t);
  }
  // result = HMM_NormQ(result);
  result = quat_norm(result);
  return result;
}

static inline float hermite_scalar(float t, float p1, float s1, float _p2,
                                   float s2) {
  float tt = t * t;
  float ttt = tt * t;

  float p2 = _p2;

  float h1 = 2.0f * ttt - 3.0f * tt + 1.0f;
  float h2 = -2.0f * ttt + 3.0f * tt;
  float h3 = ttt - 2.0f * tt + t;
  float h4 = ttt - tt;

  float result = p1 * h1 + p2 * h2 + s1 * h3 + s2 * h4;
  return result;
}

static inline vec3 hermite_vec3(float t, vec3 p1, vec3 s1, vec3 _p2, vec3 s2) {
  float tt = t * t;
  float ttt = tt * t;

  vec3 p2 = _p2;

  float h1 = 2.0f * ttt - 3.0f * tt + 1.0f;
  float h2 = -2.0f * ttt + 3.0f * tt;
  float h3 = ttt - 2.0f * tt + t;
  float h4 = ttt - tt;

  vec3 result = HMM_AddV3(HMM_AddV3(HMM_MulV3F(p1, h1), HMM_MulV3F(p2, h2)),
                          HMM_AddV3(HMM_MulV3F(s1, h3), HMM_MulV3F(s2, h4)));

  return result;
}

static inline void quat_neighborhood(const quat *a, quat *b) {
  if (quat_dot(*a, *b) < 0) {
    *b = quat_inverse(*b);
  }
}

static inline quat hermite_quat(float t, quat p1, quat s1, quat _p2, quat s2) {
  float tt = t * t;
  float ttt = tt * t;

  quat p2 = _p2;
  quat_neighborhood(&p1, &p2);

  float h1 = 2.0f * ttt - 3.0f * tt + 1.0f;
  float h2 = -2.0f * ttt + 3.0f * tt;
  float h3 = ttt - 2.0f * tt + t;
  float h4 = ttt - tt;

  quat result = HMM_AddQ(HMM_AddQ(HMM_MulQF(p1, h1), HMM_MulQF(p2, h2)),
                         HMM_AddQ(HMM_MulQF(s1, h3), HMM_MulQF(s2, h4)));

  return HMM_NormQ(result);
}

#define frame_type(T)                                                          \
  struct {                                                                     \
    FrameHeader header;                                                        \
    T value;                                                                   \
    T in;                                                                      \
    T out;                                                                     \
  }

typedef struct {
  float time;
} FrameHeader;

typedef frame_type(float) scalar_frame_t;
typedef frame_type(vec3) vec3_frame_t;
typedef frame_type(quat) quat_frame_t;
#define frame_value(T, frame_ptr) ((T *)(frame_ptr))->value
#define frame_in(T, frame_ptr) ((T *)(frame_ptr))->in
#define frame_out(T, frame_ptr) ((T *)(frame_ptr))->out
#define frame_time(frame_ptr) ((FrameHeader *)(frame_ptr))->time
#define frame_set_time(frame_ptr, val)                                         \
  ((FrameHeader *)(frame_ptr))->time = (val)

typedef struct {
  slice_t frames;
  Interpolation interpolation;
} track_t;

#define cast_ptr(T, ptr) ((T *)(ptr))

track_t track_new() {
  return (track_t){
      .frames = slice_empty(slice_t),
      .interpolation = INTERP_CONSTANT,
  };
}
float track_start_time(const track_t *trk, FrameType ft);
float track_end_time(const track_t *trk, FrameType ft);
int track_frame_index(const track_t *trk, float time, bool looping,
                      FrameType ft);
void track_sample(const track_t *trk, float time, bool looping, void *out,
                  FrameType ft);
void track_sample_constant(const track_t *trk, float time, bool looping,
                           void *out, FrameType ft);
void track_sample_linear(const track_t *trk, float time, bool looping,
                         void *out, FrameType ft);
void track_sample_cubic(const track_t *trk, float time, bool looping, void *out,
                        FrameType ft);
void track_default_sample(const track_t *trk, void *out, FrameType ft);
float track_time_at_index(const track_t *trk, int idx, FrameType ft);
void track_value_at_index(const track_t *trk, void *out, int idx, FrameType ft);
float track_adjust_time_to_fit(const track_t *trk, float time, bool looping,
                               FrameType ft);

float track_start_time(const track_t *trk, FrameType ft) {
  return track_time_at_index(trk, 0, ft);
}

float track_end_time(const track_t *trk, FrameType ft) {
  return track_time_at_index(trk, trk->frames.len - 1, ft);
}

float track_adjust_time_to_fit(const track_t *trk, float time, bool looping,
                               FrameType ft) {
  usize size = trk->frames.len;
  if (size <= 1)
    return 0.0;

  float start_time = track_time_at_index(trk, 0, ft);
  float end_time = track_time_at_index(trk, size - 1, ft);
  float duration = end_time - start_time;
  if (duration <= 0.0f)
    return 0.0f;

  if (looping) {
    time = fmodf(time - start_time, end_time - start_time);
    if (time < 0.0f) {
      time += end_time - start_time;
    }
    time = time + start_time;
  } else {
    if (time <= track_time_at_index(trk, 0, ft)) {
      time = start_time;
    }
    if (time >= track_time_at_index(trk, size - 1, ft)) {
      time = end_time;
    }
  }

  return time;
}

#define switch_ft(ft, scalar_branch, vec3_branch, quat_branch)                 \
  switch (ft) {                                                                \
  case FRAME_SCALAR: {                                                         \
    scalar_branch;                                                             \
    break;                                                                     \
  }                                                                            \
  case FRAME_VEC3: {                                                           \
    vec3_branch;                                                               \
    break;                                                                     \
  }                                                                            \
  case FRAME_QUAT: {                                                           \
    quat_branch;                                                               \
    break;                                                                     \
  }                                                                            \
  }

#define switch_ft_set_out(ft, out, scalar_branch, vec3_branch, quat_branch)    \
  switch (ft) {                                                                \
  case FRAME_SCALAR: {                                                         \
    *cast_ptr(float, out) = scalar_branch;                                     \
    break;                                                                     \
  }                                                                            \
  case FRAME_VEC3: {                                                           \
    *cast_ptr(vec3, out) = vec3_branch;                                        \
    break;                                                                     \
  }                                                                            \
  case FRAME_QUAT: {                                                           \
    *cast_ptr(quat, out) = quat_branch;                                        \
    break;                                                                     \
  }                                                                            \
  }

#define switch_ft_set_out_block(ft, out, casted_out, scalar_branch,            \
                                vec3_branch, quat_branch)                      \
  switch (ft) {                                                                \
  case FRAME_SCALAR: {                                                         \
    float *casted_out = cast_ptr(float, out);                                  \
    scalar_branch;                                                             \
    break;                                                                     \
  }                                                                            \
  case FRAME_VEC3: {                                                           \
    vec3 *casted_out = cast_ptr(vec3, out);                                    \
    vec3_branch;                                                               \
    break;                                                                     \
  }                                                                            \
  case FRAME_QUAT: {                                                           \
    quat *casted_out = cast_ptr(quat, out);                                    \
    quat_branch;                                                               \
    break;                                                                     \
  }                                                                            \
  }

float track_time_at_index(const track_t *trk, int idx, FrameType ft) {
  switch_ft(
      ft, { return frame_time(array_ref(scalar_frame_t, &trk->frames, idx)); },
      { return frame_time(array_ref(vec3_frame_t, &trk->frames, idx)); },
      { return frame_time(array_ref(quat_frame_t, &trk->frames, idx)); });
}

inline void track_value_at_index(const track_t *trk, void *out, int idx,
                                 FrameType ft) {
  switch_ft_set_out(
      ft, out,
      frame_value(scalar_frame_t, array_ref(scalar_frame_t, &trk->frames, idx)),
      frame_value(vec3_frame_t, array_ref(vec3_frame_t, &trk->frames, idx)),
      frame_value(quat_frame_t, array_ref(quat_frame_t, &trk->frames, idx)));
}

inline void track_default_sample(const track_t *trk, void *out, FrameType ft) {
  switch_ft_set_out(ft, out, 0.0, vec3_default(), quat_default());
}

inline void track_sample(const track_t *trk, float time, bool looping,
                         void *out, FrameType ft) {
  switch (trk->interpolation) {
  case INTERP_CONSTANT: {
    track_sample_constant(trk, time, looping, out, ft);
    break;
  }
  case INTERP_LINEAR: {
    track_sample_linear(trk, time, looping, out, ft);
    // track_sample_constant(trk, time, looping, out, ft);
    break;
  }
  case INTERP_CUBIC: {
    track_sample_cubic(trk, time, looping, out, ft);
    // track_sample_constant(trk, time, looping, out, ft);
    break;
  }
  }
}

void track_sample_constant(const track_t *trk, float time, bool looping,
                           void *out, FrameType ft) {
  int frame = track_frame_index(trk, time, looping, ft);
  if (frame < 0 || frame >= (int)(trk->frames.len - 1)) {
    track_default_sample(trk, out, ft);
    return;
  }

  void *first = &trk->frames.ptr[0];
  switch_ft_set_out(ft, out, frame_value(scalar_frame_t, &first),
                    frame_value(vec3_frame_t, &first),
                    frame_value(quat_frame_t, &first));
}
void track_sample_linear(const track_t *trk, float time, bool looping,
                         void *out, FrameType ft) {
  int frame = track_frame_index(trk, time, looping, ft);
  if (frame < 0 || frame >= (int)(trk->frames.len - 1)) {
    track_default_sample(trk, out, ft);
    return;
  }
  int next_frame = frame + 1;

  float track_time = track_adjust_time_to_fit(trk, time, looping, ft);
  float end_time = track_time_at_index(trk, next_frame, ft);
  float start_time = track_time_at_index(trk, frame, ft);
  float frame_delta = end_time - start_time;
  if (flte_zero(frame_delta))
    return track_default_sample(trk, out, ft);

  float t = (track_time - track_time_at_index(trk, frame, ft)) / frame_delta;

  switch_ft_set_out_block(ft, out, x, ({
                            float start, end;
                            track_value_at_index(trk, &start, frame, ft);
                            track_value_at_index(trk, &end, next_frame, ft);
                            *x = interpolate_scalar(start, end, t);
                          }),
                          ({
                            vec3 start, end;
                            track_value_at_index(trk, &start, frame, ft);
                            track_value_at_index(trk, &end, next_frame, ft);
                            *x = interpolate_vec3(start, end, t);
                          }),
                          ({
                            quat start, end;
                            track_value_at_index(trk, &start, frame, ft);
                            track_value_at_index(trk, &end, next_frame, ft);
                            *x = interpolate_quat(start, end, t);
                          }));
}
void track_sample_cubic(const track_t *trk, float time, bool looping, void *out,
                        FrameType ft) {
  int frame = track_frame_index(trk, time, looping, ft);
  if (frame < 0 || frame >= (int)(trk->frames.len - 1)) {
    track_default_sample(trk, out, ft);
    return;
  }
  int next_frame = frame + 1;

  float track_time = track_adjust_time_to_fit(trk, time, looping, ft);
  float frame_delta = track_time_at_index(trk, next_frame, ft) -
                      track_time_at_index(trk, frame, ft);
  if (flte_zero(frame_delta)) {
    return track_default_sample(trk, out, ft);
  }

  float t = (track_time - track_time_at_index(trk, frame, ft)) / frame_delta;
  switch_ft_set_out_block(
      ft, out, x,
      {
        float point1 = frame_value(
            scalar_frame_t, array_ref(scalar_frame_t, &trk->frames, frame));
        float slope1 = frame_out(
            scalar_frame_t, array_ref(scalar_frame_t, &trk->frames, frame));
        float point2 =
            frame_value(scalar_frame_t,
                        array_ref(scalar_frame_t, &trk->frames, next_frame));
        float slope2 =
            frame_out(scalar_frame_t,
                      array_ref(scalar_frame_t, &trk->frames, next_frame));
        slope1 = slope1 * frame_delta;
        slope2 = slope2 * frame_delta;
        *x = hermite_scalar(t, point1, slope1, point2, slope2);
      },
      {
        vec3 point1 = frame_value(vec3_frame_t,
                                  array_ref(vec3_frame_t, &trk->frames, frame));
        vec3 slope1 = frame_out(vec3_frame_t,
                                array_ref(vec3_frame_t, &trk->frames, frame));
        vec3 point2 = frame_value(
            vec3_frame_t, array_ref(vec3_frame_t, &trk->frames, next_frame));
        vec3 slope2 = frame_out(
            vec3_frame_t, array_ref(vec3_frame_t, &trk->frames, next_frame));
        slope1 = HMM_MulV3F(slope1, frame_delta);
        slope2 = HMM_MulV3F(slope2, frame_delta);
        *x = hermite_vec3(t, point1, slope1, point2, slope2);
      },
      {
        quat point1 = frame_value(quat_frame_t,
                                  array_ref(quat_frame_t, &trk->frames, frame));
        quat slope1 = frame_out(quat_frame_t,
                                array_ref(quat_frame_t, &trk->frames, frame));
        quat point2 = frame_value(
            quat_frame_t, array_ref(quat_frame_t, &trk->frames, next_frame));
        quat slope2 = frame_out(
            quat_frame_t, array_ref(quat_frame_t, &trk->frames, next_frame));
        slope1 = HMM_MulQF(slope1, frame_delta);
        slope2 = HMM_MulQF(slope2, frame_delta);
        *x = hermite_quat(t, point1, slope1, point2, slope2);
      });
}

int track_frame_index(const track_t *trk, float time, bool looping,
                      FrameType ft) {
  usize size = trk->frames.len;
  if (size <= 1)
    return -1;

  if (looping) {
    float start_time = track_time_at_index(trk, 0, ft);
    float end_time = track_time_at_index(trk, size - 1, ft);
    float duration = end_time - start_time;

    time = fmodf(time - start_time, end_time - start_time);
    if (time < 0.0f) {
      time += end_time - start_time;
    }
    time = time + start_time;
  } else {
    if (time <= track_time_at_index(trk, 0, ft)) {
      return 0;
    }
    if (time >= track_time_at_index(trk, size - 1, ft)) {
      return (int)size - 2;
    }
  }

  for (int i = (int)size - 1; i >= 0; --i) {
    if (time >= track_time_at_index(trk, i, ft)) {
      return i;
    }
  }
  // Invalid code, we should not reach here!
  return -1;
}

void track_from_cgltf_channel(track_t *result,
                              const cgltf_animation_channel *channel,
                              FrameType ft) {
  cgltf_animation_sampler *sampler = channel->sampler;

  Interpolation interp = INTERP_CONSTANT;
  if (sampler->interpolation == cgltf_interpolation_type_linear) {
    interp = INTERP_LINEAR;
  } else if (sampler->interpolation == cgltf_interpolation_type_cubic_spline) {
    interp = INTERP_CUBIC;
  }
  bool is_sampler_cubic = interp == INTERP_CUBIC;
  result->interpolation = interp;

  floatarray_t timeline_floats = array_empty(floatarray_t);
  floatarray_t value_floats = array_empty(floatarray_t);

  u8 component_count = frame_type_float_count(ft);
  cgltf_get_scalar_values(&timeline_floats, 1, sampler->input);
  cgltf_get_scalar_values(&value_floats, component_count, sampler->output);

  u32 num_frames = (u32)sampler->input->count;
  u32 num_values_per_frame = value_floats.len / timeline_floats.len;
  usize frame_size = 0;
  switch_ft(
      ft,
      //
      { frame_size = sizeof(scalar_frame_t); },
      { frame_size = sizeof(vec3_frame_t); },
      { frame_size = sizeof(quat_frame_t); });
  result->frames.ptr = (u8 *)malloc(num_frames * frame_size);
  result->frames.len = num_frames;

  for (u32 i = 0; i < num_frames; i++) {
    int base_index = i * num_values_per_frame;
    int frame_index = i * frame_size;
    float *value_ptr = &value_floats.ptr[base_index];
    u8 *frame = &result->frames.ptr[frame_index];

    frame_set_time(frame, timeline_floats.ptr[i]);

    float _zeroes[component_count];
    ZERO(_zeroes);
    float *zeroes = (float *)(&_zeroes);
    for (u32 i = 0; i < component_count; i++) {
      safecheck(float_eq(zeroes[i], 0.0));
    }

    switch_ft(
        ft,
        {
          memcpy(&cast_ptr(scalar_frame_t, frame)->in,
                 is_sampler_cubic ? value_ptr : zeroes,
                 sizeof(float) * component_count);
        },
        {
          memcpy(&cast_ptr(vec3_frame_t, frame)->in,
                 is_sampler_cubic ? value_ptr : zeroes,
                 sizeof(float) * component_count);
        },
        {
          memcpy(&cast_ptr(quat_frame_t, frame)->in,
                 is_sampler_cubic ? value_ptr : zeroes,
                 sizeof(float) * component_count);
        });

    if (is_sampler_cubic) {
      value_ptr += component_count;
    }

    switch_ft(
        ft,
        {
          memcpy(&cast_ptr(scalar_frame_t, frame)->value, value_ptr,
                 sizeof(float) * component_count);
        },
        {
          memcpy(&cast_ptr(vec3_frame_t, frame)->value, value_ptr,
                 sizeof(float) * component_count);

          vec3 value = frame_value(vec3_frame_t, frame);
        },
        {
          memcpy(&cast_ptr(quat_frame_t, frame)->value, value_ptr,
                 sizeof(float) * component_count);
        });

    value_ptr += component_count;

    switch_ft(
        ft,
        {
          memcpy(&cast_ptr(scalar_frame_t, frame)->out,
                 is_sampler_cubic ? value_ptr : zeroes,
                 sizeof(float) * component_count);
        },
        {
          memcpy(&cast_ptr(vec3_frame_t, frame)->out,
                 is_sampler_cubic ? value_ptr : zeroes,
                 sizeof(float) * component_count);
        },
        {
          memcpy(&cast_ptr(quat_frame_t, frame)->out,
                 is_sampler_cubic ? value_ptr : zeroes,
                 sizeof(float) * component_count);
        });
  }

  array_free(&timeline_floats);
  array_free(&value_floats);
}

typedef struct {
  u32 id;
  // vec3
  track_t position;
  // quat
  track_t rotation;
  // vec3
  track_t scale;
} TransformTrack;

bool transform_track_is_valid(const TransformTrack *trk) {
  return trk->position.frames.len > 1 || trk->rotation.frames.len > 1 ||
         trk->scale.frames.len > 1;
}

float transform_track_start_time(const TransformTrack *trk) {
  float result = 0.0;
  bool is_set = false;

  if (trk->position.frames.len > 1) {
    result = track_start_time(&trk->position, FRAME_VEC3);
    is_set = true;
  }

  if (trk->rotation.frames.len > 1) {
    float start = track_start_time(&trk->rotation, FRAME_QUAT);
    if (start < result || !is_set) {
      is_set = true;
      result = start;
    }
  }

  if (trk->scale.frames.len > 1) {
    float start = track_start_time(&trk->scale, FRAME_VEC3);
    if (start < result || !is_set) {
      is_set = true;
      result = start;
    }
  }

  return result;
}

float transform_track_end_time(const TransformTrack *trk) {
  float result = 0.0;
  bool is_set = false;

  if (trk->position.frames.len > 1) {
    result = track_end_time(&trk->position, FRAME_VEC3);
    is_set = true;
  }

  if (trk->rotation.frames.len > 1) {
    float start = track_end_time(&trk->rotation, FRAME_QUAT);
    if (start > result || !is_set) {
      is_set = true;
      result = start;
    }
  }

  if (trk->scale.frames.len > 1) {
    float start = track_end_time(&trk->scale, FRAME_VEC3);
    if (start > result || !is_set) {
      is_set = true;
      result = start;
    }
  }

  return result;
}

TransformTrack transform_track_new(u32 id) {
  return (TransformTrack){.id = id,
                          .position = track_new(),
                          .rotation = track_new(),
                          .scale = track_new()};
}

Transform transform_track_sample(const TransformTrack *trk,
                                 const Transform *ref, float time,
                                 bool looping) {
  Transform result = *ref;
  // only assign if animated
  if (trk->position.frames.len > 1) {
    track_sample(&trk->position, time, looping, &result.position, FRAME_VEC3);
  }
  if (trk->rotation.frames.len > 1) {
    track_sample(&trk->rotation, time, looping, &result.rotation, FRAME_QUAT);
  }
  if (trk->scale.frames.len > 1) {
    track_sample(&trk->scale, time, looping, &result.scale, FRAME_VEC3);
  }
  return result;
}

// Some important things to note. Each joint has a transform and a parent.
//
// A transform is a slimmer representation of a linear transformation, they
// get turned into matrices when handed over to the GPU.
//
// The transform takes vertices from its parent's coordinate space, and
// transforms them into the joint's coordinate space.
//
// If you concatenate a joint's transform with its parent's and its parent's
// parent's and so on, you will get a transformation that transforms a
// vertex in the joint's coordinate space to a vertex in object/model space.
// In this code we call that a joint's global transformation.
//
// Another important transformation is the one carried by a joint's `inverse
// bind matrix`. It basically does the opposite of a joint's global
// transformation, it transforms vertices in object/model space to vertices
// in the joint's coordinate space.
typedef struct Pose {
  Transform *joints;
  int *parents;
  u32 len;
} Pose;

Pose pose_default() {
  return (Pose){
      .joints = NULL,
      .parents = NULL,
      .len = 0,
  };
}

void pose_deinit(Pose *pose) {
  free(pose->joints);
  free(pose->parents);
  pose->joints = NULL;
  pose->parents = NULL;
  pose->len = 0;
}

bool pose_is_in_hierarchy(const Pose *pose, u32 parent, u32 search) {
  if (search == parent)
    return true;

  for (int p = pose->parents[search]; p >= 0; p = pose->parents[p]) {
    if (p == (int)parent)
      return true;
  }

  return false;
}

void pose_blend(Pose *output, const Pose *a, const Pose *b, float t, int root) {
  u32 num_joints = output->len;
  for (u32 i = 0; i < num_joints; i++) {
    if (root >= 0) {
      if (!pose_is_in_hierarchy(output, (u32)root, i))
        continue;
    }

    output->joints[i] = transform_mix(&a->joints[i], &b->joints[i], t);
  }
}

void pose_init(Pose *pose, u32 joints_len) {
  pose->joints = (Transform *)malloc(sizeof(Transform) * joints_len);
  pose->parents = (int *)malloc(sizeof(int) * joints_len);
  pose->len = joints_len;
}

void pose_resize(Pose *pose, u32 new_joints_len) {
  pose->joints =
      (Transform *)realloc(pose->joints, sizeof(Transform) * new_joints_len);
  pose->parents = (int *)realloc(pose->parents, sizeof(int) * new_joints_len);
  pose->len = new_joints_len;
}

void pose_cpy(const Pose *src, Pose *dest) {
  pose_init(dest, src->len);
  memcpy(dest->joints, src->joints, sizeof(Transform) * src->len);
  memcpy(dest->parents, src->parents, sizeof(int) * src->len);
}

Transform pose_get_local_transform(const Pose *pose, u32 index) {
  return pose->joints[index];
}

void pose_set_local_transform(Pose *pose, u32 index, Transform transform) {
  pose->joints[index] = transform;
}

// Combine all the transforms up the parent chain until it reaches the root
// bone. Remember, transform concatenation is carried out from right to
// left.
Transform pose_get_global_transform(const Pose *pose, u32 index) {
  Transform result = pose->joints[index];
  for (int p = pose->parents[index]; p >= 0; p = pose->parents[p]) {
    result = transform_combine(pose->joints[p], result);
  }
  return result;
}

Pose pose_load_rest_pose_from_cgltf(cgltf_data *data) {
  usize bone_count = data->nodes_count;
  Pose out;
  pose_init(&out, bone_count);

  for (usize i = 0; i < bone_count; i++) {
    cgltf_node *node = &data->nodes[i];
    Transform transform = cgltf_get_local_transform(node);
    // transform_convert_handedness(&transform);
    out.joints[i] = transform;
    int parent = cgltf_get_node_index(node->parent, data->nodes, bone_count);
    out.parents[i] = parent;
  }

  return out;
}

// glTF stores inverse bind pose matrix for each joint. We want bind pose.To
// do this we load the rest pose and invert each joint's matrix. Inverting
// an inverse matrix returns the original matrix.
//
// We default to the joint's world space / global matrix in the case that
// the skin didn't specify an inverse bind matrix.
//
// Finally, we have to make sure that the inverse bind matrix of each joint
// is local to the space of its parent.
Pose pose_load_bind_pose_from_cgltf(cgltf_data *data, Pose rest_pose) {
  //   Pose rest_pose = pose_load_rest_pose_from_cgltf(data);
  u32 num_bones = rest_pose.len;
  // Space(bone) -> Space(model/world)
  Transform *world_bind_poses =
      (Transform *)malloc(sizeof(Transform) * num_bones);

  // Initialize to world space by default
  for (u32 i = 0; i < num_bones; i++) {
    world_bind_poses[i] = pose_get_global_transform(&rest_pose, i);
  }

  u32 num_skins = data->skins_count;
  for (u32 i = 0; i < num_skins; i++) {
    cgltf_skin *skin = &data->skins[i];
    floatarray_t inv_bind_accessor = array_empty(floatarray_t);
    cgltf_get_scalar_values(&inv_bind_accessor, 16,
                            skin->inverse_bind_matrices);

    u32 num_joints = skin->joints_count;
    for (u32 j = 0; j < num_joints; j++) {
      float *matrix = &inv_bind_accessor.ptr[j * 16];
      // Inverse bind matrix: Space(model/world) -> Space(bone)
      // Re-invert to: Space(bone) -> Space(model/world)
      mat4 inv_bind_matrix = mat4_new_from_arr(matrix);
      mat4 bind_matrix = HMM_InvGeneralM4(inv_bind_matrix);
      Transform bind_transform = transform_from_mat4_rh(&bind_matrix);
      // transform_convert_handedness(&bind_transform);
      cgltf_node *joint_node = skin->joints[j];
      int joint_index =
          cgltf_get_node_index(joint_node, data->nodes, data->nodes_count);
      world_bind_poses[joint_index] = bind_transform;
    }
  }

  Pose bind_pose;
  pose_cpy(&rest_pose, &bind_pose);

  for (u32 i = 0; i < num_bones; i++) {
    Transform current = world_bind_poses[i];
    int p = bind_pose.parents[i];
    if (p >= 0) {
      // Bring into parent space, not world space
      Transform parent = world_bind_poses[p];
      // Current: Space(bone) -> Space(model/world)
      // Parent : Space(bone) -> Space(model/world)
      // Inverted Parent : Space(model/world) -> Space(bone)
      current = transform_combine(transform_inverse(parent), current);
    }
    bind_pose.joints[i] = current;
  }

  return bind_pose;
}

void mat4_print(const mat4 *m) {
  printf("[");
  float *f = (float *)&m->Elements;
  for (u32 i = 0; i < 16; i++) {
    printf("%f,", f[i]);
  }
  printf("]");
  printf("\n");
}

// When passing to GPU, pose needs to be converted to a linear array of
// matrices
void pose_get_matrix_palette(const Pose *pose, mat4array_t *matrices) {
  safecheckf(matrices->len == 0 || matrices->len == pose->len,
             "matrices len is %zu\n", matrices->len);

  if (matrices->cap != pose->len) {
    array_reserve(mat4, matrices, pose->len);
  }

  for (u32 i = 0; i < pose->len; i++) {
    Transform t = pose_get_global_transform(pose, i);
    transform_convert_handedness(&t);
    matrices->ptr[i] = transform_to_mat4(t);
    // mat4_print(&matrices->ptr[i]);
  }
  matrices->len = pose->len;
}

// Contains shared data for all instances of a character.
typedef struct {
  Pose rest_pose;
  Pose bind_pose;

  char **joint_names;
  mat4 *inv_bind_pose;
  u32 joints_len;
} Skeleton;

void skeleton_update_inverse_bind_pose(Skeleton *sk);

void skeleton_cpy(Skeleton *dest, const Skeleton *src) {
  pose_cpy(&src->rest_pose, &dest->rest_pose);
  pose_cpy(&src->bind_pose, &dest->bind_pose);
  dest->joint_names = (char **)malloc(sizeof(char *) * src->joints_len);
  dest->inv_bind_pose = (mat4 *)malloc(sizeof(mat4) * src->joints_len);
  memcpy(&dest->joint_names, &src->joint_names,
         sizeof(char *) * src->joints_len);
  memcpy(&dest->inv_bind_pose, &src->inv_bind_pose,
         sizeof(mat4) * src->joints_len);
}

Skeleton skeleton_load(cgltf_data *data) {
  Pose rest_pose = pose_load_rest_pose_from_cgltf(data);
  Skeleton sk = {.rest_pose = rest_pose,
                 .bind_pose = pose_load_bind_pose_from_cgltf(data, rest_pose),
                 .joint_names = cgltf_load_joint_names(data),
                 .joints_len = rest_pose.len,
                 .inv_bind_pose = (mat4 *)malloc(sizeof(mat4) * rest_pose.len)};
  skeleton_update_inverse_bind_pose(&sk);
  return sk;
}

void skeleton_update_inverse_bind_pose(Skeleton *sk) {
  u32 size = sk->bind_pose.len;
  for (u32 i = 0; i < size; i++) {
    Transform world = pose_get_global_transform(&sk->bind_pose, i);
    transform_convert_handedness(&world);
    sk->inv_bind_pose[i] = HMM_InvGeneralM4(transform_to_mat4(world));
  }
}

char *skeleton_joint_name(const Skeleton *sk, u32 i) {
  return sk->joint_names[i];
}

typedef struct {
  vec3 *positions;
  vec2 *texcoords;
  vec3 *norms;
  vec4 *weights;
  u8vec4 *influences;
  vec3 *skinned_positions;
  vec3 *skinned_normals;
  u32 vertices_len;

  u32array_t indices;

  mat4array_t pose_palette;
} BoneMesh;

typedef array_type(BoneMesh) bonemesh_array_t;

void bonemesh_init(BoneMesh *mesh) {
  mesh->positions = NULL;
  mesh->texcoords = NULL;
  mesh->norms = NULL;
  mesh->weights = NULL;
  mesh->influences = NULL;
  mesh->skinned_positions = NULL;
  mesh->skinned_normals = NULL;
  mesh->vertices_len = 0;

  mesh->indices = array_empty(u32array_t);
  mesh->pose_palette = array_empty(mat4array_t);
}

void bonemesh_cpu_skin(BoneMesh *mesh, const Skeleton *sk, const Pose *p) {
  u32 num_verts = mesh->vertices_len;
  if (num_verts == 0)
    return;

  mat4array_t *palette = &mesh->pose_palette;
  pose_get_matrix_palette(p, palette);
  mat4 *inv_pose_palette = sk->inv_bind_pose;

  for (u32 i = 0; i < num_verts; i++) {
    // ivec4 joint = mesh->influences[i];
    u8vec4 joint = mesh->influences[i];
    vec4 weight = mesh->weights[i];

    mat4 m0 = HMM_MulM4F(
        HMM_MulM4(mesh->pose_palette.ptr[joint.x], inv_pose_palette[joint.x]),
        weight.X);
    mat4 m1 = HMM_MulM4F(
        HMM_MulM4(mesh->pose_palette.ptr[joint.y], inv_pose_palette[joint.y]),
        weight.Y);
    mat4 m2 = HMM_MulM4F(
        HMM_MulM4(mesh->pose_palette.ptr[joint.z], inv_pose_palette[joint.z]),
        weight.Z);
    mat4 m3 = HMM_MulM4F(
        HMM_MulM4(mesh->pose_palette.ptr[joint.w], inv_pose_palette[joint.w]),
        weight.W);

    mat4 skin = HMM_AddM4(HMM_AddM4(HMM_AddM4(m0, m1), m2), m3);

    // mesh->skinned_positions[i] = vec4_truncate(
    //     HMM_MulM4V4(skin, vec3_to_vec4_point(mesh->positions[i])));
    // mesh->skinned_normals[i] =
    //     vec4_truncate(HMM_MulM4V4(skin,
    //     vec3_to_vec4_vec(mesh->norms[i])));

    mesh->skinned_positions[i] = mat4_mul_p(skin, mesh->positions[i]);
    mesh->skinned_normals[i] = mat4_mul_v3(skin, mesh->norms[i]);
  }
}

void bonemesh_cpu_skin1(BoneMesh *mesh, const Skeleton *sk, const Pose *p) {
  u32 num_verts = mesh->vertices_len;
  if (num_verts == 0)
    return;

  Pose bind_pose = sk->bind_pose;

  for (u32 i = 0; i < num_verts; i++) {
    // ivec4 *joint = &mesh->influences[i];
    u8vec4 *joint = &mesh->influences[i];
    vec4 weight = mesh->weights[i];

    Transform skin0 =
        transform_combine(p->joints[joint->v[0]],
                          transform_inverse(bind_pose.joints[joint->v[0]]));
    vec3 p0 = transform_point(&skin0, mesh->positions[i]);
    vec3 n0 = transform_vector(&skin0, mesh->norms[i]);

    Transform skin1 =
        transform_combine(p->joints[joint->v[1]],
                          transform_inverse(bind_pose.joints[joint->v[1]]));
    vec3 p1 = transform_point(&skin1, mesh->positions[i]);
    vec3 n1 = transform_vector(&skin1, mesh->norms[i]);

    Transform skin2 =
        transform_combine(p->joints[joint->v[2]],
                          transform_inverse(bind_pose.joints[joint->v[2]]));
    vec3 p2 = transform_point(&skin2, mesh->positions[i]);
    vec3 n2 = transform_vector(&skin2, mesh->norms[i]);

    Transform skin3 =
        transform_combine(p->joints[joint->v[3]],
                          transform_inverse(bind_pose.joints[joint->v[3]]));
    vec3 p3 = transform_point(&skin3, mesh->positions[i]);
    vec3 n3 = transform_vector(&skin3, mesh->norms[i]);

    // p0 * weight.x +
    // p1 * weight.y +
    // p2 * weight.z +
    // p3 * weight.w;
    mesh->skinned_positions[i] = HMM_AddV3(
        HMM_AddV3(HMM_MulV3F(p0, weight.X), HMM_MulV3F(p1, weight.Y)),
        HMM_AddV3(HMM_MulV3F(p2, weight.Z), HMM_MulV3F(p3, weight.W)));

    mesh->skinned_normals[i] = HMM_AddV3(
        HMM_AddV3(HMM_MulV3F(n0, weight.X), HMM_MulV3F(n1, weight.Y)),
        HMM_AddV3(HMM_MulV3F(n2, weight.Z), HMM_MulV3F(n3, weight.W)));
  }
}

void bonemesh_init_attributes(BoneMesh *mesh, cgltf_attribute *attribute) {
  cgltf_accessor *accessor = attribute->data;
  u32 len = accessor->count;
  if (len == 0 || mesh->vertices_len != 0)
    return;

  mesh->positions = malloc(sizeof(vec3) * len);
  mesh->texcoords = malloc(sizeof(vec2) * len);
  mesh->norms = malloc(sizeof(vec3) * len);
  mesh->weights = malloc(sizeof(vec4) * len);
  // mesh->influences = malloc(sizeof(ivec4) * len);
  mesh->influences = malloc(sizeof(u8vec4) * len);
  mesh->skinned_positions = malloc(sizeof(vec3) * len);
  mesh->skinned_normals = malloc(sizeof(vec3) * len);
  mesh->vertices_len = len;
}

// Assume normal is your input normal, and matToLeft is the same as before.
HMM_Vec3 TransformNormal(HMM_Vec3 normal, HMM_Mat4 mat) {
  HMM_Vec3 result;
  result.X = normal.X * mat.Elements[0][0] + normal.Y * mat.Elements[1][0] +
             normal.Z * mat.Elements[2][0];
  result.Y = normal.X * mat.Elements[0][1] + normal.Y * mat.Elements[1][1] +
             normal.Z * mat.Elements[2][1];
  result.Z = normal.X * mat.Elements[0][2] + normal.Y * mat.Elements[1][2] +
             normal.Z * mat.Elements[2][2];
  return result;
}

void bonemesh_from_attribute(BoneMesh *out_mesh, cgltf_attribute *attribute,
                             cgltf_skin *skin, cgltf_node *nodes,
                             u32 node_count) {
  bonemesh_init_attributes(out_mesh, attribute);

  cgltf_attribute_type attrib_type = attribute->type;
  cgltf_accessor *accessor = attribute->data;

  u32 component_count = 0;
  if (accessor->type == cgltf_type_vec2) {
    component_count = 2;
  } else if (accessor->type == cgltf_type_vec3) {
    component_count = 3;
  } else if (accessor->type == cgltf_type_vec4) {
    component_count = 4;
  }

  floatarray_t values = array_empty(floatarray_t);
  cgltf_get_scalar_values(&values, component_count, accessor);

  for (u32 i = 0; i < accessor->count; i++) {
    int index = i * component_count;
    switch (attrib_type) {
    case cgltf_attribute_type_position:
      out_mesh->positions[i] = vec3_convert_handedness(
          true, HMM_V3(values.ptr[index], values.ptr[index + 1],
                       values.ptr[index + 2]));
      break;
    case cgltf_attribute_type_normal:
      out_mesh->norms[i] = vec3_convert_handedness(
          false, HMM_V3(values.ptr[index], values.ptr[index + 1],
                        values.ptr[index + 2]));
      break;
    case cgltf_attribute_type_texcoord:
      out_mesh->texcoords[i] = HMM_V2(values.ptr[index], values.ptr[index + 1]);
      break;
    case cgltf_attribute_type_weights:
      out_mesh->weights[i] =
          HMM_V4(values.ptr[index], values.ptr[index + 1],
                 values.ptr[index + 2], values.ptr[index + 3]);
      break;
    case cgltf_attribute_type_joints: {
      // These indices are skin relative. This function has no
      // information about the skin that is being parsed.
      u8vec4 joints = {
          .v = {(int)(values.ptr[index + 0]), (int)(values.ptr[index + 1]),
                (int)(values.ptr[index + 2]), (int)(values.ptr[index + 3])}};

      // Now convert from being relative to joints array to being
      // relative to the skeleton hierarchy
      joints.x = MAX(
          0, cgltf_get_node_index(skin->joints[joints.x], nodes, node_count));
      joints.y = MAX(
          0, cgltf_get_node_index(skin->joints[joints.y], nodes, node_count));
      joints.z = MAX(
          0, cgltf_get_node_index(skin->joints[joints.z], nodes, node_count));
      joints.w = MAX(
          0, cgltf_get_node_index(skin->joints[joints.w], nodes, node_count));

      out_mesh->influences[i] = joints;
      break;
    }
    default:
      break;
    }
  }

  array_free(&values);
}

bonemesh_array_t bonemesh_load_meshes(cgltf_data *data) {
  bonemesh_array_t result = array_empty(bonemesh_array_t);
  cgltf_node *nodes = data->nodes;
  u32 node_count = data->nodes_count;

  for (u32 i = 0; i < node_count; i++) {
    cgltf_node *node = &nodes[i];
    if (node->mesh == NULL || node->skin == NULL)
      continue;

    int num_prims = node->mesh->primitives_count;
    for (int j = 0; j < num_prims; j++) {
      BoneMesh mesh;
      bonemesh_init(&mesh);

      cgltf_primitive *primitive = &node->mesh->primitives[j];

      u32 num_attributes = primitive->attributes_count;
      for (u32 k = 0; k < num_attributes; k++) {
        cgltf_attribute *attribute = &primitive->attributes[k];
        bonemesh_from_attribute(&mesh, attribute, node->skin, nodes,
                                node_count);
      }

      if (primitive->indices != NULL) {
        u32 index_count = primitive->indices->count;
        array_reserve(u32, &mesh.indices, index_count);
        for (u32 k = 0; k < index_count; k++) {
          array_push(u32, &mesh.indices,
                     (u32)cgltf_accessor_read_index(primitive->indices, k));
        }
        safecheck(mesh.indices.len == index_count);
      }

      // if (primitive->material && prVjjimitive->material) {
      //   primitive->material->co
      // }

      array_push(BoneMesh, &result, mesh);
    }
  }

  return result;
}

typedef array_type(TransformTrack) transform_track_array_t;

typedef struct {
  transform_track_array_t tracks;
  const char *name;
  float start_time;
  float end_time;
  bool looping;
} Clip;

typedef array_type(Clip) clip_array_t;

Clip clip_new();
clip_array_t clip_load_animations_from_cgltf(cgltf_data *data);
float clip_duration(const Clip *clip);
void clip_recalculate_duration(Clip *clip);
TransformTrack *clip_transform_track_at(Clip *clip, u32 joint_idx);
float clip_sample(const Clip *clip, Pose *out_pose, float time);
float clip_adjust_time_to_fit_range(const Clip *clip, float in_time);

Clip clip_new() {
  return (Clip){
      .tracks = array_empty(transform_track_array_t),
      .name = "<unknown>",
      .start_time = 0.0,
      .end_time = 0.0,
      .looping = 0.0,
  };
}

float clip_adjust_time_to_fit_range(const Clip *clip, float in_time) {
  if (clip->looping) {
    float duration = clip->end_time - clip->start_time;
    if (duration <= 0) {
      return 0.0f;
    }
    in_time =
        fmodf(in_time - clip->start_time, clip->end_time - clip->start_time);
    if (in_time < 0.0f) {
      in_time += clip->end_time - clip->start_time;
    }
    in_time = in_time + clip->start_time;
  } else {
    if (in_time < clip->start_time) {
      in_time = clip->start_time;
    }
    if (in_time > clip->end_time) {
      in_time = clip->end_time;
    }
  }
  return in_time;
}

// This will not work for if model and animation are in separate file.
// This is because we use node index as ID, when we should use the names.
// Meaning that for some i: gltfFile1.nodes[i] != gltfFile2.nodes[i]
clip_array_t clip_load_animations_from_cgltf(cgltf_data *data) {
  u32 num_clips = data->animations_count;
  u32 num_nodes = data->nodes_count;

  clip_array_t result = array_empty(clip_array_t);
  array_reserve(Clip, &result, num_clips);
  result.len = num_clips;

  for (u32 i = 0; i < num_clips; i++) {
    Clip *clip = &result.ptr[i];
    *clip = clip_new();
    cgltf_animation *anim = &data->animations[i];
    clip->name = anim->name;
    u32 num_channels = (u32)anim->channels_count;
    array_reserve(TransformTrack, &clip->tracks, num_channels);

    for (u32 j = 0; j < num_channels; ++j) {
      cgltf_animation_channel *channel = &anim->channels[j];
      cgltf_node *target = channel->target_node;

      int node_id = cgltf_get_node_index(target, data->nodes, num_nodes);
      TransformTrack *trk = clip_transform_track_at(clip, node_id);

      if (channel->target_path == cgltf_animation_path_type_translation) {
        track_from_cgltf_channel(&trk->position, channel, FRAME_VEC3);
      } else if (channel->target_path == cgltf_animation_path_type_scale) {
        track_from_cgltf_channel(&trk->scale, channel, FRAME_VEC3);
      } else if (channel->target_path == cgltf_animation_path_type_rotation) {
        track_from_cgltf_channel(&trk->rotation, channel, FRAME_QUAT);
      }
    }

    // clip->tracks.len = num_channels;
    clip_recalculate_duration(clip);
  }

  return result;
}

void clip_load_additional_animations_from_cgltf(cgltf_data *data,
                                                const char *name,
                                                clip_array_t *clips,
                                                cgltf_node *model_nodes,
                                                cgltf_size model_node_count) {
  u32 num_clips = data->animations_count;
  u32 num_nodes = data->nodes_count;

  u32 start = clips->len;
  array_reserve(Clip, clips, num_clips);
  clips->len += num_clips;

  for (u32 i = 0; i < num_clips; i++) {
    Clip *clip = &clips->ptr[start + i];
    *clip = clip_new();
    cgltf_animation *anim = &data->animations[i];
    clip->name = anim->name;
    clip->name = name;
    printf("ANIM NAME: %s\n", clip->name);
    u32 num_channels = (u32)anim->channels_count;
    array_reserve(TransformTrack, &clip->tracks, num_channels);

    for (u32 j = 0; j < num_channels; ++j) {
      cgltf_animation_channel *channel = &anim->channels[j];
      cgltf_node *target = channel->target_node;

      int node_id = cgltf_get_node_index_by_name(target->name, model_nodes,
                                                 model_node_count);
      TransformTrack *trk = clip_transform_track_at(clip, node_id);

      if (channel->target_path == cgltf_animation_path_type_translation) {
        track_from_cgltf_channel(&trk->position, channel, FRAME_VEC3);
        cgltf_node *parent = model_nodes[node_id].parent;
        int parent_idx =
            cgltf_get_node_index(parent, model_nodes, model_node_count);
        if (strcmp(name, "goofyrunning") == 0) {
          vec3_frame_t *frames = (vec3_frame_t *)&trk->position.frames.ptr;
          for (u32 i = 0; i < trk->position.frames.len; i++) {
            frames[i].value = HMM_V3(0.0, 0.0, 0.0);
          }
        }
      } else if (channel->target_path == cgltf_animation_path_type_scale) {
        track_from_cgltf_channel(&trk->scale, channel, FRAME_VEC3);
      } else if (channel->target_path == cgltf_animation_path_type_rotation) {
        track_from_cgltf_channel(&trk->rotation, channel, FRAME_QUAT);
      }
    }

    // clip->tracks.len = num_channels;
    clip_recalculate_duration(clip);
  }
}

float clip_duration(const Clip *clip) {
  return clip->end_time - clip->start_time;
}

void clip_recalculate_duration(Clip *clip) {
  clip->start_time = 0;
  clip->end_time = 0;
  bool start_set = false;
  bool end_set = false;

  u32 tracks_len = clip->tracks.len;

  for (u32 i = 0; i < tracks_len; i++) {
    const TransformTrack *trk = &clip->tracks.ptr[i];
    if (transform_track_is_valid(trk)) {
      float start_time = transform_track_start_time(trk);
      float end_time = transform_track_end_time(trk);

      if (start_time < clip->start_time || !start_set) {
        clip->start_time = start_time;
        start_set = true;
      }

      if (end_time > clip->end_time || !end_set) {
        clip->end_time = end_time;
        end_set = true;
      }
    }
  }
}

TransformTrack *clip_transform_track_at(Clip *clip, u32 joint_idx) {
  for (u32 i = 0; (usize)i < clip->tracks.len; i++) {
    if (clip->tracks.ptr[i].id == joint_idx)
      return &clip->tracks.ptr[i];
  }

  u32 idx = clip->tracks.len;
  array_push(TransformTrack, &clip->tracks, transform_track_new(joint_idx));
  return &clip->tracks.ptr[idx];
}

float clip_sample(const Clip *clip, Pose *out_pose, float time) {
#define TR_PRINT_PANIC
  // #define TR_PRINT_DEFAULT
  if (float_eq(clip_duration(clip), 0.0)) {
    return 0.0f;
  }
  time = clip_adjust_time_to_fit_range(clip, time);

  u32 len = clip->tracks.len;
  for (u32 i = 0; i < len; i++) {
    const TransformTrack *ttrk = &clip->tracks.ptr[i];
    u32 joint = ttrk->id;
    // left handed:
    // anim = matToLeft*matAnim*matToRight
    Transform local = pose_get_local_transform(out_pose, joint);

    // right handed:
    // matAnimLeftHanded = matToRight*matToLeft*matAnim*matToRight
    Transform animated =
        transform_track_sample(ttrk, &local, time, clip->looping);

    // left handed:
    // matAnimLeftHanded = matToRight*matToLeft*matAnim*matToRight
    // transform_print(&animated);
    out_pose->joints[joint] = animated;
  }

  return time;
}

// Make sure to call `transform_convert_handedness()` if you want to use
// this
Transform cgltf_get_local_transform(cgltf_node *n) {
  Transform result = transform_default();

  if (n->has_matrix) {
    mat4 mat =
        mat4_new(n->matrix[0], n->matrix[1], n->matrix[2], n->matrix[3],
                 n->matrix[4], n->matrix[5], n->matrix[6], n->matrix[7],
                 n->matrix[8], n->matrix[9], n->matrix[10], n->matrix[11],
                 n->matrix[12], n->matrix[13], n->matrix[14], n->matrix[15]);
    result = transform_from_mat4_rh(&mat);
  }
  if (n->has_scale) {
#ifdef TRANSFORM_USE_MATRICES
    result.m = HMM_MulM4(
        HMM_Scale(HMM_V3(n->scale[0], n->scale[1], n->scale[2])), result.m);
#else
    result.scale = HMM_V3(n->scale[0], n->scale[1], n->scale[2]);
#endif
  }

  if (n->has_rotation) {
#ifdef TRANSFORM_USE_MATRICES
    result.m = HMM_MulM4(HMM_QToM4(HMM_Q(n->rotation[0], n->rotation[1],
                                         n->rotation[2], n->rotation[3])),
                         result.m);
#else
    result.rotation =
        HMM_Q(n->rotation[0], n->rotation[1], n->rotation[2], n->rotation[3]);
#endif
  }

  if (n->has_translation) {
#ifdef TRANSFORM_USE_MATRICES
    result.m =
        HMM_MulM4(HMM_Translate(HMM_V3(n->translation[0], n->translation[1],
                                       n->translation[2])),
                  result.m);
#else
    result.position =
        HMM_V3(n->translation[0], n->translation[1], n->translation[2]);
#endif
  }

  return result;
}

int cgltf_get_node_index(cgltf_node *target, cgltf_node *nodes, u32 nodes_len) {
  if (target == NULL)
    return -1;

  for (u32 i = 0; i < nodes_len; i++) {
    if (target == &nodes[i])
      return (int)i;
  }

  return -1;
}

int cgltf_get_node_index_by_name(const char *name, cgltf_node *nodes,
                                 cgltf_size nodes_len) {
  safecheck(name != NULL);

  for (u32 i = 0; i < nodes_len; i++) {
    if (strcmp(name, nodes[i].name) == 0)
      return (int)i;
  }

  return -1;
}

char **cgltf_load_joint_names(cgltf_data *data) {
  char **out = (char **)malloc(sizeof(char *) * data->nodes_count);

  for (usize i = 0; i < data->nodes_count; i++) {
    cgltf_node *node = &data->nodes[i];
    if (node->name == NULL) {
      out[i] = "<unknown>";
    } else {
      out[i] = node->name;
    }
  }

  return out;
}

void cgltf_get_scalar_values(floatarray_t *out, u32 comp_count,
                             const cgltf_accessor *in_accessor) {
  usize len = in_accessor->count * comp_count;
  array_reserve(float, out, len);
  for (cgltf_size i = 0; i < in_accessor->count; i++) {
    safecheck(cgltf_accessor_read_float(in_accessor, i,
                                        &out->ptr[i * comp_count], comp_count));
  }
  out->len = len;
}

typedef struct {
  Pose pose;
  // this struct does not own this clip
  Clip *clip;
  float time;
  float duration;
  float elapsed;
} CrossFadeTarget;

void cross_fade_target_deinit(CrossFadeTarget *cft) { pose_deinit(&cft->pose); }

typedef array_type(CrossFadeTarget) cross_fade_target_array_t;

typedef struct {
  cross_fade_target_array_t targets;
  Clip *clip;
  float time;
  Pose pose;
  Skeleton skeleton;
  bool skeleton_set;
} CrossFadeController;

CrossFadeController cross_fade_controller_init() {
  CrossFadeController out;
  ZERO(out);
  return out;
}

void cross_fade_controller_set_skeleton(CrossFadeController *cfc,
                                        Skeleton *sk) {
  pose_cpy(&sk->rest_pose, &cfc->pose);
  skeleton_cpy(&cfc->skeleton, sk);
  cfc->skeleton_set = true;
}

void cross_fade_controller_play(CrossFadeController *cfc, Clip *target) {
  for (usize i = 0; i < cfc->targets.len; i++) {
    cross_fade_target_deinit(&cfc->targets.ptr[i]);
  }
  array_clear(&cfc->targets);

  cfc->clip = target;
  pose_cpy(&cfc->skeleton.rest_pose, &cfc->pose);
  cfc->time = target->start_time;
}

void cross_fade_controller_fade_to(CrossFadeController *cfc, Clip *target,
                                   float fade_time) {
  if (cfc->clip == NULL) {
    cross_fade_controller_play(cfc, target);
    return;
  }

  // If it is the last target do nothing
  if (cfc->targets.len >= 1 &&
      cfc->targets.ptr[cfc->targets.len - 1].clip == target) {
    return;
  } else if (cfc->clip == target) {
    return;
  }

  CrossFadeTarget cft;
  ZERO(cft);
  cft.clip = target;
  pose_cpy(&cfc->skeleton.rest_pose, &cft.pose);
  cft.duration = fade_time;

  array_push(CrossFadeTarget, &cfc->targets, cft);
}

void cross_fade_controller_update(CrossFadeController *cfc, float dt) {
  if (cfc->clip == NULL || !cfc->skeleton_set)
    return;

  usize num_targets = cfc->targets.len;
  for (u32 i = 0; i < num_targets; i++) {
    CrossFadeTarget *target = &cfc->targets.ptr[i];
    float duration = target->duration;
    if (target->elapsed >= duration) {
      cfc->clip = target->clip;
      cfc->time = target->time;
      pose_deinit(&cfc->pose);
      pose_cpy(&target->pose, &cfc->pose);
      array_erase(CrossFadeTarget, &cfc->targets, i);
      break;
    }
  }

  num_targets = cfc->targets.len;
  pose_cpy(&cfc->skeleton.rest_pose, &cfc->pose);
  cfc->time = clip_sample(cfc->clip, &cfc->pose, cfc->time + dt);

  for (u32 i = 0; i < num_targets; i++) {
    CrossFadeTarget *target = &cfc->targets.ptr[i];
    target->time = clip_sample(target->clip, &target->pose, target->time + dt);
    target->elapsed += dt;
    float t = target->elapsed / target->duration;
    if (fgt(t, 1.0)) {
      t = 1.0;
    }
    pose_blend(&cfc->pose, &cfc->pose, &target->pose, t, -1);
  }
}

Pose blending_make_additive_pose(const Skeleton *sk, const Clip *clip) {
  Pose result;
  ZERO(result);
  pose_cpy(&sk->rest_pose, &result);
  clip_sample(clip, &result, clip->start_time);
  return result;
}

void blending_add(Pose *output, const Pose *in_pose, const Pose *add_pose,
                  const Pose *base_pose, int blendroot) {
  u32 num_joints = add_pose->len;
  for (u32 i = 0; i < num_joints; i++) {
    Transform input = in_pose->joints[i];
    Transform additive = add_pose->joints[i];
    Transform additive_base = base_pose->joints[i];

    if (blendroot >= 0 && !pose_is_in_hierarchy(add_pose, blendroot, i)) {
      continue;
    }

    // out_pose = in_pose + (add_pose - base_pose)
    Transform result = {
        .position =
            HMM_AddV3(input.position,
                      HMM_SubV3(additive.position, additive_base.position)),
        .rotation = quat_norm(quat_mul_quat(
            input.rotation, quat_mul_quat(quat_inverse(additive_base.rotation),
                                          additive.rotation))),
        .scale = HMM_AddV3(input.scale,
                           HMM_SubV3(additive.scale, additive_base.scale)),
    };
  }
}

typedef struct {
  mat4array_t pose_palette;
  u32 clip_idx;
} AnimationInstance;

#ifdef DEBUG
typedef struct {
  strarray_t animations;
  usize selected_animation;
} DebugGui;
#endif

typedef struct {
  vec3 focus_point; // The point around which the camera orbits
  float distance;   // Distance from the focus point
  float yaw;        // Horizontal rotation angle in degrees
  float pitch;      // Vertical rotation angle in degrees
  mat4 view_matrix;
} OrbitCamera;

void orbit_camera_update(OrbitCamera *camera);

#ifdef LEFT_HANDED
#define ZPOS 1.1
#define ZCAMERAPOS -3.1
#else
#define ZPOS -1.1
#define ZCAMERAPOS 3.1
#endif

void orbit_camera_init(OrbitCamera *camera) {
  camera->focus_point = HMM_V3(0.0f, 0.0f, ZPOS);
  camera->distance = 6.0f; // Initial distance from the focus point
  camera->yaw = 0.0f;      // Initial yaw, in degrees
  camera->pitch = 90.0f; // Initial pitch, in degrees (90 degrees means looking
                         // straight down)

  orbit_camera_update(camera);
}

vec3 orbit_camera_direction(const OrbitCamera *camera) {
  float yaw_radians = HMM_ToRad(camera->yaw);
  float pitch_radians = HMM_ToRad(camera->pitch);

  // Calculate the new camera position based on the angles and distance
  vec3 eye_position;
  eye_position.X = camera->focus_point.X +
                   camera->distance * cosf(yaw_radians) * sinf(pitch_radians);
  eye_position.Y =
      camera->focus_point.Y + camera->distance * cosf(pitch_radians);
  eye_position.Z = camera->focus_point.Z +
                   camera->distance * sinf(yaw_radians) * sinf(pitch_radians);

  return HMM_SubV3(camera->focus_point, eye_position);
}

void orbit_camera_update(OrbitCamera *camera) {
  float yaw_radians = HMM_ToRad(camera->yaw);
  float pitch_radians = HMM_ToRad(camera->pitch);

  // Calculate the new camera position based on the angles and distance
  vec3 eye_position;
  eye_position.X = camera->focus_point.X +
                   camera->distance * cosf(yaw_radians) * sinf(pitch_radians);
  eye_position.Y =
      camera->focus_point.Y + camera->distance * cosf(pitch_radians);
  eye_position.Z = camera->focus_point.Z +
                   camera->distance * sinf(yaw_radians) * sinf(pitch_radians);

  // Create the view matrix
  camera->view_matrix =
      HMM_LookAt(eye_position, camera->focus_point, HMM_V3(0.0f, 1.0f, 0.0f));
}

static struct {
  float rx, ry;
  sg_pipeline pip;
  sg_bindings bind;
  BoneMesh bm;
  Skeleton sk;
  clip_array_t clips;
  Pose animated_pose;
  mat4array_t pose_palette;
  mat4array_t inv_bind_pose;
  vs_params_t vs_params;
  AnimationInstance animation;
  float fade_timer;
  CrossFadeController fade_controller;
  OrbitCamera camera;
#ifdef DEBUG
  DebugGui gui;
#endif
} state;

void set_vs_params() {
  vs_params_t vs_params;
  ZERO(vs_params);
  // const float w = sapp_widthf();
  // const float h = sapp_heightf();
  const float w = 800;
  const float h = 600;
  //   const float t = (float)(sapp_frame_duration() * 60.0);
  // const float t = (float)(sapp_frame_duration());
  const float t = 1.0;

  HMM_Mat4 proj = HMM_Perspective(60.0f, w / h, 0.01f, 10.0f);
  HMM_Mat4 view = HMM_LookAt((HMM_Vec3){.X = 0.0f, .Y = 0.0f, .Z = 0.0f},
                             (HMM_Vec3){.X = 0.0f, .Y = 1.5f, .Z = 6.0f},
                             (HMM_Vec3){.X = 0.0f, .Y = 1.0f, .Z = 0.0f});
  state.rx += 1.0f * t;
  state.ry += 2.0f * t;
  // HMM_Mat4 rxm = HMM_Rotate(state.rx, (HMM_Vec3){{1.0f, 0.0f, 0.0f}});
  // HMM_Mat4 rym = HMM_Rotate(state.ry, (HMM_Vec3){{0.0f, 1.0f, 0.0f}});

  // HMM_Mat4 model = HMM_MulM4(HMM_Translate(HMM_V3(-0.5, 0.0, -0.5)),
  //                            HMM_Scale(HMM_V3(1.0, 1.0, 1.0)));
  // HMM_Mat4 model = HMM_Scale(HMM_V3(1.0, 1.0, 1.0));

  HMM_Mat4 model = HMM_M4D(1.0);

  // HMM_MulM4(HMM_MulM4(rxm, rym), HMM_Scale(HMM_V3(1.01, 1.01, 1.01))));
  // HMM_Mat4 model = HMM_M4D(1.0);

  vs_params.model = model;
  vs_params.view = view;
  vs_params.projection = proj;

#ifndef CPU_SKIN
  pose_get_matrix_palette(&state.animated_pose, &state.pose_palette);
  memcpy(&vs_params.pose, state.pose_palette.ptr,
         sizeof(mat4) * state.pose_palette.len);
  memcpy(&vs_params.invBindPose, state.inv_bind_pose.ptr,
         sizeof(mat4) * state.inv_bind_pose.len);
#endif

  state.vs_params = vs_params;
}

void cgltf_load_data(cgltf_data **data, cgltf_options *opts, const char *path) {
  if (cgltf_parse_file(opts, path, data) != cgltf_result_success) {
    printf("Failed to parse scene\n");
    exit(1);
  }

  if (cgltf_load_buffers(opts, *data, path) != cgltf_result_success) {
    cgltf_free(*data);
    printf("Failed to load buffers\n");
    exit(1);
  }

  if (cgltf_validate(*data) != cgltf_result_success) {
    cgltf_free(*data);
    printf("glTF data validation failed!\n");
    exit(1);
  }
}

// Vertex structure for quad
typedef struct {
  float x, y;
  float u, v;
} vertex_t;

void init(void) {
  ZERO(state);

  sg_setup(&(sg_desc){
      .context = sapp_sgcontext(),
      .logger.func = slog_func,
  });

  cgltf_options opts = {};
  ZERO(opts);
  cgltf_data *model_data = NULL;
  // const char *path = "./src/assets/Woman.gltf";
  // const char *path = "./src/assets/scene.gltf";
  // const char *path = "./src/assets/simple_skin.gltf";
  const char *path = "./src/assets/vanguard.glb";
  // const char *path = "./src/assets/stacy.glb";
  // const char *path = "./src/assets/master_chief/scene.gltf";
  // const char *path = "./src/assets/low_poly_crystal_pangolin/scene.gltf";
  cgltf_load_data(&model_data, &opts, path);

  printf("TEXTURES: %zu\n", model_data->textures_count);

  cgltf_data *anim_data = NULL;
  cgltf_options opts2 = {};
  ZERO(opts2);
  // const char *anim_path = "./src/assets/vanguard@bellydance.glb";
  char *anim_path = "./src/assets/vanguard@goofyrunning.glb";
  cgltf_load_data(&anim_data, &opts2, anim_path);

  // cgltf_free(model_data);

  state.clips = clip_load_animations_from_cgltf(model_data);
  state.clips.ptr[0].name = "bind";
  clip_load_additional_animations_from_cgltf(anim_data, "goofyrunning",
                                             &state.clips, model_data->nodes,
                                             model_data->nodes_count);
  anim_path = "./src/assets/vanguard@bellydance.glb";
  cgltf_load_data(&anim_data, &opts2, anim_path);
  clip_load_additional_animations_from_cgltf(anim_data, "bellydance",
                                             &state.clips, model_data->nodes,
                                             model_data->nodes_count);
  anim_path = "./src/assets/vanguard@samba.glb";
  cgltf_load_data(&anim_data, &opts2, anim_path);
  clip_load_additional_animations_from_cgltf(anim_data, "samba", &state.clips,
                                             model_data->nodes,
                                             model_data->nodes_count);

  // clip_array_t anim_clips = clip_load_animations_from_cgltf(anim_data);
  // array_concat(Clip, &state.clips, &anim_clips);

  for (usize i = 0; i < state.clips.len; i++) {
    state.clips.ptr[i].looping = true;
  }

  bonemesh_array_t meshes = bonemesh_load_meshes(model_data);
  state.bm = meshes.ptr[0];
  state.sk = skeleton_load(model_data);
  pose_cpy(&state.sk.rest_pose, &state.animated_pose);
  state.animation.pose_palette = array_empty(mat4array_t);
  array_reserve(mat4, &state.animation.pose_palette, state.sk.rest_pose.len);

  state.pose_palette = array_empty(mat4array_t);
  state.inv_bind_pose = (mat4array_t){
      .ptr = state.sk.inv_bind_pose,
      .len = state.sk.joints_len,
      .cap = state.sk.joints_len,
  };

  JOINT_NAMES = state.sk.joint_names;

  u32 default_clip_index = 0;

  state.fade_timer = 3.0f;
  state.fade_controller = cross_fade_controller_init();
  cross_fade_controller_set_skeleton(&state.fade_controller, &state.sk);
  state.animation.clip_idx = 1;
  cross_fade_controller_play(&state.fade_controller, &state.clips.ptr[1]);
  cross_fade_controller_update(&state.fade_controller, 0.0);
  pose_get_matrix_palette(&state.fade_controller.pose,
                          &state.animation.pose_palette);

#ifdef DEBUG
  strarray_t names = array_empty(strarray_t);
  array_reserve(const char *, &names, state.clips.len);

  for (usize i = 0; i < state.clips.len; i++) {
    printf("NAME: %s\n", state.clips.ptr[i].name);
    if (state.clips.ptr[i].name == NULL) {
      array_push(const char *, &names, (const char *)"NULL");
    } else {
      array_push(const char *, &names, state.clips.ptr[i].name);
    }
  }
  state.gui.animations = names;
  state.gui.selected_animation = default_clip_index;
#endif

#ifdef CPU_SKIN
  bonemesh_cpu_skin(&state.bm, &state.sk, &state.animated_pose);
#endif

  /* create shader */
#ifdef CPU_SKIN
  sg_shader shd = sg_make_shader(bonemesh_cpu_shader_desc(sg_query_backend()));
#else
  sg_shader shd = sg_make_shader(bonemesh_shader_desc(sg_query_backend()));
#endif

  // BoneMesh *mesh = &meshes.ptr[0];

  /* __dbgui_setup(sapp_sample_count()); */

  sg_buffer vbuf = sg_make_buffer(
      &(sg_buffer_desc){.data =
#ifdef CPU_SKIN
                            (sg_range){state.bm.skinned_positions,
                                       sizeof(vec3) * state.bm.vertices_len},
#else
                            (sg_range){state.bm.positions,
                                       sizeof(vec3) * state.bm.vertices_len},
#endif
                        .label = "model-vertices"});
  sg_buffer txbuf = sg_make_buffer(&(sg_buffer_desc){
      .data =
          (sg_range){state.bm.texcoords, sizeof(vec2) * state.bm.vertices_len},
      .label = "model-texcoords"});
  sg_buffer normbuf = sg_make_buffer(&(sg_buffer_desc){
#ifdef CPU_SKIN
      .data = (sg_range){state.bm.skinned_normals,
                         sizeof(vec3) * state.bm.vertices_len},
#else
      .data = (sg_range){state.bm.norms, sizeof(vec3) * state.bm.vertices_len},
#endif
      .label = "model-norm"});
  sg_buffer weightbuf = sg_make_buffer(&(sg_buffer_desc){
      .data =
          (sg_range){state.bm.weights, sizeof(vec4) * state.bm.vertices_len},
      .label = "model-weights"});
  sg_buffer influencebuf = sg_make_buffer(&(sg_buffer_desc){
      // .data = (sg_range){(vec4 *)state.bm.influences,
      //                    sizeof(ivec4) * state.bm.vertices_len},
      .data = (sg_range){state.bm.influences,
                         sizeof(u8vec4) * state.bm.vertices_len},
      .label = "model-influences"});

  sg_buffer ibuf = sg_make_buffer(
      &(sg_buffer_desc){.type = SG_BUFFERTYPE_INDEXBUFFER,
                        .data = SG_RANGE_ARR(u32, state.bm.indices),
                        .label = "model-indices"});

  /* create pipeline object */
  state.pip = sg_make_pipeline(&(sg_pipeline_desc){
      .layout =
          {
              /* test to provide buffer stride, but no attr offsets */
              // pos
              .buffers[0].stride = sizeof(vec3),
              .attrs[0] = {.format = SG_VERTEXFORMAT_FLOAT3, .buffer_index = 0},
              // norms
              .buffers[1].stride = sizeof(vec3),
              .attrs[1] = {.format = SG_VERTEXFORMAT_FLOAT3, .buffer_index = 1},
              // texcoords
              .buffers[2].stride = sizeof(vec2),
              .attrs[2] = {.format = SG_VERTEXFORMAT_FLOAT2, .buffer_index = 2},
              // weights
              .buffers[3].stride = sizeof(vec4),
              .attrs[3] = {.format = SG_VERTEXFORMAT_FLOAT4, .buffer_index = 3},
              // influences
              // .buffers[4].stride = sizeof(ivec4),
              .buffers[4].stride = sizeof(u8vec4),
              .attrs[4] = {.format = SG_VERTEXFORMAT_UBYTE4, .buffer_index = 4},
          },
      .shader = shd,
      .index_type = SG_INDEXTYPE_UINT32,
#ifdef SOKOL_METAL
      .face_winding = SG_FACEWINDING_CCW,
      .cull_mode = SG_CULLMODE_BACK,
  // .face_winding = SG_FACEWINDING_CW,
  // .cull_mode = SG_CULLMODE_BACK,
#else
      .face_winding = SG_FACEWINDING_CCW,
      .cull_mode = SG_CULLMODE_BACK,
#endif
      .depth =
          {
              .write_enabled = true,
              .compare = SG_COMPAREFUNC_LESS_EQUAL,
          },
      .label = "cube-pipeline"});

  /* setup resource bindings */
  state.bind = (sg_bindings){.vertex_buffers[0] = vbuf,
                             .vertex_buffers[1] = normbuf,
                             .vertex_buffers[2] = txbuf,
                             .vertex_buffers[3] = weightbuf,
                             .vertex_buffers[4] = influencebuf,
                             .index_buffer = ibuf};

  // create a checkerboard texture
  uint32_t pixels[4 * 4] = {
      0xFFFFFFFF, 0xFF000000, 0xFFFFFFFF, 0xFF000000, 0xFF000000, 0xFFFFFFFF,
      0xFF000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFF000000, 0xFFFFFFFF, 0xFF000000,
      0xFF000000, 0xFFFFFFFF, 0xFF000000, 0xFFFFFFFF,
  };

  int width, height, channels;
  // stbi_set_flip_vertically_on_load(1);
  // u8 *rgba_image =
  //     stbi_load("./src/assets/stacy.jpeg", &width, &height, &channels, 4);
  // u8 *rgba_image =
  //     stbi_load("./src/assets/vanguard.png", &width, &height, &channels, 4);
  u8 *rgba_image = stbi_load("./src/assets/vanguard_pixel.png", &width, &height,
                             &channels, 4);
  safecheck(rgba_image != NULL);

  state.bind.fs.images[SLOT_tex] = sg_make_image(&(sg_image_desc){
      .width = width,
      .height = height,
      .pixel_format = SG_PIXELFORMAT_RGBA8,
      //  .data.subimage[0][0] = SG_RANGE(pixels),
      .data.subimage[0][0] = (sg_range){rgba_image, width * height * 4},
      .label = "cube-texture"});

  // create a sampler object with default attributes
  state.bind.fs.samplers[SLOT_smp] = sg_make_sampler(&(sg_sampler_desc){0});

  set_vs_params();
  stm_setup();

#ifndef DISABLE_GUI
  // use sokol-nuklear with all default-options (we're not doing
  // multi-sampled rendering or using non-default pixel formats)
  snk_setup(&(snk_desc_t){
      .dpi_scale = sapp_dpi_scale(),
      .logger.func = slog_func,
  });
#endif

  orbit_camera_init(&state.camera);
}

#ifndef DISABLE_GUI
static int draw_debug_gui(struct nk_context *ctx);
#endif

static u64 last_time = 0;
void frame(void) {
#ifndef DISABLE_GUI
  struct nk_context *ctx = snk_new_frame();
  draw_debug_gui(ctx);
#endif

  const float w = sapp_widthf();
  const float h = sapp_heightf();

  float delta_time = (float)stm_sec(stm_laptime(&last_time));

  fs_params_t fs_params;
  vec3 light_dir = orbit_camera_direction(&state.camera);
  memcpy(&fs_params.light, &light_dir, sizeof(vec3));

  //   fs_params.light[0] = 0.0;
  //   fs_params.light[1] = 0.0;
  // #ifdef LEFT_HANDED
  //   fs_params.light[2] = 1.0;
  // #else
  //   fs_params.light[2] = -1.0;
  // #endif

  const float t = (float)(sapp_frame_duration() * 60.0);

  // HMM_Mat4 proj = HMM_Perspective(90.0, w / h, 0.1f, 100.0f);
  // HMM_Mat4 view = HMM_LookAt((HMM_Vec3){.X = 0.0f, .Y = 0.0f, .Z =
  // ZCAMERAPOS},
  //                            (HMM_Vec3){.X = 0.0f, .Y = 0.0f, .Z = 0.0f},
  //                            (HMM_Vec3){.X = 0.0f, .Y = 1.0f, .Z = 0.0f});

  // safecheck(state.animation.clip_idx == 0);
  cross_fade_controller_update(&state.fade_controller, delta_time);
  state.fade_timer -= delta_time;
  // if (flt_zero(state.fade_timer)) {
  //   state.fade_timer = 3.0f;

  //   u32 clip = state.animation.clip_idx;
  //   while (clip == state.animation.clip_idx) {
  //     clip = (clip + 1) % state.clips.len;
  //   }
  //   state.animation.clip_idx = clip;
  //   cross_fade_controller_fade_to(&state.fade_controller,
  //                                 &state.clips.ptr[state.animation.clip_idx],
  //                                 0.5);
  // }
  // printf("animt %f delta_time %f last_time %llu\n", state.animation.t,
  //        (float)delta_time, last_time);

  pose_get_matrix_palette(&state.fade_controller.pose,
                          &state.animation.pose_palette);

  // Copy the transformation matrices for the joints
  memcpy(&state.vs_params.pose, state.animation.pose_palette.ptr,
         sizeof(mat4) * state.animation.pose_palette.len);
  // Copy the inverse transformation matrices for the bind pose of the
  // skeleton
  memcpy(&state.vs_params.invBindPose, state.inv_bind_pose.ptr,
         sizeof(mat4) * state.inv_bind_pose.len);

  const float aspect_ratio = w / h;
  // HMM_Mat4 proj = HMM_Perspective(50.0, w / h, 0.1f, 100.0f);
  HMM_Mat4 proj =
      HMM_Orthographic(-aspect_ratio, aspect_ratio, -1, 1, 0.0001, 1000);
  // HMM_Mat4 view = HMM_M4D(1.0);
  HMM_Mat4 view = state.camera.view_matrix;

  state.ry += 1.0f * t;
  state.rx += 1.0f * t;
  // HMM_Mat4 rym = HMM_Rotate(state.ry, (HMM_Vec3){{0.0f, 1.0f, 0.0f}});
  HMM_Mat4 rym = HMM_M4D(1.0);

  HMM_Mat4 rxm = HMM_Rotate(state.rx, (HMM_Vec3){{1.0f, 0.0f, 0.0f}});
  HMM_Mat4 model =
      HMM_MulM4(HMM_Translate(HMM_V3(0.0, -0.5, ZPOS)),
                HMM_MulM4(rym, HMM_Scale(HMM_V3(0.75, 0.75, .75))));

  state.vs_params.model = model;
  state.vs_params.view = view;
  state.vs_params.projection = proj;

  sg_pass_action pass_action = {
      .colors[0] = {.load_action = SG_LOADACTION_CLEAR,
                    .clear_value = {0.25f, 0.5f, 0.75f, 1.0f}}};
  // sg_pass_action pass_action = {
  //     .colors[0] = {.load_action = SG_LOADACTION_CLEAR,
  //                   .clear_value = {0.0, 0.0, 0.0, 1.0f}}};
  sg_begin_default_pass(&pass_action, (int)w, (int)h);
  sg_apply_pipeline(state.pip);
  sg_apply_bindings(&state.bind);
  sg_apply_uniforms(SG_SHADERSTAGE_VS, SLOT_vs_params,
                    &SG_RANGE(state.vs_params));
  sg_apply_uniforms(SG_SHADERSTAGE_FS, SLOT_fs_params, &SG_RANGE(fs_params));
  sg_draw(0, state.bm.indices.len, 1);

#ifndef DISABLE_GUI
  snk_render(sapp_width(), sapp_height());
#endif

  sg_end_pass();
  sg_commit();
}

void cleanup(void) { sg_shutdown(); }

static bool mouse_dragging = false;

void input(const sapp_event *ev) {
  switch (ev->type) {
  case SAPP_EVENTTYPE_MOUSE_DOWN:
    if (ev->mouse_button == SAPP_MOUSEBUTTON_LEFT) {
      mouse_dragging = true;
    }
    break;
  case SAPP_EVENTTYPE_MOUSE_UP:
    if (ev->mouse_button == SAPP_MOUSEBUTTON_LEFT) {
      mouse_dragging = false;
    }
    break;
  case SAPP_EVENTTYPE_MOUSE_MOVE:
    if (mouse_dragging) {
      float MOUSE_SENSITIVITY = 0.2f;

      // Update camera angles based on mouse movement
      state.camera.yaw += ev->mouse_dx * MOUSE_SENSITIVITY;
      state.camera.pitch -= ev->mouse_dy * MOUSE_SENSITIVITY;

      // Clamp the pitch angle to avoid gimbal lock
      if (state.camera.pitch > 89.0f)
        state.camera.pitch = 89.0f;
      if (state.camera.pitch < -89.0f)
        state.camera.pitch = -89.0f;

      // Update the camera's position and view matrix
      orbit_camera_update(&state.camera);
    }
    break;
  default:
    break;
  }

#ifndef DISABLE_GUI
  snk_handle_event(ev);
#endif
}

sapp_desc sokol_main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

#ifdef SOKOL_METAL
  printf("Using Metal\n");
  const char *name = "occam (Metal)";
#else
  printf("Using OpenGL\n");
  const char *name = "occam (OpenGL)";
#endif

#ifdef CPU_SKIN
  printf("Using CPU skinning\n");
#else
  printf("Using GPU skinning\n");
#endif

  return (sapp_desc){
      .init_cb = init,
      .frame_cb = frame,
      .event_cb = input,
      .cleanup_cb = cleanup,
      // .event_cb = __dbgui_event,
      .width = 800,
      .height = 600,
      .sample_count = 4,
      .window_title = name,
      .icon.sokol_default = true,
      .logger.func = slog_func,
  };
}

#ifndef DISABLE_GUI
static int draw_debug_gui(struct nk_context *ctx) {
  if (nk_begin(ctx, "Animation Selector", nk_rect(50, 50, 220, 220),
               NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_CLOSABLE)) {

    nk_layout_row_static(ctx, 30, 200, 1);
    if (nk_checkbox_label(ctx, "Disable loop",
                          &state.clips.ptr[state.animation.clip_idx].looping)) {
      // state.animation.t = 0.0;
    }
    int selected_animation =
        nk_combo(ctx, state.gui.animations.ptr, state.gui.animations.len,
                 state.animation.clip_idx, 25, nk_vec2(200, 200));
    if ((u32)selected_animation != state.animation.clip_idx) {
      // state.animation.t = 0.0;
      state.animation.clip_idx = selected_animation;
      cross_fade_controller_fade_to(&state.fade_controller,
                                    &state.clips.ptr[state.animation.clip_idx],
                                    0.5);
    }
  }

  nk_end(ctx);
  return 1;
}
#endif

ASSUME_NONNULL_END