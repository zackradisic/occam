// For wasm i think turn off?
#define HANDMADE_MATH_NO_SSE

// #define CPU_SKIN

#define SOKOL_IMPL
#include <HandmadeMath.h>
#include "common.h"
#include <sokol/sokol_glue.h>
#include "sokol.h"

#ifdef CPU_SKIN
#include "shaders/bonemesh_cpu.glsl.h"
#else
#include "shaders/bonemesh.glsl.h"
#endif

#include "stdio.h"
#include "arena.h"
#include <sokol/sokol_glue.h>
#define CGLTF_IMPLEMENTATION
#include <cgltf.h>
#include "array.h"

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

typedef array_type(mat4) mat4array_t;
typedef array_type(float) floatarray_t;
typedef array_type(u32) u32array_t;

int cgltf_get_node_index(cgltf_node *target, cgltf_node *nodes, u32 nodes_len);
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
  char **_Nullable ptr;
  usize len;
  usize cap;
} strarray_t;

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

void convertm4(mat4 *m) {
#ifdef LEFT_HANDED
  convertRHtoLH(m->Elements);
#else
#endif
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
vec3 quat_mul_v3(quat q, vec3 v) {
  // 	return    q.vector * 2.0f * dot(q.vector, v) +
  //   v *(q.scalar * q.scalar - dot(q.vector, q.vector)) +
  //       cross(q.vector, v) * 2.0f * q.scalar;
  vec3 out;

  out = HMM_MulV3F(HMM_MulV3F(q.XYZ, 2.0), HMM_DotV3(q.XYZ, v));
  out = HMM_AddV3(out, HMM_MulV3F(v, q.W * q.W - HMM_DotV3(q.XYZ, q.XYZ)));
  out = HMM_AddV3(out, HMM_MulV3F(HMM_MulV3F(HMM_Cross(q.XYZ, v), 2.0), q.W));
  return out;
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

typedef struct {
  vec3 position;
  quat rotation;
  vec3 scale;
} Transform;

Transform cgltf_get_local_transform(cgltf_node *n);

Transform transform_default() {
  Transform out;
  out.position = HMM_V3(0, 0, 0);
  out.rotation = HMM_Q(0, 0, 0, 1);
  out.scale = HMM_V3(1, 1, 1);
  return out;
}

// Order is right -> left
Transform transform_combine(Transform a, Transform b) {
  Transform out = transform_default();
  out.scale = HMM_MulV3(a.scale, b.scale);
  out.rotation = HMM_MulQ(b.rotation, a.rotation);
  out.position = quat_mul_v3(a.rotation, HMM_MulV3(a.scale, b.position));
  out.position = HMM_AddV3(a.position, out.position);
  return out;
}

Transform transform_inverse(Transform t) {
  Transform inv = transform_default();

  inv.rotation = HMM_InvQ(t.rotation);

  inv.scale.X = fabs(t.scale.X) < VEC3_EPSILON ? 0.0f : 1.0f / t.scale.X;
  inv.scale.Y = fabs(t.scale.Y) < VEC3_EPSILON ? 0.0f : 1.0f / t.scale.Y;
  inv.scale.Z = fabs(t.scale.Z) < VEC3_EPSILON ? 0.0f : 1.0f / t.scale.Z;

  vec3 inv_translation = HMM_MulV3F(t.position, -1.0f);
  inv.position =
      quat_mul_v3(inv.rotation, HMM_MulV3(inv.scale, inv_translation));

  return inv;
}

bool transform_eq(const Transform *a, const Transform *b) {
  return HMM_EqV3(a->position, b->position) && HMM_EqV3(a->scale, b->scale) &&
         quat_eq(a->rotation, b->rotation);
}

vec3 transform_point(const Transform *t, vec3 p) {
  vec3 out = vec3_default();
  out = quat_mul_v3(t->rotation, HMM_MulV3(t->scale, p));
  out = HMM_AddV3(t->position, out);
  return out;
}

vec3 transform_vector(const Transform *t, vec3 v) {
  vec3 out = vec3_default();
  out = quat_mul_v3(t->rotation, HMM_MulV3(t->scale, v));
  return out;
}

#if 0
Transform transform_from_mat4(const mat4 *m) {
  Transform out = transform_default();

  out.position = HMM_V3(m->Columns[3].X, m->Columns[3].Y, m->Columns[3].Z);
  out.rotation = HMM_M4ToQ(*m);

  mat4 rot_scale_mat =
      mat4_new(m->Columns[0].X, m->Columns[0].Y, m->Columns[0].Z, 0, // col 1
               m->Columns[1].X, m->Columns[1].Y, m->Columns[1].Z, 0, //
               m->Columns[2].X, m->Columns[2].Y, m->Columns[2].Z, 0, //
               0, 0, 0, 1);

  mat4 inv_rot_mat = HMM_QToM4(HMM_InvQ(out.rotation));
  mat4 scale_skew_mat = HMM_MulM4(rot_scale_mat, inv_rot_mat);

  out.scale = (vec3){
      // scale_skew_mat.Elements[0][0],
      // scale_skew_mat.Elements[1][1],
      // scale_skew_mat.Elements[2][2],
      scale_skew_mat.Columns[0].X,
      scale_skew_mat.Columns[1].Y,
      scale_skew_mat.Columns[2].Z,
  };

  return out;
}
#else
// float signum(float x) {
//   if (x > 0.0f)
//     return 1;
//   if (x < 0.0f)
//     return -1;
//   return 0;
// }
float signum(float x) {
  float epsilon = 1e-6; // Choose a small value that suits your needs
  if (x > epsilon)
    return 1.0f;
  if (x < -epsilon)
    return -1.0f;
  return 0.0f;
}

float mat3_trace(const mat3 *m) {
  return m->Columns[0].X + m->Columns[1].Y + m->Columns[2].Z;
}

quat mat3_to_quat(mat3 m) {
  float trace = mat3_trace(&m);
  if (trace >= 0.0f) {
    float s = sqrtf(1.0f + trace) * 2.0f;
    float invS = 1.0f / s;
    float x = (m.Columns[2].Y - m.Columns[1].Z) * invS;
    float y = (m.Columns[0].Z - m.Columns[2].X) * invS;
    float z = (m.Columns[1].X - m.Columns[0].Y) * invS;
    float w = 0.25f * s;
    return HMM_Q(x, y, z, w);
  } else if (m.Columns[0].X > m.Columns[1].Y &&
             m.Columns[0].X > m.Columns[2].Z) {
    float s =
        sqrtf(1.0f + m.Columns[0].X - m.Columns[1].Y - m.Columns[2].Z) * 2.0f;
    float invS = 1.0f / s;
    float x = 0.25f * s;
    float y = (m.Columns[0].Y + m.Columns[1].X) * invS;
    float z = (m.Columns[0].Z + m.Columns[2].X) * invS;
    float w = (m.Columns[2].Y - m.Columns[1].Z) * invS;
    return HMM_Q(x, y, z, w);
  } else if (m.Columns[1].Y > m.Columns[2].Z) {
    float s =
        sqrtf(1.0f + m.Columns[1].Y - m.Columns[0].X - m.Columns[2].Z) * 2.0f;
    float invS = 1.0f / s;
    float x = (m.Columns[0].Y + m.Columns[1].X) * invS;
    float y = 0.25f * s;
    float z = (m.Columns[1].Z + m.Columns[2].Y) * invS;
    float w = (m.Columns[0].Z - m.Columns[2].X) * invS;
    return HMM_Q(x, y, z, w);
  } else {
    float s =
        sqrtf(1.0f + m.Columns[2].Z - m.Columns[0].X - m.Columns[1].Y) * 2.0f;
    float invS = 1.0f / s;
    float x = (m.Columns[0].Z + m.Columns[2].X) * invS;
    float y = (m.Columns[1].Z + m.Columns[2].Y) * invS;
    float z = 0.25f * s;
    float w = (m.Columns[1].X - m.Columns[0].Y) * invS;
    return HMM_Q(x, y, z, w);
  }
}

Transform transform_from_mat4(mat4 *m) {
  convertm4(m);
  Transform result = transform_default();
  vec3 translation =
      HMM_V3(m->Elements[3][0], m->Elements[3][1], m->Elements[3][2]);
  mat3 i = {
      .Elements = {{m->Elements[0][0], m->Elements[0][1], m->Elements[0][2]},
                   {m->Elements[1][0], m->Elements[1][1], m->Elements[1][2]},
                   {m->Elements[2][0], m->Elements[2][1], m->Elements[2][2]}}};

  float sx = HMM_LenV3(i.Columns[0]);
  float sy = HMM_LenV3(i.Columns[1]);
  float sz = signum(HMM_DeterminantM3(i)) * HMM_LenV3(i.Columns[2]);
  vec3 scale = HMM_V3(sx, sy, sz);
  i.Columns[0] = HMM_MulV3F(i.Columns[0], 1.0 / sx);
  i.Columns[1] = HMM_MulV3F(i.Columns[1], 1.0 / sy);
  i.Columns[2] = HMM_MulV3F(i.Columns[2], 1.0 / sz);
  quat r = mat3_to_quat(i);

  result.position = translation;
  result.scale = scale;
  result.rotation = r;

  return result;
}
#endif

#if 1
mat4 transform_to_mat4(Transform t) {
  mat4 out;
  out = HMM_Scale(t.scale);
  // NOTE THIS IS FOR LEFT HANDED COORDINATE SYSTEM
  out = HMM_MulM4(HMM_QToM4(t.rotation), out);
  out = HMM_MulM4(HMM_Translate(t.position), out);
  return out;
}
#else
// original
mat4 transform_to_mat4(Transform t) {
  // First, extract the rotation basis of the transform
  //   vec3 x = t.rotation * vec3(1, 0, 0);
  //   vec3 y = t.rotation * vec3(0, 1, 0);
  //   vec3 z = t.rotation * vec3(0, 0, 1);
  vec3 x = quat_mul_v3(t.rotation, HMM_V3(1.0, 0.0, 0.0));
  vec3 y = quat_mul_v3(t.rotation, HMM_V3(0.0, 1.0, 0.0));
  vec3 z = quat_mul_v3(t.rotation, HMM_V3(0.0, 0.0, 1.0));

  // Next, scale the basis vectors
  x = HMM_MulV3F(x, t.scale.X);
  y = HMM_MulV3F(y, t.scale.Y);
  z = HMM_MulV3F(z, t.scale.Z);

  // Extract the position of the transform
  vec3 p = t.position;

  // Create matrix
  //   return (HMM_Mat4){.Columns = {{x.X, y.X, z.X, p.X},
  //                                 {x.Y, y.Y, z.Y, p.Y},
  //                                 {x.Z, y.Z, z.Z, p.Z},
  //                                 {0, 0, 0, 1}}};
  return mat4_new(x.X, x.Y, x.Z, 0, // X basis (& Scale)
                  y.X, y.Y, y.Z, 0, // Y basis (& scale)
                  z.X, z.Y, z.Z, 0, // Z basis (& scale)
                  p.X, p.Y, p.Z, 1  // Position
  );
}
#endif

// Some important things to note. Each joint has a transform and a parent.
//
// A transform is a slimmer representation of a linear transformation, they
// get turned into matrices when handed over to the GPU.
//
// The transform takes vertices from its parent's coordinate space, and
// transforms them into the joint's coordinate space.
//
// If you concatenate a joint's transform with its parent's and its parent's
// parent's and so on, you will get a transformation that transforms a vertex
// in the joint's coordinate space to a vertex in object/model space. In this
// code we call that a joint's global transformation.
//
// Another important transformation is the one carried by a joint's `inverse
// bind matrix`. It basically does the opposite of a joint's global
// transformation, it transforms vertices in object/model space to vertices in
// the joint's coordinate space.
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

void pose_init(Pose *pose, u32 joints_len) {
  pose->joints = (Transform *)malloc(sizeof(Transform) * joints_len);
  pose->parents = (int *)malloc(sizeof(int) * joints_len);
  pose->len = joints_len;
}

void pose_deinit(Pose *pose) {
  free(pose->joints);
  free(pose->parents);
  pose->joints = NULL;
  pose->parents = NULL;
  pose->len = 0;
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
// bone. Remember, transform concatenation is carried out from right to left.
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
    out.joints[i] = transform;
    int parent = cgltf_get_node_index(node->parent, data->nodes, bone_count);
    out.parents[i] = parent;
  }

  return out;
}

// glTF stores inverse bind pose matrix for each joint. We want bind pose.To
// do this we load the rest pose and invert each joint's matrix. Inverting an
// inverse matrix returns the original matrix.
//
// We default to the joint's world space / global matrix in the case that the
// skin didn't specify an inverse bind matrix.
//
// Finally, we have to make sure that the inverse bind matrix of each joint is
// local to the space of its parent.
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
      Transform bind_transform = transform_from_mat4(&bind_matrix);
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

// When passing to GPU, pose needs to be converted to a linear array of
// matrices
void pose_get_matrix_palette(const Pose *pose, mat4array_t *matrices) {
  safecheckf(matrices->len == 0, "matrices len is 0 (got %zu)\n",
             matrices->len);

  if (matrices->cap != pose->len) {
    array_reserve(mat4, matrices, pose->len);
  }

  for (u32 i = 0; i < pose->len; i++) {
    Transform t = pose_get_global_transform(pose, i);
    matrices->ptr[matrices->len++] = transform_to_mat4(t);
  }
}

bool pose_eq(const Pose *a, const Pose *b) {
  if (a->len != b->len)
    return false;

  for (u32 i = 0; i < a->len; i++) {
    if (!(transform_eq(&a->joints[i], &b->joints[i]) &&
          a->parents[i] == b->parents[i])) {
      return false;
    }
  }

  return true;
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
  // ivec4 *influences;
  u8vec4 *influences;
  vec3 *skinned_positions;
  vec3 *skinned_normals;
  u32 vertices_len;

  u32array_t indices;

  mat4array_t pose_palette;
} BoneMesh;

typedef array_type(BoneMesh) bonemesh_array_t;

#define M4V4D(m, mRow, x, y, z, w)                                             \
  x *m.Elements[0][mRow] + y *m.Elements[1][mRow] + z *m.Elements[2][mRow] +   \
      w *m.Elements[3][mRow]

vec3 test_this_way(mat4 m, vec3 v) {
  return HMM_V3(M4V4D(m, 0, v.X, v.Y, v.Z, 1.0f),
                M4V4D(m, 1, v.X, v.Y, v.Z, 1.0f),
                M4V4D(m, 2, v.X, v.Y, v.Z, 1.0f));
}

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
    //     vec4_truncate(HMM_MulM4V4(skin, vec3_to_vec4_vec(mesh->norms[i])));

    mesh->skinned_positions[i] = test_this_way(skin, mesh->positions[i]);
    mesh->skinned_normals[i] = test_this_way(skin, mesh->norms[i]);
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

static inline vec3 convert(vec3 in) {
#ifdef LEFT_HANDED
  return HMM_V3(in.X, in.Z, -in.Y);
  // return HMM_V3(in.X, -in.Y, in.Z);
#else
  return in;
#endif
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
      out_mesh->positions[i] = convert(HMM_V3(
          values.ptr[index], values.ptr[index + 1], values.ptr[index + 2]));
      break;
    case cgltf_attribute_type_normal:
      out_mesh->norms[i] = convert(HMM_V3(
          values.ptr[index], values.ptr[index + 1], values.ptr[index + 2]));
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
      // information about the skin that is being parsed. Add +0.5f to
      // round, since we can't read ints
      // ivec4 joints = {.v = {(int)(values.ptr[index + 0] + 0.5f),
      //                       (int)(values.ptr[index + 1] + 0.5f),
      //                       (int)(values.ptr[index + 2] + 0.5f),
      //                       (int)(values.ptr[index + 3] + 0.5f)}};
      u8vec4 joints = {.v = {(int)(values.ptr[index + 0] + 0.5f),
                             (int)(values.ptr[index + 1] + 0.5f),
                             (int)(values.ptr[index + 2] + 0.5f),
                             (int)(values.ptr[index + 3] + 0.5f)}};

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

      array_push(BoneMesh, &result, mesh);
    }
  }

  return result;
}

Transform cgltf_get_local_transform(cgltf_node *n) {
  Transform result = transform_default();

  if (n->has_matrix) {
    mat4 mat =
        mat4_new(n->matrix[0], n->matrix[1], n->matrix[2], n->matrix[3],
                 n->matrix[4], n->matrix[5], n->matrix[6], n->matrix[7],
                 n->matrix[8], n->matrix[9], n->matrix[10], n->matrix[11],
                 n->matrix[12], n->matrix[13], n->matrix[14], n->matrix[15]);
    result = transform_from_mat4(&mat);
  }
  if (n->has_translation) {
    result.position = convert(
        HMM_V3(n->translation[0], n->translation[1], n->translation[2]));
  }
  if (n->has_rotation) {
    result.rotation =
        HMM_Q(n->rotation[0], n->rotation[1], n->rotation[2], n->rotation[3]);
  }

  if (n->has_scale) {
    result.scale = convert(HMM_V3(n->scale[0], n->scale[1], n->scale[2]));
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

static struct {
  float rx, ry;
  sg_pipeline pip;
  sg_bindings bind;
  BoneMesh bm;
  Skeleton sk;
  Pose animated_pose;
  mat4array_t pose_palette;
  mat4array_t inv_bind_pose;
  vs_params_t vs_params;
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
  HMM_Mat4 view = HMM_LookAt((HMM_Vec3){.X = 0.0f, .Y = 1.5f, .Z = 6.0f},
                             (HMM_Vec3){.X = 0.0f, .Y = 0.0f, .Z = 0.0f},
                             (HMM_Vec3){.X = 0.0f, .Y = 1.0f, .Z = 0.0f});
  state.rx += 1.0f * t;
  state.ry += 2.0f * t;
  // HMM_Mat4 rxm = HMM_Rotate(state.rx, (HMM_Vec3){{1.0f, 0.0f, 0.0f}});
  // HMM_Mat4 rym = HMM_Rotate(state.ry, (HMM_Vec3){{0.0f, 1.0f, 0.0f}});

  HMM_Mat4 model = HMM_MulM4(HMM_Translate(HMM_V3(-0.5, 0.0, -0.5)),
                             HMM_Scale(HMM_V3(0.01, .01, .01)));

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

void init(void) {
  sg_setup(&(sg_desc){
      .context = sapp_sgcontext(),
      .logger.func = slog_func,
  });

  cgltf_options opts = {};
  ZERO(opts);
  cgltf_data *data = NULL;
  // const char *path = "./src/assets/Woman.gltf";
  const char *path = "./src/assets/scene.gltf";
  // const char *path = "./src/assets/simple_skin.gltf";
  if (cgltf_parse_file(&opts, path, &data) != cgltf_result_success) {
    printf("Failed to parse scene\n");
    exit(1);
  }
  printf("Parsed file\n");

  if (cgltf_load_buffers(&opts, data, path) != cgltf_result_success) {
    cgltf_free(data);
    printf("Failed to load buffers\n");
    exit(1);
  }
  printf("Loaded buffers\n");

  if (cgltf_validate(data) != cgltf_result_success) {
    cgltf_free(data);
    printf("glTF data validation failed!\n");
    exit(1);
  }
  printf("Validated glTF\n");

  bonemesh_array_t meshes = bonemesh_load_meshes(data);
  state.bm = meshes.ptr[0];
  state.sk = skeleton_load(data);
  pose_cpy(&state.sk.rest_pose, &state.animated_pose);
  state.pose_palette = array_empty(mat4array_t);
  state.inv_bind_pose = (mat4array_t){
      .ptr = state.sk.inv_bind_pose,
      .len = state.sk.joints_len,
      .cap = state.sk.joints_len,
  };

  printf("VERTICES LEN: %d\n", state.bm.vertices_len);
  printf("INDICES LEN: %zu\n", state.bm.indices.len);

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
      // .face_winding = SG_FACEWINDING_CCW,
      .cull_mode = SG_CULLMODE_FRONT,
      .face_winding = SG_FACEWINDING_CCW,
      // .cull_mode = SG_CULLMODE_FRONT,
      // .face_winding = SG_FACEWINDING_CW,
      // .cull_mode = SG_CULLMODE_FRONT,
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

  state.bind.fs.images[SLOT_tex] =
      sg_make_image(&(sg_image_desc){.width = 4,
                                     .height = 4,
                                     .data.subimage[0][0] = SG_RANGE(pixels),
                                     .label = "cube-texture"});

  // create a sampler object with default attributes
  state.bind.fs.samplers[SLOT_smp] = sg_make_sampler(&(sg_sampler_desc){0});

  set_vs_params();
}

void frame(void) {
  const float w = sapp_widthf();
  const float h = sapp_heightf();

  fs_params_t fs_params;
  fs_params.light[0] = 1.0;
  fs_params.light[1] = 1.0;
  fs_params.light[2] = 1.0;

  //   const float t = (float)(sapp_frame_duration() * 60.0);
  const float t = (float)(sapp_frame_duration());

  HMM_Mat4 proj = HMM_Perspective(60.0f, w / h, 0.01f, 1000.0f);
  // const float aspect_ratio = w / h;
  // HMM_Mat4 proj = HMM_Orthographic(-aspect_ratio, aspect_ratio, -1, 1, -1,
  // 10);
  HMM_Mat4 view = HMM_LookAt((HMM_Vec3){.X = 0.0f, .Y = 5.0f, .Z = 7.0f},
                             (HMM_Vec3){.X = 0.0f, .Y = 3.0f, .Z = 0.0f},
                             (HMM_Vec3){.X = 0.0f, .Y = 1.0f, .Z = 0.0f});
  state.ry += 1.0f * t;
  state.rx += 1.0f * t;
  HMM_Mat4 rym = HMM_Rotate(state.ry, (HMM_Vec3){{0.0f, 1.0f, 0.0f}});
  HMM_Mat4 rxm = HMM_Rotate(state.rx, (HMM_Vec3){{1.0f, 0.0f, 0.0f}});
  HMM_Mat4 model = HMM_MulM4(HMM_Translate(HMM_V3(-0.0, 0.0, -10.5)),
                             HMM_MulM4(rym, HMM_Scale(HMM_V3(0.1, 0.1, 0.1))));

  // model = HMM_M4D(1.0);

  // HMM_Mat4 model = HMM_MulM4(HMM_Translate(HMM_V3(-0.0, 0.0, -10.5)),
  //                            HMM_Scale(HMM_V3(.1, .1, .1)));

  // HMM_Mat4 model = HMM_MulM4(
  //     HMM_Translate(HMM_V3(-0.0, 0.0, -5.5)),
  //     //  HMM_Scale(HMM_V3(0.5, 0.5, 0.5)));
  //     HMM_MulM4(HMM_MulM4(rxm, rym), HMM_Scale(HMM_V3(0.1, 0.1, 0.1))));

  state.vs_params.model = model;
  state.vs_params.view = view;
  state.vs_params.projection = proj;

  sg_pass_action pass_action = {
      .colors[0] = {.load_action = SG_LOADACTION_CLEAR,
                    .clear_value = {0.25f, 0.5f, 0.75f, 1.0f}}};
  sg_begin_default_pass(&pass_action, (int)w, (int)h);
  sg_apply_pipeline(state.pip);
  sg_apply_bindings(&state.bind);
  sg_apply_uniforms(SG_SHADERSTAGE_VS, SLOT_vs_params,
                    &SG_RANGE(state.vs_params));
  sg_apply_uniforms(SG_SHADERSTAGE_FS, SLOT_fs_params, &SG_RANGE(fs_params));
  // SG_PRIMITIVETYPE_TRIANGLES
  // sg_draw(0, state.bm.indices.len, state.bm.vertices_len / 3);
  // sg_draw(0, state.bm.vertices_len * 3, 1);
  // sg_draw(0, state.bm.indices.len, 1);
  // sg_draw(0, 3, state.bm.vertices_len / 3);
  sg_draw(0, state.bm.indices.len, 1);
  sg_end_pass();
  sg_commit();
}

void cleanup(void) { sg_shutdown(); }

sapp_desc sokol_main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  return (sapp_desc){
      .init_cb = init,
      .frame_cb = frame,
      .cleanup_cb = cleanup,
      // .event_cb = __dbgui_event,
      .width = 800,
      .height = 600,
      .sample_count = 4,
      .window_title = "Cube (sokol-app)",
      .icon.sokol_default = true,
      .logger.func = slog_func,
  };
}

ASSUME_NONNULL_END