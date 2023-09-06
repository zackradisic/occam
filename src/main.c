// For wasm i think turn off?
// #define HANDMADE_MATH_NO_SSE

#define SOKOL_IMPL
#include <HandmadeMath.h>
#include "common.h"
#include <sokol/sokol_glue.h>
#include "sokol.h"
#include "shaders/cube.glsl.h"
#include "stdio.h"
#include "arena.h"
#include <sokol/sokol_glue.h>
#define CGLTF_IMPLEMENTATION
#include <cgltf.h>
#include "array.h"

ASSUME_NONNULL_BEGIN

#define QUAT_EPSILON 0.000001f
#define VEC3_EPSILON 0.000001f

typedef HMM_Vec2 vec2;
typedef HMM_Vec3 vec3;
typedef HMM_Vec4 vec4;
typedef HMM_Quat quat;
typedef HMM_Mat4 mat4;

typedef array_type(mat4) mat4array_t;
typedef array_type(float) floatarray_t;
typedef array_type(u32) u32array_t;

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
    int x, y, z, w;
  };

  int v[4];

} ivec4;

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

vec3 quat_mul_v3(quat q, vec3 v) {
  // 	return    q.vector * 2.0f * dot(q.vector, v) +
  //   v *(q.scalar * q.scalar - dot(q.vector, q.vector)) +
  //       cross(q.vector, v) * 2.0f * q.scalar;
  vec3 out;

  out = HMM_MulV3F(q.XYZ, 2.0);
  out = HMM_MulV3F(out, HMM_DotV3(q.XYZ, v));
  out = HMM_AddV3(out, HMM_MulV3F(v, q.W * q.W - HMM_DotQ(q, q)));
  out = HMM_AddV3(out, HMM_MulV3F(HMM_MulV3F(HMM_Cross(q.XYZ, v), 2.0), q.W));
  return out;
}

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

// Order is right -> left
Transform transform_combine(Transform a, Transform b) {
  Transform out;
  out.scale = HMM_MulV3(a.scale, b.scale);
  out.rotation = HMM_MulQ(b.rotation, a.rotation);
  out.position = quat_mul_v3(a.rotation, HMM_MulV3(a.scale, b.position));
  out.position = HMM_AddV3(a.position, out.position);
  return out;
}

Transform transform_inverse(Transform t) {
  Transform inv;

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
  vec3 out;
  out = quat_mul_v3(t->rotation, HMM_MulV3(t->scale, p));
  out = HMM_AddV3(t->position, out);
  return out;
}

vec3 transform_vector(const Transform *t, vec3 v) {
  vec3 out;
  out = quat_mul_v3(t->rotation, HMM_MulV3(t->scale, v));
  return out;
}

Transform transform_default() {
  Transform out;
  out.position = HMM_V3(0, 0, 0);
  out.rotation = HMM_Q(0, 0, 0, 1);
  out.scale = HMM_V3(1, 1, 1);
  return out;
}

Transform transform_from_mat4(const mat4 *m) {
  Transform out;

  out.position = HMM_V3(m->Columns[3].X, m->Columns[3].Y, m->Columns[3].Z);
  out.rotation = HMM_M4ToQ_LH(*m);

  mat4 rot_scale_mat =
      mat4_new(m->Columns[0].X, m->Columns[0].Y, m->Columns[0].Z, 0, // col 1
               m->Columns[1].X, m->Columns[1].Y, m->Columns[1].Z, 0, //
               m->Columns[2].X, m->Columns[2].Y, m->Columns[2].Z, 0, //
               0, 0, 0, 1);

  mat4 inv_rot_mat = HMM_QToM4(HMM_InvQ(out.rotation));
  mat4 scale_skew_mat = HMM_MulM4(rot_scale_mat, inv_rot_mat);

  out.scale = (vec3){
      scale_skew_mat.Elements[0][0],
      scale_skew_mat.Elements[1][1],
      scale_skew_mat.Elements[2][2],
  };

  return out;
}

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

// Some important things to note. Each joint has a transform and a parent.
//
// A transform is a slimmer representation of a linear transformation, they get
// turned into matrices when handed over to the GPU.
//
// The transform takes vertices from its parent's coordinate space, and
// transforms them into the joint's coordinate space.
//
// If you concatenate a joint's transform with its parent's and its parent's
// parent's and so on, you will get a transformation that transforms a vertex in
// the joint's coordinate space to a vertex in object/model space. In this code
// we call that a joint's global transformation.
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
  memcpy(dest->joints, src->joints, src->len);
  memcpy(dest->parents, src->parents, src->len);
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
    int parent = cgltf_get_node_index(node, data->nodes, bone_count);
    out.parents[i] = parent;
  }

  return out;
}

// glTF stores inverse bind pose matrix for each joint. We want bind pose. To do
// this we load the rest pose and invert each joint's matrix. Inverting an
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
    for (int j = 0; j < num_joints; j++) {
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

// When passing to GPU, pose needs to be converted to a linear array of matrices
void pose_get_matrix_palette(const Pose *pose, mat4array_t *matrices) {
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
  ivec4 *influences;
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
  pose_get_matrix_palette(p, &palette);
  mat4 *inv_pose_palette = sk->inv_bind_pose;

  for (u32 i = 0; i < num_verts; i++) {
    ivec4 joint = mesh->influences[i];
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
    mesh->skinned_positions[i] = vec4_truncate(
        HMM_MulM4V4(skin, vec3_to_vec4_point(mesh->positions[i])));
    mesh->skinned_normals[i] =
        vec4_truncate(HMM_MulM4V4(skin, vec3_to_vec4_vec(mesh->norms[i])));
  }
}

void bonemesh_cpu_skin1(BoneMesh *mesh, const Skeleton *sk, const Pose *p) {
  u32 num_verts = mesh->vertices_len;
  if (num_verts == 0)
    return;

  Pose bind_pose = sk->bind_pose;

  for (u32 i = 0; i < num_verts; i++) {
    ivec4 *joint = &mesh->influences[i];
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
  if (len == 0)
    return;

  mesh->positions = malloc(sizeof(vec3) * len);
  mesh->texcoords = malloc(sizeof(vec2) * len);
  mesh->norms = malloc(sizeof(vec3) * len);
  mesh->weights = malloc(sizeof(vec4) * len);
  mesh->influences = malloc(sizeof(ivec4) * len);
  mesh->skinned_positions = malloc(sizeof(vec3) * len);
  mesh->skinned_normals = malloc(sizeof(vec3) * len);
  mesh->vertices_len = len;
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
      out_mesh->positions[i] = HMM_V3(values.ptr[index], values.ptr[index + 1],
                                      values.ptr[index + 2]);
      break;
    case cgltf_attribute_type_normal:
      out_mesh->norms[i] = HMM_V3(values.ptr[index], values.ptr[index + 1],
                                  values.ptr[index + 2]);
      break;
    case cgltf_attribute_type_texcoord:
      out_mesh->texcoords[i] = HMM_V2(values.ptr[index], values.ptr[index + 1]);
      break;
    case cgltf_attribute_type_weights:
      out_mesh->weights[i] =
          HMM_V4(values.ptr[index], values.ptr[index + 1],
                 values.ptr[index + 2], values.ptr[index + 3]);
      break;
    case cgltf_attribute_type_joints:
      // These indices are skin relative. This function has no information about
      // the skin that is being parsed. Add +0.5f to round, since we can't read
      // ints
      ivec4 joints = {(int)(values.ptr[index + 0] + 0.5f),
                      (int)(values.ptr[index + 1] + 0.5f),
                      (int)(values.ptr[index + 2] + 0.5f),
                      (int)(values.ptr[index + 3] + 0.5f)};

      // Now convert from being relative to joints array to being relative to
      // the skeleton hierarchy
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
  }
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
          mesh.indices.ptr[k] =
              (u32)cgltf_accessor_read_index(primitive->indices, k);
        }
      }

      array_push(BoneMesh, &result, mesh);
    }
  }

  return result;
}

Transform cgltf_get_local_transform(cgltf_node *n) {
  Transform result = transform_default();

  if (n->has_matrix) {
    mat4 mat = (mat4){.Elements = *n->matrix};
    result = transform_from_mat4(&mat);
  }
  if (n->has_translation) {
    result.position =
        HMM_V3(n->translation[0], n->translation[1], n->translation[2]);
  }
  if (n->has_rotation) {
    result.rotation =
        HMM_Q(n->rotation[0], n->rotation[1], n->rotation[2], n->rotation[3]);
  }

  if (n->has_scale) {
    result.scale = HMM_V3(n->scale[0], n->scale[1], n->scale[2]);
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
    cgltf_accessor_read_float(in_accessor, i, &out->ptr[i * comp_count],
                              comp_count);
  }
  out->len = len;
}

static struct {
  float rx, ry;
  sg_pipeline pip;
  sg_bindings bind;
} state;

typedef struct ModelData {
  const HMM_Vec3 *vertices;
  u32 vertices_len;
} ModelData;

ModelData load_cgltf(cgltf_options *opts, cgltf_data *data) {
  ModelData model_data = {};

  if (cgltf_parse_file(opts, "./src/assets/scene.gltf", &data) !=
      cgltf_result_success) {
    printf("Failed to parse scene\n");
    exit(1);
  }

  if (cgltf_load_buffers(opts, data, "./src/assets/") != cgltf_result_success) {
    cgltf_free(data);
    printf("Failed to load buffers\n");
    exit(1);
  }

  const cgltf_mesh *mesh = &data->meshes[0];
  for (usize i = 0; i < mesh->primitives_count; i++) {
    const cgltf_primitive *prim = &mesh->primitives[i];
    for (usize i = 0; i < prim->attributes_count; i++) {
      const cgltf_attribute *attrib = &prim->attributes[i];
      if (attrib->type == cgltf_attribute_type_position) {
        const cgltf_accessor *accessor = &data->accessors[attrib->index];
        const cgltf_buffer_view *buffer_view = accessor->buffer_view;
        const u8 *buf = cgltf_buffer_view_data(buffer_view);
        model_data.vertices = (const HMM_Vec3 *)buf;
        model_data.vertices_len = accessor->count;
      }
    }
  }

  return model_data;
}

void init(void) {
  sg_setup(&(sg_desc){
      .context = sapp_sgcontext(),
      .logger.func = slog_func,
  });

  cgltf_options opts = {};
  cgltf_data *data = NULL;
  ModelData model_data = load_cgltf(&opts, data);

  /* __dbgui_setup(sapp_sample_count()); */

  /* cube vertex buffer */
  float vertices[] = {
      -1.0, -1.0, -1.0, 1.0,  0.0,  0.0,  1.0,  1.0,  -1.0, -1.0,
      1.0,  0.0,  0.0,  1.0,  1.0,  1.0,  -1.0, 1.0,  0.0,  0.0,
      1.0,  -1.0, 1.0,  -1.0, 1.0,  0.0,  0.0,  1.0,

      -1.0, -1.0, 1.0,  0.0,  1.0,  0.0,  1.0,  1.0,  -1.0, 1.0,
      0.0,  1.0,  0.0,  1.0,  1.0,  1.0,  1.0,  0.0,  1.0,  0.0,
      1.0,  -1.0, 1.0,  1.0,  0.0,  1.0,  0.0,  1.0,

      -1.0, -1.0, -1.0, 0.0,  0.0,  1.0,  1.0,  -1.0, 1.0,  -1.0,
      0.0,  0.0,  1.0,  1.0,  -1.0, 1.0,  1.0,  0.0,  0.0,  1.0,
      1.0,  -1.0, -1.0, 1.0,  0.0,  0.0,  1.0,  1.0,

      1.0,  -1.0, -1.0, 1.0,  0.5,  0.0,  1.0,  1.0,  1.0,  -1.0,
      1.0,  0.5,  0.0,  1.0,  1.0,  1.0,  1.0,  1.0,  0.5,  0.0,
      1.0,  1.0,  -1.0, 1.0,  1.0,  0.5,  0.0,  1.0,

      -1.0, -1.0, -1.0, 0.0,  0.5,  1.0,  1.0,  -1.0, -1.0, 1.0,
      0.0,  0.5,  1.0,  1.0,  1.0,  -1.0, 1.0,  0.0,  0.5,  1.0,
      1.0,  1.0,  -1.0, -1.0, 0.0,  0.5,  1.0,  1.0,

      -1.0, 1.0,  -1.0, 1.0,  0.0,  0.5,  1.0,  -1.0, 1.0,  1.0,
      1.0,  0.0,  0.5,  1.0,  1.0,  1.0,  1.0,  1.0,  0.0,  0.5,
      1.0,  1.0,  1.0,  -1.0, 1.0,  0.0,  0.5,  1.0};

  sg_buffer vbuf = sg_make_buffer(
      &(sg_buffer_desc){.data = SG_RANGE(vertices), .label = "cube-vertices"});

  /* create an index buffer for the cube */
  uint16_t indices[] = {0,  1,  2,  0,  2,  3,  6,  5,  4,  7,  6,  4,
                        8,  9,  10, 8,  10, 11, 14, 13, 12, 15, 14, 12,
                        16, 17, 18, 16, 18, 19, 22, 21, 20, 23, 22, 20};
  sg_buffer ibuf =
      sg_make_buffer(&(sg_buffer_desc){.type = SG_BUFFERTYPE_INDEXBUFFER,
                                       .data = SG_RANGE(indices),
                                       .label = "cube-indices"});

  /* create shader */
  sg_shader shd = sg_make_shader(cube_shader_desc(sg_query_backend()));

  /* create pipeline object */
  state.pip = sg_make_pipeline(&(sg_pipeline_desc){
      .layout =
          {/* test to provide buffer stride, but no attr offsets */
           .buffers[0].stride = 28,
           .attrs = {[ATTR_vs_position].format = SG_VERTEXFORMAT_FLOAT3,
                     [ATTR_vs_color0].format = SG_VERTEXFORMAT_FLOAT4}},
      .shader = shd,
      .index_type = SG_INDEXTYPE_UINT16,
      .face_winding = SG_FACEWINDING_CCW,
      .cull_mode = SG_CULLMODE_BACK,
      .depth =
          {
              .write_enabled = true,
              .compare = SG_COMPAREFUNC_LESS_EQUAL,
          },
      .label = "cube-pipeline"});

  /* setup resource bindings */
  state.bind = (sg_bindings){.vertex_buffers[0] = vbuf, .index_buffer = ibuf};
}

void frame(void) {
  /* NOTE: the vs_params_t struct has been code-generated by the shader-code-gen
   */
  vs_params_t vs_params;
  const float w = sapp_widthf();
  const float h = sapp_heightf();
  //   const float t = (float)(sapp_frame_duration() * 60.0);
  const float t = (float)(sapp_frame_duration());

  HMM_Mat4 proj = HMM_Perspective_LH_ZO(60.0f, w / h, 0.01f, 10.0f);
  HMM_Mat4 view = HMM_LookAt_LH((HMM_Vec3){.X = 0.0f, .Y = 1.5f, .Z = 6.0f},
                                (HMM_Vec3){.X = 0.0f, .Y = 0.0f, .Z = 0.0f},
                                (HMM_Vec3){.X = 0.0f, .Y = 1.0f, .Z = 0.0f});
  HMM_Mat4 view_proj = HMM_MulM4(proj, view);
  state.rx += 1.0f * t;
  state.ry += 2.0f * t;
  HMM_Mat4 rxm = HMM_Rotate_LH(state.rx, (HMM_Vec3){{1.0f, 0.0f, 0.0f}});
  HMM_Mat4 rym = HMM_Rotate_LH(state.ry, (HMM_Vec3){{0.0f, 1.0f, 0.0f}});
  HMM_Mat4 model = HMM_MulM4(rxm, rym);
  //   HMM_Mat4 model = HMM_M4D(1.0);
  vs_params.mvp = HMM_MulM4(view_proj, model);
  //   vs_params.mvp = HMM_MulM4(model, view_proj);

  sg_pass_action pass_action = {
      .colors[0] = {.load_action = SG_LOADACTION_CLEAR,
                    .clear_value = {0.25f, 0.5f, 0.75f, 1.0f}}};
  sg_begin_default_pass(&pass_action, (int)w, (int)h);
  sg_apply_pipeline(state.pip);
  sg_apply_bindings(&state.bind);
  sg_apply_uniforms(SG_SHADERSTAGE_VS, SLOT_vs_params, &SG_RANGE(vs_params));
  sg_draw(0, 36, 1);
  sg_end_pass();
  sg_commit();
}

void cleanup(void) { sg_shutdown(); }

#define M4V4D(m, mRow, x, y, z, w)                                             \
  x *m.Elements[0][mRow] + y *m.Elements[1][mRow] + z *m.Elements[2][mRow] +   \
      w *m.Elements[3][mRow]

vec3 test_this_way(mat4 m, vec4 v) {
  return HMM_V3(M4V4D(m, 0, v.X, v.Y, v.Z, 1.0f),
                M4V4D(m, 1, v.X, v.Y, v.Z, 1.0f),
                M4V4D(m, 2, v.X, v.Y, v.Z, 1.0f));
}

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