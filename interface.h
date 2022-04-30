#ifndef WALNUT_JAVASCRIPT_INTERFACE_H__
#define WALNUT_JAVASCRIPT_INTERFACE_H__

#include <emscripten/emscripten.h>
#include <vector>

#include "walnut/bsp_tree.h"
#include "walnut/homo_point3.h"
#include "walnut/mesh_plane_repairer.h"
#include "walnut/mutable_convex_polygon.h"

namespace walnut {

struct DoublePlane {
  double x, y, z, d;
};

struct OutputBuffer {
  OutputBuffer() = default;

  ~OutputBuffer();

  // This wipes out the existing data.
  void EnsureCapacity(size_t required);

  char* data = nullptr;
  size_t capacity = 0;
};

struct DoublePolygonArray {
  size_t polygon_count;
  // Contains the plane of all polygons.
  DoublePlane* planes;
  // Contains the vertex count for each polygon.
  size_t* vertex_counts;
  // Contains the vertices for all polygons. Use `vertex_counts` to find which
  // subarrays map to which polygons.
  DoublePoint3* vertices;
  // Tracks the raw memory used by `planes`, `vertex_counts`, and `vertices`.
  OutputBuffer buffer;
};

extern "C" {

float* EMSCRIPTEN_KEEPALIVE AllocateFloatVertexArray(size_t max_vertices);

void EMSCRIPTEN_KEEPALIVE FreeFloatVertexArray(float* vertices);

float* EMSCRIPTEN_KEEPALIVE ResizeFloatVertexArray(float* vertices,
                                                   size_t new_max_vertices);

double* EMSCRIPTEN_KEEPALIVE AllocateDoubleVertexArray(size_t max_vertices);

void EMSCRIPTEN_KEEPALIVE FreeDoubleVertexArray(double* vertices);

double* EMSCRIPTEN_KEEPALIVE ResizeDoubleVertexArray(double* vertices,
                                                     size_t new_max_vertices);

std::vector<HomoPoint3>* EMSCRIPTEN_KEEPALIVE AllocateTempVertexBuffer();

void EMSCRIPTEN_KEEPALIVE FreeTempVertexBuffer(std::vector<HomoPoint3>* v);

MeshPlaneRepairer<>* EMSCRIPTEN_KEEPALIVE AllocateMeshRepairer();

void EMSCRIPTEN_KEEPALIVE FreeMeshRepairer(MeshPlaneRepairer<>* repairer);

std::vector<MutableConvexPolygon<>>* EMSCRIPTEN_KEEPALIVE AllocateMesh(
    size_t min_size);

void EMSCRIPTEN_KEEPALIVE FreeMesh(std::vector<MutableConvexPolygon<>>* mesh);

void EMSCRIPTEN_KEEPALIVE AddFloatPolygonToMesh(
    size_t source_vertex_count, const float* source_vertices,
    std::vector<HomoPoint3>* temp_buffer,
    std::vector<MutableConvexPolygon<>>* target, int min_exponent);

void EMSCRIPTEN_KEEPALIVE AddDoublePolygonToMesh(
    size_t source_vertex_count, const double* source_vertices,
    std::vector<HomoPoint3>* temp_buffer,
    std::vector<MutableConvexPolygon<>>* target, int min_exponent);

void EMSCRIPTEN_KEEPALIVE AddFloatTrianglesToMesh(
    size_t triangle_count, const float* triangle_vertices,
    std::vector<MutableConvexPolygon<>>* target, int min_exponent);

void EMSCRIPTEN_KEEPALIVE AddDoublePolygonToMeshRepairer(
    size_t source_vertex_count, const double* source_vertices,
    MeshPlaneRepairer<>* repairer, int min_exponent);

void EMSCRIPTEN_KEEPALIVE AddFloatPolygonToMeshRepairer(
    size_t source_vertex_count, const float* source_vertices,
    MeshPlaneRepairer<>* repairer, int min_exponent);

void EMSCRIPTEN_KEEPALIVE FinalizeMeshFromRepairer(
    MeshPlaneRepairer<>* repairer,
    std::vector<MutableConvexPolygon<>>* target);

void EMSCRIPTEN_KEEPALIVE InvertMesh(
    std::vector<MutableConvexPolygon<>>* mesh);

// Returns true if successful
bool EMSCRIPTEN_KEEPALIVE IdentityFilter(
    const std::vector<MutableConvexPolygon<>>* source1,
    std::vector<MutableConvexPolygon<>>* target);

// Returns true if successful
bool EMSCRIPTEN_KEEPALIVE UnionMeshes(
    const std::vector<MutableConvexPolygon<>>* source1,
    const std::vector<MutableConvexPolygon<>>* source2,
    std::vector<MutableConvexPolygon<>>* target);

// Returns true if successful
bool EMSCRIPTEN_KEEPALIVE IntersectMeshes(
    const std::vector<MutableConvexPolygon<>>* source1,
    const std::vector<MutableConvexPolygon<>>* source2,
    std::vector<MutableConvexPolygon<>>* target);

// Returns true if successful
//
// `subtrahend` must be an inverted mesh.
bool EMSCRIPTEN_KEEPALIVE SubtractMesh(
    const std::vector<MutableConvexPolygon<>>* minuend,
    const std::vector<MutableConvexPolygon<>>* subtrahend,
    std::vector<MutableConvexPolygon<>>* result);

size_t EMSCRIPTEN_KEEPALIVE GetPolygonCount(
    const std::vector<MutableConvexPolygon<>>* mesh);

size_t EMSCRIPTEN_KEEPALIVE GetPolygonVertexCount(
    const std::vector<MutableConvexPolygon<>>* mesh, size_t polygon_index);

void EMSCRIPTEN_KEEPALIVE GetPolygonVertices(
    const std::vector<MutableConvexPolygon<>>* mesh, size_t polygon_index,
    double* output_vertices);

size_t EMSCRIPTEN_KEEPALIVE GetTriangleCountInMesh(
    const std::vector<MutableConvexPolygon<>>* mesh);

void EMSCRIPTEN_KEEPALIVE GetFloatTrianglesFromMesh(
    const std::vector<MutableConvexPolygon<>>* mesh,
    float* triangle_vertices);

// Converts `mesh` into a `DoublePolygonArray`.
//
// This returns a pointer to the output `DoublePolygonArray`. If `output` is
// non-null, it is used as the output buffer and returned. Otherwise a new
// `DoublePolygonArray` is allocated and returned.
//
// All `DoublePolygonArray`s should be freed once per allocation using
// `FreeDoublePolygonArray`.
DoublePolygonArray* EMSCRIPTEN_KEEPALIVE GetDoublePolygonArrayFromMesh(
    const std::vector<MutableConvexPolygon<>>* mesh,
    DoublePolygonArray* output);

void EMSCRIPTEN_KEEPALIVE FreeDoublePolygonArray(DoublePolygonArray* array);

// Adds a polygon to `tree`.
//
// This function should be called once for every polygon in the polyhedron that
// the caller wants to add to the tree. That polyhedron should be identified by
// `id`. The caller can make up any id, as long as it is unique relative to the
// other polyhedrons that the caller adds to the tree.
//
// `temp_buffer` is used internally to hold `source_vertices` after the are
// converted to `HomoPoint3`. If `temp_buffer` is too small to hold all of
// `source_vertices`, then this function will resize it.
//
// `source_vertices` are snapped to a grid before adding the polygon. The
// length of each cube in the grid is 2^min_exponent. So a smaller min_exponent
// means more precision. The grid snapping can be disabled by specifying a
// `min_exponent` that is less than or equal to the minimum double exponent:
// -1024.
//
// If `tree` is null, a new tree is allocated. In either case, the target tree
// is returned. Every allocated tree should be freed with `FreeTree`.
BSPTree<>* EMSCRIPTEN_KEEPALIVE AddDoublePolygonToTree(
    BSPContentId id, size_t source_vertex_count, const double* source_vertices,
    int min_exponent, std::vector<HomoPoint3>* temp_buffer, BSPTree<>* tree);

void EMSCRIPTEN_KEEPALIVE FreeTree(BSPTree<>* tree);

BSPContentId* EMSCRIPTEN_KEEPALIVE AllocateIdArray(size_t size);

void EMSCRIPTEN_KEEPALIVE FreeIdArray(BSPContentId* ids);

// Intersects the meshes in `tree` idenified by `ids`.
//
// The polyhedrons should be added to `tree` before calling this function,
// using `AddDoublePolygonToTree`. This function may subdivide polygons within
// `tree`, but `tree` will still represent the same polyhedrons when the
// function ends.
//
// Any polyhedrons that happen to be in `tree` with an id not in `ids` are
// ignored.
//
// This returns a pointer to the output `DoublePolygonArray`. If the passed in
// `output` is non-null, it is used as the output buffer and returned.
// Otherwise a new `DoublePolygonArray` is allocated and returned.
//
// The returned `DoublePolygonArray` must be freed exactly once using
// `FreeDoublePolygonArray`. The returned `DoublePolygonArray` may be reused
// before freeing it.
DoublePolygonArray* EMSCRIPTEN_KEEPALIVE IntersectInTree(
    BSPTree<>* tree, const BSPContentId* ids, size_t id_count,
    DoublePolygonArray* output);

}

} // walnut

#endif // WALNUT_JAVASCRIPT_INTERFACE_H__
