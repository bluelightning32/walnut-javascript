#ifndef WALNUT_JAVASCRIPT_INTERFACE_H__
#define WALNUT_JAVASCRIPT_INTERFACE_H__

#include <emscripten/emscripten.h>
#include <vector>

#include "walnut/homo_point3.h"
#include "walnut/mesh_plane_repairer.h"
#include "walnut/mutable_convex_polygon.h"

namespace walnut {

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

}

} // walnut

#endif // WALNUT_JAVASCRIPT_INTERFACE_H__
