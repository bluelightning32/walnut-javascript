#ifndef WALNUT_JAVASCRIPT_INTERFACE_H__
#define WALNUT_JAVASCRIPT_INTERFACE_H__

#include <emscripten/emscripten.h>
#include <vector>

#include "walnut/homo_point3.h"
#include "walnut/mutable_convex_polygon.h"

namespace walnut {

extern "C" {

double* EMSCRIPTEN_KEEPALIVE AllocateDoubleVertexArray(size_t max_vertices);

void EMSCRIPTEN_KEEPALIVE FreeDoubleVertexArray(double* vertices);

double* EMSCRIPTEN_KEEPALIVE ResizeDoubleVertexArray(double* vertices,
                                                     size_t new_max_vertices);

std::vector<HomoPoint3>* EMSCRIPTEN_KEEPALIVE AllocateTempVertexBuffer();

void EMSCRIPTEN_KEEPALIVE FreeTempVertexBuffer(std::vector<HomoPoint3>* v);

std::vector<MutableConvexPolygon<>>* EMSCRIPTEN_KEEPALIVE AllocateMesh(
    size_t min_size);

void EMSCRIPTEN_KEEPALIVE FreeMesh(std::vector<MutableConvexPolygon<>>* mesh);

void EMSCRIPTEN_KEEPALIVE AddPolygonToMesh(
    size_t source_vertex_count, const double* source_vertices,
    std::vector<HomoPoint3>* temp_buffer,
    std::vector<MutableConvexPolygon<>>* target, int min_exponent);

// Returns true if successful
bool EMSCRIPTEN_KEEPALIVE UnionMeshes(
    const std::vector<MutableConvexPolygon<>>* source1,
    const std::vector<MutableConvexPolygon<>>* source2,
    std::vector<MutableConvexPolygon<>>* target);

size_t EMSCRIPTEN_KEEPALIVE GetPolygonCount(
    const std::vector<MutableConvexPolygon<>>* mesh);

size_t EMSCRIPTEN_KEEPALIVE GetPolygonVertexCount(
    const std::vector<MutableConvexPolygon<>>* mesh, size_t polygon_index);

void EMSCRIPTEN_KEEPALIVE GetPolygonVertices(
    const std::vector<MutableConvexPolygon<>>* mesh, size_t polygon_index,
    double* output_vertices);

}

} // walnut

#endif // WALNUT_JAVASCRIPT_INTERFACE_H__
