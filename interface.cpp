#include "interface.h"

#include <cstdlib>
#include <iterator>

#include "walnut/bsp_tree.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

float* AllocateFloatVertexArray(size_t max_vertices) {
  return reinterpret_cast<float*>(
      std::malloc(sizeof(float) * max_vertices * 3));
}

void FreeFloatVertexArray(float* vertices) {
  std::free(vertices);
}

float* ResizeFloatVertexArray(float* vertices, size_t new_max_vertices) {
  return reinterpret_cast<float*>(
    std::realloc(vertices, sizeof(float) * new_max_vertices * 3));
}

double* AllocateDoubleVertexArray(size_t max_vertices) {
  return reinterpret_cast<double*>(
      std::malloc(sizeof(double) * max_vertices * 3));
}

void FreeDoubleVertexArray(double* vertices) {
  std::free(vertices);
}

double* ResizeDoubleVertexArray(double* vertices, size_t new_max_vertices) {
  return reinterpret_cast<double*>(
    std::realloc(vertices, sizeof(double) * new_max_vertices * 3));
}

std::vector<HomoPoint3>* AllocateTempVertexBuffer() {
  return new std::vector<HomoPoint3>();
}

void FreeTempVertexBuffer(std::vector<HomoPoint3>* v) {
  delete v;
}

std::vector<MutableConvexPolygon<>>* AllocateMesh(size_t min_size) {
  auto result = std::make_unique<std::vector<MutableConvexPolygon<>>>();
  result->reserve(min_size);
  return result.release();
}

void FreeMesh(std::vector<MutableConvexPolygon<>>* mesh) {
  delete mesh;
}

void AddFloatTrianglesToMesh(size_t triangle_count,
                             const float* triangle_vertices,
                             std::vector<MutableConvexPolygon<>>* target,
                             int min_exponent) {
  HomoPoint3 vertices[3];
  const float* source_pos = triangle_vertices;
  for (size_t i = 0; i < triangle_count; ++i) {
    for (size_t j = 0; j < 3; ++j, source_pos += 3) {
      vertices[j] = HomoPoint3::FromDoubles(min_exponent,
                                            source_pos[0],
                                            source_pos[1],
                                            source_pos[2]);
    }
    HalfSpace3 plane(vertices[0], vertices[1], vertices[2]);
    int drop_dimension = plane.normal().GetFirstNonzeroDimension();
    target->emplace_back(std::move(plane), drop_dimension,
                         std::move(vertices));
  }
}

size_t GetTriangleCountInMesh(
    const std::vector<MutableConvexPolygon<>>* mesh) {
  size_t triangle_count = 0;
  for (const MutableConvexPolygon<>& polygon : *mesh) {
    // 3 vertices means 1 triangle
    // 4 vertices means 2 triangles, etc...
    triangle_count += polygon.vertex_count() - 2;
  }
  return triangle_count;
}

void GetFloatTrianglesFromMesh(const std::vector<MutableConvexPolygon<>>* mesh,
                               float* triangle_vertices) {
  float* output_pos = triangle_vertices;
  for (const MutableConvexPolygon<>& polygon : *mesh) {
    const DoublePoint3 first = polygon.vertex(0).GetDoublePoint3();
    for (size_t i = 2; i < polygon.vertex_count(); ++i) {
      const DoublePoint3 second = polygon.vertex(i - 1).GetDoublePoint3();
      const DoublePoint3 third = polygon.vertex(i).GetDoublePoint3();

      output_pos[0] = first.x;
      output_pos[1] = first.y;
      output_pos[2] = first.z;
      output_pos[3] = second.x;
      output_pos[4] = second.y;
      output_pos[5] = second.z;
      output_pos[6] = third.x;
      output_pos[7] = third.y;
      output_pos[8] = third.z;

      output_pos += 9;
    }
  }
}

void AddPolygonToMesh(const std::vector<HomoPoint3>& source,
                      std::vector<MutableConvexPolygon<>>& target) {
  class Collector : public ConvexPolygonFactory<HomoPoint3> {
   public:
    using ConvexPolygonFactory<HomoPoint3>::ConvexPolygonRep;

    Collector(std::vector<MutableConvexPolygon<>>& target)
      : target_(target) { }

   protected:
    void Emit(ConvexPolygonRep&& polygon) override {
      target_.push_back(std::move(polygon));
    }

   private:
    std::vector<MutableConvexPolygon<>>& target_;
  };
  Collector polygon_factory(target);

  polygon_factory.Build(/*begin=*/source.begin(),
                        /*end=*/source.end());
}

void AddDoublePolygonToMesh(size_t source_vertex_count,
                            const double* source_vertices,
                            std::vector<HomoPoint3>* temp_buffer,
                            std::vector<MutableConvexPolygon<>>* target,
                            int min_exponent) {
  temp_buffer->clear();
  temp_buffer->reserve(source_vertex_count);
  {
    const double* source_vertices_end = source_vertices +
                                        3 * source_vertex_count;
    for (const double* pos = source_vertices; pos < source_vertices_end;
         pos += 3) {
      temp_buffer->push_back(HomoPoint3::FromDoubles(min_exponent,
                                                     pos[0],
                                                     pos[1],
                                                     pos[2]));
    }
  }

  AddPolygonToMesh(*temp_buffer, *target);
}

void AddFloatPolygonToMesh(size_t source_vertex_count,
                           const float* source_vertices,
                           std::vector<HomoPoint3>* temp_buffer,
                           std::vector<MutableConvexPolygon<>>* target,
                           int min_exponent) {
  temp_buffer->clear();
  temp_buffer->reserve(source_vertex_count);
  {
    const float* source_vertices_end = source_vertices +
                                        3 * source_vertex_count;
    for (const float* pos = source_vertices; pos < source_vertices_end;
         pos += 3) {
      temp_buffer->push_back(HomoPoint3::FromDoubles(min_exponent,
                                                     pos[0],
                                                     pos[1],
                                                     pos[2]));
    }
  }

  AddPolygonToMesh(*temp_buffer, *target);
}

void InvertMesh(std::vector<MutableConvexPolygon<>>* mesh) {
  for (auto& polygon : *mesh) {
    polygon.Invert();
  }
}

bool UnionMeshes(const std::vector<MutableConvexPolygon<>>* source1,
                 const std::vector<MutableConvexPolygon<>>* source2,
                 std::vector<MutableConvexPolygon<>>* target) {
  BSPTree<> tree;

  BSPContentId id1 = tree.AllocateId();
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id1, *source1);
  tree.AddContents(id2, *source2);

  auto filter = MakeUnionFilter(PolygonFilter(id1), PolygonFilter(id2));

  bool errored = false;
  auto error_log = [&errored](const std::string& error) {
    errored = true;
  };

  ConnectingVisitor<decltype(filter), MutableConvexPolygon<>> visitor(filter, error_log);
  tree.Traverse(visitor);
  visitor.FilterEmptyPolygons();
  target->clear();
  auto polygons = visitor.TakePolygons();
  target->reserve(polygons.size());
  for (auto& polygon : polygons) {
    target->emplace_back(std::move(polygon));
  }

  return !errored;
}

bool IntersectMeshes(const std::vector<MutableConvexPolygon<>>* source1,
                     const std::vector<MutableConvexPolygon<>>* source2,
                     std::vector<MutableConvexPolygon<>>* target) {
  BSPTree<> tree;

  BSPContentId id1 = tree.AllocateId();
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id1, *source1);
  tree.AddContents(id2, *source2);

  auto filter = MakeIntersectionFilter(PolygonFilter(id1), PolygonFilter(id2));

  bool errored = false;
  auto error_log = [&errored](const std::string& error) {
    errored = true;
  };

  ConnectingVisitor<decltype(filter), MutableConvexPolygon<>> visitor(filter, error_log);
  tree.Traverse(visitor);
  visitor.FilterEmptyPolygons();
  target->clear();
  auto polygons = visitor.TakePolygons();
  target->reserve(polygons.size());
  for (auto& polygon : polygons) {
    target->emplace_back(std::move(polygon));
  }

  return !errored;
}

bool SubtractMesh(const std::vector<MutableConvexPolygon<>>* minuend,
                  const std::vector<MutableConvexPolygon<>>* subtrahend,
                  std::vector<MutableConvexPolygon<>>* result) {
  BSPTree<> tree;

  BSPContentId id1 = tree.AllocateId();
  BSPContentId id2 = tree.AllocateId();
  tree.AddContents(id1, *minuend);
  tree.AddContents(id2, *subtrahend);

  auto filter = MakeSubtractionFilter(PolygonFilter(id1),
                                      InvertedPolygonFilter(id2));

  bool errored = false;
  auto error_log = [&errored](const std::string& error) {
    errored = true;
  };

  ConnectingVisitor<decltype(filter), MutableConvexPolygon<>> visitor(filter, error_log);
  tree.Traverse(visitor);
  visitor.FilterEmptyPolygons();
  result->clear();
  auto polygons = visitor.TakePolygons();
  result->reserve(polygons.size());
  for (auto& polygon : polygons) {
    result->emplace_back(std::move(polygon));
  }

  return !errored;
}

size_t GetPolygonCount(const std::vector<MutableConvexPolygon<>>* mesh) {
  return mesh->size();
}

size_t GetPolygonVertexCount(const std::vector<MutableConvexPolygon<>>* mesh,
                             size_t polygon_index) {
  return (*mesh)[polygon_index].vertex_count();
}

void EMSCRIPTEN_KEEPALIVE GetPolygonVertices(
    const std::vector<MutableConvexPolygon<>>* mesh, size_t polygon_index,
    double* output_vertices) {
  const MutableConvexPolygon<>& polygon = (*mesh)[polygon_index];

  for (size_t i = 0; i < polygon.vertex_count(); ++i) {
    DoublePoint3 v = polygon.vertex(i).GetDoublePoint3();
    output_vertices[i + 0] = v.x;
    output_vertices[i + 1] = v.y;
    output_vertices[i + 2] = v.z;
  }
}

} // walnut
