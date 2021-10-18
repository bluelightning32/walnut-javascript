#include "interface.h"

#include <cstdlib>
#include <iterator>

#include "walnut/bsp_tree.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

double* AllocateDoubleVertexArray(size_t max_vertices) {
  return reinterpret_cast<double*>(
      std::malloc(sizeof(double *) * max_vertices * 3));
}

void FreeDoubleVertexArray(double* vertices) {
  std::free(vertices);
}

double* ResizeDoubleVertexArray(double* vertices, size_t new_max_vertices) {
  return reinterpret_cast<double*>(
    std::realloc(vertices, sizeof(double *) * new_max_vertices * 3));
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

void AddPolygonToMesh(size_t source_vertex_count,
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
  Collector polygon_factory(*target);

  polygon_factory.Build(/*begin=*/temp_buffer->begin(),
                        /*end=*/temp_buffer->end());
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
  target->clear();
  auto polygons = visitor.TakePolygons();
  target->reserve(polygons.size());
  for (auto& polygon : polygons) {
    target->emplace_back(std::move(polygon));
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
