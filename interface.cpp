#include "interface.h"

#include <cstdlib>
#include <iterator>

#include "walnut/boolean_operation_filter.h"
#include "walnut/bsp_tree.h"
#include "walnut/convex_polygon_factory.h"

namespace walnut {

constexpr bool is_power_of_2(size_t i) {
  while (i > 1) {
    i /= 2;
  }
  return i == 1;
}

class OffsetCalculator {
 public:
  OffsetCalculator() : offset_(0) { }

  template <typename T>
  void Align() {
    static_assert(
        is_power_of_2(alignof(T)),
        "OffsetCalculator only works for types with a power of 2 alignment.");
    static_assert(
        sizeof(T) % alignof(T) == 0,
        "The size of the type must be a multiple of the type's alignment, "
        "because OffsetCalculator does not insert padding between elements of "
        "an array.");
    offset_ += alignof(T) - 1;
    offset_ &= ~(alignof(T) - 1);
  }

  template <typename T>
  void Append(size_t count) {
    offset_ += sizeof(T) * count;
  }
  constexpr size_t offset() const {
    return offset_;
  }

 private:
  size_t offset_;
};

OutputBuffer::~OutputBuffer() {
  free(data);
}

void OutputBuffer::EnsureCapacity(size_t required) {
  if (capacity < required) {
    free(data);
    data = reinterpret_cast<char *>(std::malloc(required));
    capacity = required;
  }
}

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

MeshPlaneRepairer<>* AllocateMeshRepairer() {
  return new MeshPlaneRepairer<>();
}

void FreeMeshRepairer(MeshPlaneRepairer<>* repairer) {
  delete repairer;
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
      polygon.ReducePlaneNormal();
      target_.push_back(std::move(polygon));
    }

   private:
    std::vector<MutableConvexPolygon<>>& target_;
  };
  Collector polygon_factory(target);

  polygon_factory.Build(/*begin=*/source.begin(),
                        /*end=*/source.end());
}

void AddDoublePolygonToMeshRepairer(size_t source_vertex_count,
                                    const double* source_vertices,
                                    MeshPlaneRepairer<>* repairer,
                                    int min_exponent) {
  const double* source_vertices_end = source_vertices +
                                      3 * source_vertex_count;
  for (const double* pos = source_vertices; pos < source_vertices_end;
       pos += 3) {
    repairer->AddVertex(HomoPoint3::FromDoubles(min_exponent,
                                                pos[0],
                                                pos[1],
                                                pos[2]));
  }
  repairer->FinishFacet();
}

void AddFloatPolygonToMeshRepairer(size_t source_vertex_count,
                                   const float* source_vertices,
                                   MeshPlaneRepairer<>* repairer,
                                   int min_exponent) {
  const float* source_vertices_end = source_vertices +
                                      3 * source_vertex_count;
  for (const float* pos = source_vertices; pos < source_vertices_end;
       pos += 3) {
    repairer->AddVertex(HomoPoint3::FromDoubles(min_exponent,
                                                pos[0],
                                                pos[1],
                                                pos[2]));
  }
  repairer->FinishFacet();
}

void FinalizeMeshFromRepairer(MeshPlaneRepairer<>* repairer,
                              std::vector<MutableConvexPolygon<>>* target) {
  class Collector : public ConvexPolygonFactory<HomoPoint3> {
   public:
    using ConvexPolygonFactory<HomoPoint3>::ConvexPolygonRep;

    Collector(std::vector<MutableConvexPolygon<>>& target)
      : target_(target) { }

   protected:
    void Emit(ConvexPolygonRep&& polygon) override {
      polygon.ReducePlaneNormal();
      target_.push_back(std::move(polygon));
    }

   private:
    std::vector<MutableConvexPolygon<>>& target_;
  };
  Collector polygon_factory(*target);

  MeshPlaneRepairerProducer<> producer = std::move(*repairer).FinalizeMesh();
  while (producer.HasMorePolygons()) {
    using VertexIterator = MeshPlaneRepairerProducer<>::VertexIterator;
    VertexIterator first, last;
    HalfSpace3 plane(producer.GetNextPolygon(first, last));
    for (auto it = first; it != last; ++it) {
      assert(plane.IsCoincident(*it));
    }
    polygon_factory.Build(std::move(plane), first, last);
  }
  repairer->Clear();
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

bool IdentityFilter(const std::vector<MutableConvexPolygon<>>* source,
                    std::vector<MutableConvexPolygon<>>* target) {
  BSPTree<> tree;

  BSPContentId id1 = tree.AllocateId();
  tree.AddContents(id1, *source);

  PolygonFilter filter(id1);

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

void GetPolygonVertices(
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

template <typename PolygonRep>
DoublePolygonArray* GetDoublePolygonArrayFromMeshTemplated(
    const std::vector<PolygonRep>* mesh,
    DoublePolygonArray* output) {
  if (output == nullptr) {
    output = new DoublePolygonArray;
  }
  {
    OffsetCalculator size;
    size.Align<DoublePlane>();
    size.Append<DoublePlane>(mesh->size());
    size.Align<size_t>();
    size.Append<size_t>(mesh->size());
    size.Align<DoublePoint3>();
    for (auto it = mesh->begin(); it != mesh->end(); ++it) {
      size.Append<DoublePoint3>(it->vertex_count());
    }
    output->buffer.EnsureCapacity(size.offset());
  }

  output->polygon_count = mesh->size();
  OffsetCalculator offset;
  offset.Align<DoublePlane>();
  output->planes =
    reinterpret_cast<DoublePlane*>(output->buffer.data + offset.offset());
  offset.Append<DoublePlane>(mesh->size());
  offset.Align<size_t>();
  output->vertex_counts =
    reinterpret_cast<size_t*>(output->buffer.data + offset.offset());
  offset.Append<size_t>(mesh->size());
  offset.Align<DoublePoint3>();
  output->vertices =
    reinterpret_cast<DoublePoint3*>(output->buffer.data + offset.offset());

  size_t polygon_index = 0;
  size_t vertex_index = 0;
  for (auto it = mesh->begin(); it != mesh->end(); ++it, ++polygon_index) {
    const HalfSpace3& input_plane = it->plane();
    DoublePlane& output_plane = output->planes[polygon_index];
    output_plane.x = (double)input_plane.x();
    output_plane.y = (double)input_plane.y();
    output_plane.z = (double)input_plane.z();
    output_plane.d = (double)input_plane.d();

    output->vertex_counts[polygon_index] = it->vertex_count();
    for (auto vertex_it = it->vertices_begin();
         vertex_it != it->vertices_end();
         ++vertex_it) {
      output->vertices[vertex_index] = vertex_it->GetDoublePoint3();
      vertex_index++;
    }
  }
  return output;
}

DoublePolygonArray* GetDoublePolygonArrayFromMesh(
    const std::vector<MutableConvexPolygon<>>* mesh,
    DoublePolygonArray* output) {
  return GetDoublePolygonArrayFromMeshTemplated(mesh, output);
}

void FreeDoublePolygonArray(DoublePolygonArray* array) {
  delete array;
}

void AddPolygonToTree(BSPContentId id,
                      const std::vector<HomoPoint3>& source,
                      BSPTree<>& tree) {
  class Collector : public ConvexPolygonFactory<HomoPoint3> {
   public:
    using ConvexPolygonFactory<HomoPoint3>::ConvexPolygonRep;

    Collector(BSPContentId id, BSPTree<>& tree)
      : id_(id), tree_(tree) { }

   protected:
    void Emit(ConvexPolygonRep&& polygon) override {
      polygon.ReducePlaneNormal();
      tree_.root.AddRootContent(id_, std::move(polygon));
    }

   private:
    BSPContentId id_;
    BSPTree<>& tree_;
  };
  Collector polygon_factory(id, tree);

  polygon_factory.Build(/*begin=*/source.begin(),
                        /*end=*/source.end());
  tree.root.PushContentsToLeaves();
}

BSPTree<>* AddDoublePolygonToTree(BSPContentId id, size_t source_vertex_count,
                            const double* source_vertices,
                            int min_exponent,
                            std::vector<HomoPoint3>* temp_buffer,
                            BSPTree<>* tree) {
  if (tree == nullptr) {
    tree = new BSPTree<>;
  }
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

  AddPolygonToTree(id, *temp_buffer, *tree);
  return tree;
}

void FreeTree(BSPTree<>* tree) {
  delete tree;
}

BSPContentId* AllocateIdArray(size_t size) {
  return new BSPContentId[size];
}

void FreeIdArray(BSPContentId* ids) {
  delete[] ids;
}

DoublePolygonArray* EMSCRIPTEN_KEEPALIVE IntersectInTree(
    BSPTree<>* tree, const BSPContentId* ids, size_t id_count,
    DoublePolygonArray* output) {

  auto error_log = [](const std::string& error) {
    std::cerr << "Walnut error: " << error << std::endl;
  };

  IntersectIdsFilter filter(std::vector<BSPContentId>(ids, ids + id_count));
  ConnectingVisitor<BooleanOperationFilter> visitor(filter, error_log);
  tree->Traverse(visitor);
  visitor.FilterEmptyPolygons();

  return GetDoublePolygonArrayFromMeshTemplated(&visitor.polygons(), output);
}

} // walnut
