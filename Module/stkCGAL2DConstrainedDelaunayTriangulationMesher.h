/**
 * @class stkCGAL2DConstrainedDelaunayTriangulationMesher
 * @brief Generates Constrained Delaunay Triangulation From 2D Input Curves.
 *
 *  Inputs: 2D Polylines (port 0, vtkPolyData)
 *  Output: 2D Mesh (port 0, vtkPolyData)
 *
 * @sa
 * stkCGAL2DConstrainedDelaunayTriangulationMesherInterface
 */

#pragma once

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <stkCGAL2DConstrainedDelaunayTriangulationMesherInterface.h>
#include <stkCGALModule.h>

struct FaceInfo2
{
  FaceInfo2() {}
  int nesting_level;
  bool in_domain() { return nesting_level % 2 == 1; }
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, K> Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> TDS;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;
typedef CDT::Point Point;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CDT::Face_handle Face_handle;

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGAL2DConstrainedDelaunayTriangulationMesher
  : public stkCGAL2DConstrainedDelaunayTriangulationMesherInterface
{
public:
  static stkCGAL2DConstrainedDelaunayTriangulationMesher* New();
  vtkTypeMacro(
    stkCGAL2DConstrainedDelaunayTriangulationMesher, stkCGAL2DConstrainedDelaunayTriangulationMesherInterface);
  void PrintSelf(ostream& os, vtkIndent indent) override;

protected:
  stkCGAL2DConstrainedDelaunayTriangulationMesher() = default;
  ~stkCGAL2DConstrainedDelaunayTriangulationMesher() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGAL2DConstrainedDelaunayTriangulationMesher(const stkCGAL2DConstrainedDelaunayTriangulationMesher&) = delete;
  void operator=(const stkCGAL2DConstrainedDelaunayTriangulationMesher&) = delete;
  void markDomains(CDT& cdt);
  void mark_domains(CDT& ct, Face_handle start, int index, std::list<CDT::Edge>& border);
};
