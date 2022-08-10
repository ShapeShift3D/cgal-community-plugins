/**
 * @class stkCGALConstrainedDelaunayTriangulation
 * @brief Remeshes a given vtkPolyData
 *
 * This filter performs isotropic remeshing. I takes a vtkPolyData as input, and returns a
 * vtkPolyData containing the remeshed input. An optional input array can be supplied to mask the
 * remeshing zone.
 *
 * Based on vtkIsotropicRemeshingFilter from https://github.com/CGAL/cgal-paraview-plugins
 *
 *
 * When masking is used, the following steps are executed:
 *  - Threshold the regions (inner and outer)
 *  - Remesh the inner region
 *  - Append remeshed and outer regions
 *
 * Note that in such scenario, we don't split border edges to preserve topology.
 * Otherwise the output mesh could be non-manifold.
 *
 *
 * TODO:
 *      - Explore alternative solution to extract and process the remeshing region.
 *        At the moment, a threshold is applied to the input, its result (inner region) is remeshed
 * then the output will be assigned to the remeshed region appended with the outer region (from the
 * threshold). There might be a better approach using CGAL.
 */

#pragma once

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <stkCGALConstrainedDelaunayTriangulationInterface.h>
#include <stkCGALModule.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <iostream>

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
class STKCGAL_EXPORT stkCGALConstrainedDelaunayTriangulation
  : public stkCGALConstrainedDelaunayTriangulationInterface
{
public:
  static stkCGALConstrainedDelaunayTriangulation* New();
  vtkTypeMacro(
    stkCGALConstrainedDelaunayTriangulation, stkCGALConstrainedDelaunayTriangulationInterface);
  void PrintSelf(ostream& os, vtkIndent indent) override;

protected:
  stkCGALConstrainedDelaunayTriangulation() = default;
  ~stkCGALConstrainedDelaunayTriangulation() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  virtual int RequestInformation(
    vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALConstrainedDelaunayTriangulation(const stkCGALConstrainedDelaunayTriangulation&) = delete;
  void operator=(const stkCGALConstrainedDelaunayTriangulation&) = delete;
  void markDomains(CDT& cdt);
  void mark_domains(CDT& ct, Face_handle start, int index, std::list<CDT::Edge>& border);
};
