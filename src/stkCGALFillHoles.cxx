#include "stkCGALFillHoles.h"

//---------VTK----------------------------------
#include <vtkCellTypes.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

//--------Module-----------------------------
#include "stkCGALUtilities.h"

#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;

typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;

vtkStandardNewMacro(stkCGALFillHoles);

// ----------------------------------------------------------------------------
bool IsSmallHole(halfedge_descriptor h, Mesh mesh, double max_hole_diam)
{
  CGAL::Bbox_3 hole_bbox;
  for (halfedge_descriptor hc : CGAL::halfedges_around_face(h, mesh))
  {
    const Point& p = mesh.point(CGAL::target(hc, mesh));
    hole_bbox += p.bbox();
    
    // Exit early, to avoid unnecessary traversal of large holes
    double bb_diagonal = std::pow(hole_bbox.xmax() - hole_bbox.xmin(), 2) +
      std::pow(hole_bbox.ymax() - hole_bbox.ymin(), 2) +
      std::pow(hole_bbox.zmax() - hole_bbox.zmin(), 2);

    if (bb_diagonal > std::pow(max_hole_diam, 2))
    {
      return false;
    }
  }
  return true;
}

// ----------------------------------------------------------------------------
int stkCGALFillHoles::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** vtkNotUsed(inputVector), vtkInformationVector* outputVector)
{
  vtkPolyData* input = this->GetInputPolyData();

  vtkPolyData* output = vtkPolyData::GetData(outputVector, 0);

  if (input == nullptr)
  {
    vtkErrorMacro("Input is empty");
    return 0;
  }

  auto cellsType = vtkSmartPointer<vtkCellTypes>::New();
  input->GetCellTypes(cellsType);
  int numCellTypes = cellsType->GetNumberOfTypes();

  if (numCellTypes > 1 || numCellTypes == 0)
  {
    vtkErrorMacro("Invalid Input. It should consist of only Triangle Cells");
    return 0;
  }
  else if (cellsType->GetCellType(0) != VTK_TRIANGLE)
  {
    vtkErrorMacro("Input must be a Triangular Surface Mesh (PolyData with only Triangle Cells)");
    return 0;
  }

  // Convert Input PolyData to Surface Mesh
  Mesh mesh;
  stkCGALUtilities::vtkPolyDataToPolygonMesh(input, mesh);

  std::vector<halfedge_descriptor> border_cycles;
  // collect one halfedge per boundary cycle
  CGAL::Polygon_mesh_processing::extract_boundary_cycles(mesh, std::back_inserter(border_cycles));

  Mesh temp_Mesh;
  unsigned int num_patch_skipped = 0;

  // Incrementally fill the holes that are no larger than given diameter (if specified).
  for (halfedge_descriptor h : border_cycles)
  {
    if (this->EnableMaxHoleBBDiagonal)
    {
      if (!IsSmallHole(h, mesh, this->MaxHoleBBDiagonal))
      {
        continue;
      }
    }

    if (this->SkipSelfIntersectingPatches)
    {
      temp_Mesh = mesh;
    }

    std::vector<face_descriptor> patch_facets;
    std::vector<vertex_descriptor> patch_vertices;

    switch (this->FillingType)
    {
      case FillingTypes::FILLHOLES:
        CGAL::Polygon_mesh_processing::triangulate_hole(mesh, h, std::back_inserter(patch_facets),
          CGAL::Polygon_mesh_processing::parameters::use_delaunay_triangulation(
            this->UseDelaunayTriangulation));
        break;

      case FillingTypes::FILLHOLES_REFINE:
        CGAL::Polygon_mesh_processing::triangulate_and_refine_hole(mesh, h,
          std::back_inserter(patch_facets), std::back_inserter(patch_vertices),
          CGAL::Polygon_mesh_processing::parameters::use_delaunay_triangulation(
            this->UseDelaunayTriangulation)
            .density_control_factor(this->DensityControlFactor));
        break;

      case FillingTypes::FILLHOLES_REFINE_FAIR:
        CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(mesh, h,
          std::back_inserter(patch_facets), std::back_inserter(patch_vertices),
          CGAL::Polygon_mesh_processing::parameters::use_delaunay_triangulation(
            this->UseDelaunayTriangulation)
            .density_control_factor(this->DensityControlFactor)
            .fairing_continuity(this->FairingContinuity));
        break;

      default:
        vtkErrorMacro("Invalid Filling Type selected for the algorithm");
        return 0;
    }

    if (this->SkipSelfIntersectingPatches)
    {
      if (CGAL::Polygon_mesh_processing::does_self_intersect(patch_facets, mesh))
      {
        mesh = temp_Mesh;
        num_patch_skipped++;
      }
    }
  }

  if (num_patch_skipped > 0)
  {
    vtkWarningMacro(
      "Skipped Addition of " << num_patch_skipped << " patches because of self-intersections");
  }

  // Convert Polyhedron to Output PolyData
  stkCGALUtilities::SurfaceMeshToPolyData(mesh, output);

  return 1;
}