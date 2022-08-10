#include "stkCGALFillHoles.h"

//---------VTK----------------------------------
#include <vtkCellTypes.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>

//-------BOOST-----------------------------------
#include <boost/foreach.hpp>

//--------Module-----------------------------
#include "stkCGALUtilities.h"

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef Polyhedron::Halfedge_handle Halfedge_handle;
typedef Polyhedron::Facet_handle Facet_handle;
typedef Polyhedron::Vertex_handle Vertex_handle;

vtkStandardNewMacro(stkCGALFillHoles);

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
  Polyhedron poly;
  stkCGALUtilities::vtkPolyDataToPolygonMesh(input, poly);

  // Incrementally fill the holes
  BOOST_FOREACH (Halfedge_handle h, halfedges(poly))
  {
    if (h->is_border())
    {
      std::vector<Facet_handle> patch_facets;
      std::vector<Vertex_handle> patch_vertices;

      switch (this->FillingType)
      {
        case FillingTypes::FILLHOLES:
          CGAL::Polygon_mesh_processing::triangulate_hole(poly, h, std::back_inserter(patch_facets),
            CGAL::Polygon_mesh_processing::parameters::vertex_point_map(
              get(CGAL::vertex_point, poly))
              .use_delaunay_triangulation(this->UseDelaunayTriangulation)
              .geom_traits(K()));
          break;

        case FillingTypes::FILLHOLES_REFINE:
          CGAL::Polygon_mesh_processing::triangulate_and_refine_hole(poly, h,
            std::back_inserter(patch_facets), std::back_inserter(patch_vertices),
            CGAL::Polygon_mesh_processing::parameters::vertex_point_map(
              get(CGAL::vertex_point, poly))
              .use_delaunay_triangulation(this->UseDelaunayTriangulation)
              .density_control_factor(this->DensityControlFactor)
              .geom_traits(K()));
          break;

        case FillingTypes::FILLHOLES_REFINE_FAIR:
          CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(poly, h,
            std::back_inserter(patch_facets), std::back_inserter(patch_vertices),
            CGAL::Polygon_mesh_processing::parameters::vertex_point_map(
              get(CGAL::vertex_point, poly))
              .use_delaunay_triangulation(this->UseDelaunayTriangulation)
              .density_control_factor(this->DensityControlFactor)
              .fairing_continuity(this->FairingContinuity)
              .geom_traits(K()));
          break;

        default:
          vtkErrorMacro("Invalid Filling Type selected for the algorithm");
          return 0;
      }
    }
  }

  // Convert Polyhedron to Output PolyData
  stkCGALUtilities::PolyhedronToPolyData(poly, output);

  return 1;
}
