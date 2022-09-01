#include "stkCGALFillHoles.h"

//---------VTK----------------------------------
#include <vtkCellData.h>
#include <vtkCellTypes.h>
#include <vtkDataArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMath.h>
#include <vtkPointData.h>
#include <vtkPointSet.h>
#include <vtkSmartPointer.h>
#include <vtkStaticPointLocator.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>

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
bool IsSmallHole(halfedge_descriptor h, Mesh& mesh, double max_hole_diam)
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
void ExtractSpecifiedBoundaryCycles(Mesh& mesh, int boundaryCycleSelectionMethod,
  vtkPointSet* specifiedPoints, const double& tol, std::vector<halfedge_descriptor>& border_cycles)
{
  auto pointLocator = vtkSmartPointer<vtkStaticPointLocator>::New();
  pointLocator->SetDataSet(specifiedPoints);
  pointLocator->BuildLocator();

  auto iter = std::back_inserter(border_cycles);
  boost::unordered_set<halfedge_descriptor> hedge_handled;

  for (halfedge_descriptor h : CGAL::halfedges(mesh))
  {
    if (CGAL::is_border(h, mesh) && hedge_handled.insert(h).second)
    {
      bool containsSpecifiedPoints = true;
      for (halfedge_descriptor h2 : CGAL::halfedges_around_face(h, mesh))
      {
        const Point& p = mesh.point(CGAL::target(h2, mesh));
        double point_coords[3];
        point_coords[0] = p.x();
        point_coords[1] = p.y();
        point_coords[2] = p.z();

        double dist2 = 0.0;
        if (pointLocator->FindClosestPointWithinRadius(tol, point_coords, dist2) == -1)
        {
          containsSpecifiedPoints = false;
          break;
        }

        hedge_handled.insert(h2);
      }

      switch (boundaryCycleSelectionMethod)
      {
          // FILL_SPECIFIED_CYCLES
        case 2:
          if (containsSpecifiedPoints)
          {
            *iter++ = h;
          }
          break;
          // NOT_FILL_SPECIFIED_CYCLES
        case 3:
          if (!containsSpecifiedPoints)
          {
            *iter++ = h;
          }
          break;

        default:
          *iter++ = h;
          break;
      }
    }
  }
}

// ----------------------------------------------------------------------------
int stkCGALFillHoles::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** vtkNotUsed(inputVector), vtkInformationVector* outputVector)
{
  vtkPolyData* inputSurface = this->GetInputSurfaceMesh();

  vtkPolyData* outputSurface = vtkPolyData::GetData(outputVector, 0);

  if (inputSurface == nullptr)
  {
    vtkErrorMacro("Input Surface Mesh is NULL");
    return 0;
  }

  if (inputSurface->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("Input Surface Mesh contains 0 Points");
    return 0;
  }

  if (inputSurface->GetNumberOfCells() == 0)
  {
    vtkErrorMacro("Input Surface Mesh contains 0 Cells");
    return 0;
  }

  auto cellsType = vtkSmartPointer<vtkCellTypes>::New();
  inputSurface->GetCellTypes(cellsType);
  int numCellTypes = cellsType->GetNumberOfTypes();

  if (numCellTypes > 1 || numCellTypes == 0)
  {
    vtkErrorMacro("Invalid Input Surface. It should consist of only Triangle Cells");
    return 0;
  }
  else if (cellsType->GetCellType(0) != VTK_TRIANGLE)
  {
    vtkErrorMacro(
      "Input Surface must be a Triangular Surface Mesh (PolyData with only Triangle Cells)");
    return 0;
  }

  // Convert Input PolyData to Surface Mesh
  Mesh mesh;
  stkCGALUtilities::vtkPolyDataToPolygonMesh(inputSurface, mesh);

  std::vector<halfedge_descriptor> border_cycles;
  // collect one halfedge per boundary cycle

  if (this->BoundaryCycleSelectionMethod == BoundaryCycleSelectionMethods::ALL_CYCLES)
  {
    CGAL::Polygon_mesh_processing::extract_boundary_cycles(mesh, std::back_inserter(border_cycles));
  }
  else if (this->BoundaryCycleSelectionMethod ==
      BoundaryCycleSelectionMethods::FILL_SPECIFIED_CYCLES ||
    this->BoundaryCycleSelectionMethod == BoundaryCycleSelectionMethods::NOT_FILL_SPECIFIED_CYCLES)
  {
    vtkPointSet* specifiedBoundariesPoints = this->GetSpecifiedBoundariesPointSet();

    if (specifiedBoundariesPoints == nullptr)
    {
      vtkErrorMacro("Specified Boundaired Point Set is NULL");
      return 0;
    }

    if (specifiedBoundariesPoints->GetNumberOfPoints() == 0)
    {
      vtkErrorMacro("Specified Boundaired Point Set contains 0 Points");
      return 0;
    }

    ExtractSpecifiedBoundaryCycles(mesh, this->BoundaryCycleSelectionMethod,
      specifiedBoundariesPoints, this->SearchTolerance, border_cycles);
  }
  else
  {
    vtkErrorMacro("Select Boundary Cycle Selection Method");
    return 0;
  }

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
  stkCGALUtilities::SurfaceMeshToPolyData(mesh, outputSurface);

  // Transfer Point Ids
  for (int pointArrayID = 0; pointArrayID < inputSurface->GetPointData()->GetNumberOfArrays();
       pointArrayID++)
  {
    auto sourcePointArray = inputSurface->GetPointData()->GetArray(pointArrayID);

    vtkSmartPointer<vtkDataArray> targetPointArray;
    targetPointArray.TakeReference(sourcePointArray->NewInstance());
    targetPointArray->SetName(sourcePointArray->GetName());
    targetPointArray->SetNumberOfComponents(sourcePointArray->GetNumberOfComponents());
    targetPointArray->SetNumberOfTuples(outputSurface->GetNumberOfPoints());

    switch (this->PointArrayFillValueType)
    {
      case ArrayFillValueTypes::NOT_A_NUMBER:
        targetPointArray->Fill(vtkMath::Nan());
        break;
      case ArrayFillValueTypes::INF:
        targetPointArray->Fill(vtkMath::Inf());
        break;
      case ArrayFillValueTypes::NEG_INF:
        targetPointArray->Fill(vtkMath::NegInf());
        break;
      case ArrayFillValueTypes::CUSTOM:
        targetPointArray->Fill(this->PointArrayFillValue);
        break;

      default:
        targetPointArray->Fill(vtkMath::Nan());
        break;
    }

    for (int pointID = 0; pointID < inputSurface->GetNumberOfPoints(); pointID++)
    {
      targetPointArray->SetTuple(pointID, sourcePointArray->GetTuple(pointID));
    }

    outputSurface->GetPointData()->AddArray(targetPointArray);
  }

  //  Transfer Cells Ids
  for (int cellArrayID = 0; cellArrayID < inputSurface->GetCellData()->GetNumberOfArrays();
       cellArrayID++)
  {
    auto sourceCellArray = inputSurface->GetCellData()->GetArray(cellArrayID);

    vtkSmartPointer<vtkDataArray> targetCellArray;
    targetCellArray.TakeReference(sourceCellArray->NewInstance());
    targetCellArray->SetName(sourceCellArray->GetName());
    targetCellArray->SetNumberOfComponents(sourceCellArray->GetNumberOfComponents());
    targetCellArray->SetNumberOfTuples(outputSurface->GetNumberOfCells());
    switch (this->CellArrayFillValueType)
    {
      case ArrayFillValueTypes::NOT_A_NUMBER:
        targetCellArray->Fill(vtkMath::Nan());
        break;
      case ArrayFillValueTypes::INF:
        targetCellArray->Fill(vtkMath::Inf());
        break;
      case ArrayFillValueTypes::NEG_INF:
        targetCellArray->Fill(vtkMath::NegInf());
        break;
      case ArrayFillValueTypes::CUSTOM:
        targetCellArray->Fill(this->CellArrayFillValue);
        break;

      default:
        targetCellArray->Fill(vtkMath::Nan());
        break;
    }

    for (int cellID = 0; cellID < inputSurface->GetNumberOfCells(); cellID++)
    {
      targetCellArray->SetTuple(cellID, sourceCellArray->GetTuple(cellID));
    }

    outputSurface->GetCellData()->AddArray(targetCellArray);
  }

  // Transfer Field Data
  outputSurface->GetFieldData()->PassData(inputSurface->GetFieldData());

  return 1;
}