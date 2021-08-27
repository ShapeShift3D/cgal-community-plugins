/**
 * \class stkCGALPolygonOrientOperator
 *
 * \brief
 *
 *		 Conditions for valid polygons:
 *
 *		 Closed Boundary - the polygon's outer boundary must be a connected sequence of
 *curves, that start and end at the same vertex. Simplicity - the polygon must be simple.
 *		 Orientation - the polygon's outer boundary must be counter-clockwise oriented.
 *
 * Inputs: inputMeshA (port == 0, vtkPolyData), inputMeshB (port == 1, vtkPolyData)
 * Output: output (port == 0, vtkUnstructuredGrid)
 *
 */

//---------VTK----------------------------------
#include "stkCGALPolygonOrientOperator.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>

//---------CGAL---------------------------------
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Surface_mesh.h>

//---------Module--------------------------------------------------
#include <stkCGALPolygonUtilities.h>

vtkStandardNewMacro(stkCGALPolygonOrientOperator);

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2> Pwh_list_2;
typedef CGAL::Polygon_set_2<K> Polygon_set_2;

// ----------------------------------------------------------------------------
int stkCGALPolygonOrientOperator::RequestData(
  vtkInformation*, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  //  Get the input and output data objects.
  //  Get the info objects
  vtkPolyData* inputPolyLine = this->GetInputPolyLine();

  if (inputPolyLine == nullptr)
  {
    vtkErrorMacro("Input PolyLine is empty.");
    return 0;
  }

  if (inputPolyLine->GetPoints() == nullptr)
  {
    vtkErrorMacro("Input PolyLine does not contain any point structure.");
    return 0;
  }

  if (inputPolyLine->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("Input PolyLine contains no points.");
    return 0;
  }

  vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));

  int firstCoordinate = -1;
  int secondCoordinate = -1;
  switch (this->Plane)
  {
    case Planes::XY:
    {
      firstCoordinate = 0;
      secondCoordinate = 1;
      break;
    }
    case Planes::YZ:
    {
      firstCoordinate = 1;
      secondCoordinate = 2;
      break;
    }
    case Planes::XZ:
    {
      firstCoordinate = 0;
      secondCoordinate = 2;
      break;
    }
    default:
    {
      vtkErrorMacro("Unknown Plane.");
      return 0;
    }
  }

  // A polygon is a closed chain of edges. Several algorithms are available for polygons.
  // For some of those algorithms, it is necessary that the polygon is simple.
  // A polygon is simple if edges don't intersect, except consecutive edges, which intersect in
  // their common vertex. Taken from https://doc.cgal.org/4.14.3/Polygon/index.html
  Polygon_2 polygon;

  stkCGALPolygonUtilities::vtkPolyDataToPolygon2(
    inputPolyLine, polygon, firstCoordinate, secondCoordinate);

  if (this->InvertPolyLineOrientation)
    polygon.reverse_orientation();

  if (this->ForcePolyLineOrientation)
  {
    CGAL::Orientation orient = polygon.orientation();

    switch (this->PolyLineOrientation)
    {
      case stkCGALPolygonOrientOperator::PolygonOrientations::CLOCKWISE:
      {
        if (orient == CGAL::COUNTERCLOCKWISE)
        {
          polygon.reverse_orientation();
        }
        break;
      }
      case stkCGALPolygonOrientOperator::PolygonOrientations::COUNTERCLOCKWISE:
      {
        if (orient == CGAL::CLOCKWISE)
        {
          polygon.reverse_orientation();
        }
        break;
      }
      default:
      {
        vtkErrorMacro("Unknown polygon orientation for PolyLine.");
        return 0;
      }
    }
  }

  output0->DeepCopy(inputPolyLine);
  stkCGALPolygonUtilities::Polygon2ToPolyLine(polygon, output0);
  return 1;
}
