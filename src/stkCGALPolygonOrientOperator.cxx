#include "stkCGALPolygonOrientOperator.h"

//---------VTK----------------------------------
#include <vtkInformation.h>
#include <vtkInformationVector.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

//---------Module--------------------------------------------------
#include <stkCGALPolygonUtilities.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;

vtkStandardNewMacro(stkCGALPolygonOrientOperator);

// ----------------------------------------------------------------------------
int stkCGALPolygonOrientOperator::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** vtkNotUsed(inputVector), vtkInformationVector* outputVector)
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

  if (!stkCGALPolygonUtilities::vtkPolyDataToPolygon2<K,Point_2>(
        inputPolyLine, polygon, firstCoordinate, secondCoordinate))
  {
    vtkErrorMacro("Failed to convert input into a polygon. Expected input to be a single Polygon");
    return 0;
  };

  if (this->InvertPolyLineOrientation)
  {
    polygon.reverse_orientation();
  }

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

  stkCGALPolygonUtilities::Polygon2ToVtkPolyLine<K>(polygon, output0);
  return 1;
}
