#include "stkCGALPolygonOffset.h"

//---------VTK----------------------------------
#include <vtkInformation.h>
#include <vtkInformationVector.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_offset_polygons_2.h>

//--------Module-----------------------------
#include "stkCGALPolygonUtilities.h"

#include <boost/shared_ptr.hpp>

typedef CGAL::Exact_predicates_inexact_constructions_kernel KInexact;
typedef KInexact::Point_2 Point_2Inexact;
typedef CGAL::Polygon_2<KInexact> Polygon_2Inexact;

typedef boost::shared_ptr<Polygon_2Inexact> PolygonPtr;
typedef std::vector<PolygonPtr> PolygonPtrVector;

vtkStandardNewMacro(stkCGALPolygonOffset);

// ----------------------------------------------------------------------------
int stkCGALPolygonOffset::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkPolyData* input = this->GetInputPolyLine();

  vtkPolyData* output = vtkPolyData::GetData(outputVector, 0);

  if (input == nullptr)
  {
    vtkErrorMacro("Input is empty");
    return 0;
  }

  double bounds[6];
  input->GetBounds(bounds);

  if (bounds[5] - bounds[4] != 0)
  {
    vtkErrorMacro("Input must belong to the XY plane");
    return 0;
  }

  Polygon_2Inexact outputPolygon;
  Polygon_2Inexact polygon;

  if (!stkCGALPolygonUtilities::vtkPolyDataToPolygon2<KInexact>(input, polygon, 0, 1))
  {
    vtkErrorMacro("Failed to convert input into a polygon. Expected input to be a single Polygon");
    return 0;
  };

  if (polygon.orientation() == CGAL::CLOCKWISE)
  {
    polygon.reverse_orientation();
  }

  if (!polygon.is_simple())
  {
    vtkErrorMacro(
      << "Input must be a simple polygon i.e. the edges intersect only at the vertices and at most "
         "are coincident along a line but do not cross one another");
    return 0;
  }

  if (this->OffsetType == OffsetTypes::INTERIOR)
  {
    // creates interior sketon for the
    PolygonPtrVector inner_offset_polygon =
      CGAL::create_interior_skeleton_and_offset_polygons_2(this->Offset, polygon);

    if (inner_offset_polygon.size() == 0)
    {
      vtkErrorMacro("Unable to generate straight skeleton and interior offset polygon. Recheck "
                    "offset value, offset value is probably too large.");
      return 0;
    }

    auto offsetPolyPtr = *inner_offset_polygon.begin();
    outputPolygon = *offsetPolyPtr;
  }
  else if (this->OffsetType == OffsetTypes::EXTERIOR)
  {
    // creates a exterior skeleton between the polygon and sufficiently large bounding box of the
    // input polygon and offset the polygon with hole.Exterior offset is same as interior offset
    // with the difference that input polygon becomes a hole for bounding box of the polygon
    PolygonPtrVector outer_offset_polygon =
      CGAL::create_exterior_skeleton_and_offset_polygons_2(this->Offset, polygon);

    if (outer_offset_polygon.size() == 0)
    {
      vtkErrorMacro("Unable to generate straight skeleton and exterior offset polygon");
      return 0;
    }

    PolygonPtrVector::iterator largestAreaPos = outer_offset_polygon.end();
    double largestArea = 0.0;
    for (PolygonPtrVector::iterator i = outer_offset_polygon.begin();
         i != outer_offset_polygon.end(); i++)
    {
      // Take abs() as  Polygon_2::area() is signed.
      double area = std::abs((*i)->area());
      if (area > largestArea)
      {
        largestAreaPos = i;
        largestArea = area;
      }
    }

    // Remove the offset contour that corresponds to the frame.
    outer_offset_polygon.erase(largestAreaPos);

    auto offsetPolyPtr = *outer_offset_polygon.begin();
    outputPolygon = *offsetPolyPtr;
  }

  stkCGALPolygonUtilities::Polygon2ToVtkPolyLine<KInexact>(outputPolygon, output);

  return 1;
}
