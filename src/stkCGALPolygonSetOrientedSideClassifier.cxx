/**
 * \class stkCGALPolygonSetOrientedSideClassifier
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
#include "stkCGALPolygonSetOrientedSideClassifier.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkCharArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

//---------CGAL---------------------------------
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Surface_mesh.h>

//---------Module--------------------------------------------------
#include <stkCGALPolyLineSetToPolygonSet.h>
#include <stkCGALPolygonUtilities.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2> Pwh_list_2;
typedef CGAL::Polygon_set_2<K> Polygon_set_2;

vtkStandardNewMacro(stkCGALPolygonSetOrientedSideClassifier);

// ----------------------------------------------------------------------------
int stkCGALPolygonSetOrientedSideClassifier::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** vtkNotUsed(inputVector), vtkInformationVector* outputVector)
{
  //  Get the input and output data objects.
  //  Get the info objects
  vtkPolyData* inputPointSet = this->GetInputPointSet();
  vtkPolyData* inputPolyLineSet = this->GetInputPolyLineSet();

  if (inputPointSet == nullptr)
  {
    vtkErrorMacro("Input Point Set is empty.");
    return 0;
  }

  if (inputPointSet->GetPoints() == nullptr)
  {
    vtkErrorMacro("Input Point Set does not contain any point structure.");
    return 0;
  }

  if (inputPointSet->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("Input Point Set contains no points.");
    return 0;
  }

  if (inputPolyLineSet == nullptr)
  {
    vtkErrorMacro("Input PolyLine Set is empty.");
    return 0;
  }

  if (inputPolyLineSet->GetPoints() == nullptr)
  {
    vtkErrorMacro("Input PolyLine Set does not contain any point structure.");
    return 0;
  }

  if (inputPolyLineSet->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("Input PolyLine Set contains no points.");
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

  vtkNew<stkCGALPolyLineSetToPolygonSet> polylineSetToPolygonSetFilter;
  polylineSetToPolygonSetFilter->SetPlane(this->Plane);
  polylineSetToPolygonSetFilter->SetPwhIdArrayName(this->PwhIdArrayName.c_str());
  polylineSetToPolygonSetFilter->SetInputData(0, inputPolyLineSet);
  polylineSetToPolygonSetFilter->Update();

  Polygon_set_2 polygonSet = *polylineSetToPolygonSetFilter->GetOutputPolygonSet();

  if (this->DebugMode)
  {
    stkCGALPolygonUtilities::PrintPolygonSet2Properties(polygonSet, "Polygon Set", false);
  }

  vtkNew<vtkCharArray> orientedSideArray;
  orientedSideArray->SetName(this->OrientedSideArrayName.c_str());
  orientedSideArray->SetNumberOfComponents(1);
  orientedSideArray->SetNumberOfTuples(inputPointSet->GetNumberOfPoints());

  double pt[3] = { 0.0, 0.0, 0.0 };
  for (vtkIdType i = 0; i < inputPointSet->GetNumberOfPoints(); ++i)
  {
    inputPointSet->GetPoint(i, pt);

    CGAL::Oriented_side side =
      polygonSet.oriented_side(Point_2(pt[firstCoordinate], pt[secondCoordinate]));
    orientedSideArray->SetTuple1(i, side);
  }

  output0->DeepCopy(inputPointSet);
  output0->GetPointData()->AddArray(orientedSideArray);

  return 1;
}
