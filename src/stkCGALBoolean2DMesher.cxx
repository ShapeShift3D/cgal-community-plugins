#include "stkCGALBoolean2DMesher.h"

//---------VTK----------------------------------
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>

//---------CGAL---------------------------------
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Surface_mesh.h>

//---------Module--------------------------------------------------
#include <stkCGALPolyLineSetToPolygonSet.h>
#include <stkCGALPolygonSetToPolyLineSet.h>
#include <stkCGALPolygonUtilities.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2> Pwh_list_2;
typedef CGAL::Polygon_set_2<K> Polygon_set_2;

vtkStandardNewMacro(stkCGALBoolean2DMesher);

// ----------------------------------------------------------------------------
int stkCGALBoolean2DMesher::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** vtkNotUsed(inputVector), vtkInformationVector* outputVector)
{
  //  Get the input and output data objects.
  //  Get the info objects
  vtkPolyData* inputPolyLineSetA = this->GetInputPolyLineSetA();
  vtkPolyData* inputPolyLineSetB = this->GetInputPolyLineSetB();

  if (inputPolyLineSetA == nullptr)
  {
    vtkErrorMacro("Input PolyLine Set A is empty.");
    return 0;
  }

  if (inputPolyLineSetA->GetPoints() == nullptr)
  {
    vtkErrorMacro("Input PolyLine Set A does not contain any point structure.");
    return 0;
  }

  if (inputPolyLineSetB == nullptr)
  {
    vtkErrorMacro("Input PolyLine Set B is empty.");
    return 0;
  }

  if (inputPolyLineSetB->GetPoints() == nullptr)
  {
    vtkErrorMacro("Input PolyLine Set B does not contain any point structure.");
    return 0;
  }

  vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));

  vtkNew<stkCGALPolyLineSetToPolygonSet> polylineSetToPolygonSetFilter;
  polylineSetToPolygonSetFilter->SetPlane(this->Plane);
  polylineSetToPolygonSetFilter->SetPwhIdArrayName(this->PwhIdArrayName.c_str());
  polylineSetToPolygonSetFilter->SetDebugMode(false);
  polylineSetToPolygonSetFilter->SetPrintPoints(false);

  polylineSetToPolygonSetFilter->SetInputData(0, inputPolyLineSetA);
  polylineSetToPolygonSetFilter->Update();
  Polygon_set_2 polygonSetA = *polylineSetToPolygonSetFilter->GetOutputPolygonSet();

  polylineSetToPolygonSetFilter->SetInputData(0, inputPolyLineSetB);
  polylineSetToPolygonSetFilter->Update();
  Polygon_set_2 polygonSetB = *polylineSetToPolygonSetFilter->GetOutputPolygonSet();

  if (this->DebugMode)
  {
    stkCGALPolygonUtilities::PrintPolygonSet2Properties(polygonSetA, "Polygon Set A", false);
    stkCGALPolygonUtilities::PrintPolygonSet2Properties(polygonSetB, "Polygon Set B", false);
  }

  vtkNew<stkCGALPolygonSetToPolyLineSet> polygonSetToPolylineSetFilter;
  polygonSetToPolylineSetFilter->SetPlane(this->Plane);
  polygonSetToPolylineSetFilter->SetPwhIdArrayName(this->PwhIdArrayName.c_str());
  polygonSetToPolylineSetFilter->SetOneCell(this->OneCell);
  polygonSetToPolylineSetFilter->SetDebugMode(false);
  polygonSetToPolylineSetFilter->SetPrintPoints(false);

  Polygon_set_2 result;
  std::string debugResultMessage;

  try
  {
    if (this->OperationMode == stkCGALBoolean2DMesher::OperationModes::ADD)
    {
      polygonSetA.join(polygonSetB);

      result = polygonSetA;
      debugResultMessage = "ADD";
    }
    else if (this->OperationMode == stkCGALBoolean2DMesher::OperationModes::INTERSECT)
    {
      polygonSetA.intersection(polygonSetB);

      result = polygonSetA;
      debugResultMessage = "INTERSECT";
    }
    else if (this->OperationMode == stkCGALBoolean2DMesher::OperationModes::A_MINUS_B)
    {
      polygonSetA.difference(polygonSetB);

      result = polygonSetA;
      debugResultMessage = "A_MINUS_B";
    }
    else if (this->OperationMode == stkCGALBoolean2DMesher::OperationModes::B_MINUS_A)
    {
      polygonSetB.difference(polygonSetA);

      result = polygonSetB;
      debugResultMessage = "B_MINUS_A";
    }
    else if (this->OperationMode == stkCGALBoolean2DMesher::OperationModes::EXCLUDE_OVERLAP)
    {
      polygonSetA.symmetric_difference(polygonSetB);

      result = polygonSetA;
      debugResultMessage = "EXCLUDE_OVERLAP";
    }
    else if (this->OperationMode == stkCGALBoolean2DMesher::OperationModes::COMPLEMENT)
    {
      if (this->ComplementOf == stkCGALBoolean2DMesher::Inputs::A)
      {
        polygonSetA.complement();

        result = polygonSetA;
        debugResultMessage = "COMPLEMENT:A";
      }
      else if (this->ComplementOf == stkCGALBoolean2DMesher::Inputs::B)
      {
        polygonSetB.complement();

        result = polygonSetB;
        debugResultMessage = "COMPLEMENT:B";
      }
    }
    else if (this->OperationMode == stkCGALBoolean2DMesher::OperationModes::INTERSECT_COMPLEMENT)
    {
      // NAND
      polygonSetA.intersection(polygonSetB);
      polygonSetA.complement();

      result = polygonSetA;
      debugResultMessage = "INTERSECT_COMPLEMENT";
    }
    else if (this->OperationMode == stkCGALBoolean2DMesher::OperationModes::EXCLUSIVE_ADD)
    {
      // Intersect Complement (NAND)
      Polygon_set_2 S = polygonSetA;
      Polygon_set_2 Q = polygonSetB;
      S.intersection(Q);
      S.complement();

      // OR
      polygonSetA.join(polygonSetB);

      // AND
      polygonSetA.intersection(S);

      result = polygonSetA;
      debugResultMessage = "EXCLUSIVE_ADD";
    }
  }
  catch (const std::exception& e)
  {
    vtkErrorMacro(<< "Error caught : " << e.what());
    return 0;
  }

  if (this->DebugMode)
  {
    stkCGALPolygonUtilities::PrintPolygonSet2Properties(polygonSetA, debugResultMessage, false);
  }

  polygonSetToPolylineSetFilter->SetInputPolygonSet(result);
  polygonSetToPolylineSetFilter->Update();

  output0->ShallowCopy(polygonSetToPolylineSetFilter->GetOutput());

  return 1;
}
