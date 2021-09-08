#include "stkCGALPolygonSetToPolyLineSet.h"

//---------VTK----------------------------------
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>

#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkPolyDataConnectivityFilter.h>

//---------Module--------------------------------------------------
#include <stkCGALPolygonUtilities.h>

vtkStandardNewMacro(stkCGALPolygonSetToPolyLineSet);

// -----------------------------------------------------------------------------
stkCGALPolygonSetToPolyLineSet::stkCGALPolygonSetToPolyLineSet()
  : Plane(Planes::XY)
  , PwhIdArrayName("PolygonWithHolesId")
  , OneCell(true)
  , DebugMode(false)
  , PrintPoints(false)
{
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//---------------------------------------------------
vtkPolyData* stkCGALPolygonSetToPolyLineSet::GetOutputPolyLineSet()
{
  return vtkPolyData::SafeDownCast(this->GetOutputDataObject(0));
}

//---------------------------------------------------
void stkCGALPolygonSetToPolyLineSet::SetInputPolygonSet(Polygon_set_2& polygonSet)
{
  this->PolygonSet = polygonSet;
}

// ----------------------------------------------------------------------------
int stkCGALPolygonSetToPolyLineSet::RequestData(
  vtkInformation*, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
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

  vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));

  Pwh_list_2 result;
  this->PolygonSet.polygons_with_holes(std::back_inserter(result));
  stkCGALPolygonUtilities::PwhList2ToPolyData(result, output0, this->PwhIdArrayName, this->OneCell);

  if (this->DebugMode)
  {
    stkCGALPolygonUtilities::PrintPolygonSet2Properties(
      this->PolygonSet, "Polygon Set", this->PrintPoints);
  }

  return 1;
}