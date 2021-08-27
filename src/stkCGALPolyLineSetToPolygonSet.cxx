/**
 * \class stkCGALPolyLineSetToPolygonSet
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
#include "stkCGALPolyLineSetToPolygonSet.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkCellData.h>
#include <vtkIdTypeArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

#include <vtkCleanPolyData.h>
#include <vtkGeometryFilter.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkThreshold.h>

//---------Module--------------------------------------------------
#include <stkCGALPolygonUtilities.h>

//----------
// Declare the plugin
vtkStandardNewMacro(stkCGALPolyLineSetToPolygonSet);

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
stkCGALPolyLineSetToPolygonSet::stkCGALPolyLineSetToPolygonSet()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
  this->Plane = stkCGALPolyLineSetToPolygonSet::Planes::XY;
  this->PwhIdArrayName = "PolygonWithHolesId";
  this->DebugMode = false;
  this->PrintPoints = false;
}

//---------------------------------------------------
vtkPolyData* stkCGALPolyLineSetToPolygonSet::GetInputPolyLineSet()
{
  if (this->GetNumberOfInputConnections(0) < 1)
  {
    return nullptr;
  }

  return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

//---------------------------------------------------
Polygon_set_2* stkCGALPolyLineSetToPolygonSet::GetOutputPolygonSet()
{
  return &PolygonSet;
}

// ----------------------------------------------------------------------------
// Gets the input
// Creates CGAL::Surface_mesh from vtkPolydata
// Calls the CGAL::RunBooleanOperations
// Fills the output vtkUnstructuredGrid from the result.
int stkCGALPolyLineSetToPolygonSet::RequestData(
  vtkInformation*, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  //  Get the input and output data objects.
  //  Get the info objects
  vtkPolyData* inputPolyLineSet = this->GetInputPolyLineSet();

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

  // Verifying Polygon with holes ID array
  vtkIdTypeArray* pwhIdArray = nullptr;

  vtkNew<vtkPolyDataConnectivityFilter> connectivityFilter;
  connectivityFilter->SetInputData(inputPolyLineSet);
  connectivityFilter->SetExtractionModeToAllRegions();
  connectivityFilter->Update();

  int nbOfPolylines = connectivityFilter->GetNumberOfExtractedRegions();

  if (nbOfPolylines > 1) // If we have one curve, it is necessarily one polygon.
  {
    vtkAbstractArray* pwhIdAbstractArray =
      inputPolyLineSet->GetCellData()->GetAbstractArray(this->PwhIdArrayName.c_str());
    if (pwhIdAbstractArray == nullptr)
    {
      vtkWarningMacro("Polygon with holes ID array is missing. Assuming they are all from the same "
                      "Polygon With Hole.");
    }
    else
    {
      pwhIdArray = vtkArrayDownCast<vtkIdTypeArray>(pwhIdAbstractArray);
      if (pwhIdArray == nullptr)
      {
        vtkErrorMacro("Polygon with holes ID array does not contain numeric data.");
        return 0;
      }

      if (pwhIdArray->GetNumberOfComponents() != 1)
      {
        vtkErrorMacro("Polygon with holes ID array must only have 1 component.");
        return 0;
      }
    }
  }

  vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));

  this->PolygonSet.clear();

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

  if (pwhIdArray != nullptr)
  {
    double range[3] = { 0.0, 0.0 };
    pwhIdArray->GetRange(range);

    vtkNew<vtkThreshold> pwhExtractionFilter;
    pwhExtractionFilter->SetInputData(inputPolyLineSet);
    pwhExtractionFilter->SetInputArrayToProcess(
      0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS, this->PwhIdArrayName.c_str());
    pwhExtractionFilter->SetAllScalars(true);

    vtkNew<vtkGeometryFilter> UGToPoly;
    UGToPoly->SetInputConnection(pwhExtractionFilter->GetOutputPort());

    for (int i = range[0]; i < range[1] + 1; ++i)
    {
      pwhExtractionFilter->ThresholdBetween(i, i);
      UGToPoly->Update();

      this->ProcessPwh(UGToPoly->GetOutput(), firstCoordinate, secondCoordinate);
    }
  }
  else
  {
    this->ProcessPwh(inputPolyLineSet, firstCoordinate, secondCoordinate);
  }

  if (this->DebugMode)
  {
    stkCGALPolygonUtilities::PrintPolygonSet2Properties(PolygonSet, "Polygon Set", false);
  }

  return 1;
}

//---------------------------------------------------
bool stkCGALPolyLineSetToPolygonSet::ProcessPwh(
  vtkPolyData* pwhPoly, int firstCoord, int secondCoord)
{
  vtkNew<vtkPolyDataConnectivityFilter> connectivityFilter;
  connectivityFilter->SetInputData(pwhPoly);
  connectivityFilter->SetExtractionModeToAllRegions();
  connectivityFilter->Update();

  int nbOfPolylines = connectivityFilter->GetNumberOfExtractedRegions();
  connectivityFilter->SetExtractionModeToSpecifiedRegions();

  vtkNew<vtkCleanPolyData> cleanFilter;
  cleanFilter->SetInputConnection(connectivityFilter->GetOutputPort());

  Polygon_with_holes_2 polygonResult;
  bool boundaryPresent = false;

  for (vtkIdType i = 0; i < nbOfPolylines; ++i)
  {
    connectivityFilter->InitializeSpecifiedRegionList();
    connectivityFilter->AddSpecifiedRegion(i);
    cleanFilter->Update();

    Polygon_2 polygon;
    stkCGALPolygonUtilities::vtkPolyDataToPolygon2(
      cleanFilter->GetOutput(), polygon, firstCoord, secondCoord);

    CGAL::Orientation orient = polygon.orientation();
    if (this->DebugMode)
      stkCGALPolygonUtilities::PrintPolygonProperties(
        polygon, "Polyline " + std::to_string(i), false);

    if (polygon.is_clockwise_oriented())
    {

      polygonResult.add_hole(polygon);
    }
    else
    {
      if (boundaryPresent)
      {
        vtkErrorMacro("You cannot have multiple boundaries in a polygon with hole.");
        return 0;
      }

      polygonResult.outer_boundary() = polygon;
      boundaryPresent = true;
    }
  }

  PolygonSet.insert(polygonResult);
  return 1;
}