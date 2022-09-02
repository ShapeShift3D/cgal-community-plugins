#include "doctest.h"

#include <vtkCellData.h>
#include <vtkCleanPolyData.h>
#include <vtkConnectivityFilter.h>
#include <vtkCylinderSource.h>
#include <vtkDataArray.h>
#include <vtkFeatureEdges.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>
#include <vtkPointSet.h>
#include <vtkArrayCalculator.h>

#include "stkCGALFillHoles.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALFillHoles
// CMake variable PARAVIEW_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALFillHoles(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALFillHoles*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALFillHolesNS
{

TEST_SUITE("TestCGALFillHoles")
{
  int cylinder_resolution = 8;
  auto cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();

  std::string dummyPointArrayName = "DummyPointArray";
  std::string dummyCellArrayName = "CellPointArray";
  double dummyFillValue = 5.0;

  auto featureEdges = vtkSmartPointer<vtkFeatureEdges>::New();

  auto inputSurfaceMesh = vtkSmartPointer<vtkPolyData>::New();

  auto outputSurfaceMesh = vtkSmartPointer<vtkPolyData>::New();

  auto cgalFillHoles = vtkSmartPointer<stkCGALFillHoles>::New();

  TEST_CASE("Generate Input")
  {
    cylinderSource->SetRadius(5.0);
    cylinderSource->SetHeight(5.0);
    cylinderSource->CappingOff();
    cylinderSource->SetResolution(cylinder_resolution);
    cylinderSource->SetCenter(0.0, 0.0, 0.0);
    cylinderSource->Update();

    auto triangualteFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangualteFilter->SetInputData(cylinderSource->GetOutput());
    triangualteFilter->Update();

    auto subdivideFilter = vtkSmartPointer<vtkLinearSubdivisionFilter>::New();
    subdivideFilter->SetInputData(triangualteFilter->GetOutput());
    subdivideFilter->SetNumberOfSubdivisions(4);
    subdivideFilter->Update();

    auto pointArrayCalcultor = vtkSmartPointer<vtkArrayCalculator>::New();
    pointArrayCalcultor->SetInputData(subdivideFilter->GetOutput());
    pointArrayCalcultor->SetResultArrayName(dummyPointArrayName.c_str());
    pointArrayCalcultor->SetFunction(std::to_string(dummyFillValue).c_str());
    pointArrayCalcultor->SetAttributeTypeToPointData();
    pointArrayCalcultor->Update();

    auto cellArrayCalcultor = vtkSmartPointer<vtkArrayCalculator>::New();
    cellArrayCalcultor->SetInputData(pointArrayCalcultor->GetOutput());
    cellArrayCalcultor->SetResultArrayName(dummyCellArrayName.c_str());
    cellArrayCalcultor->SetFunction(std::to_string(dummyFillValue).c_str());
    cellArrayCalcultor->SetAttributeTypeToCellData();
    cellArrayCalcultor->Update();

    inputSurfaceMesh->ShallowCopy(cellArrayCalcultor->GetOutput());

    REQUIRE_MESSAGE(
      inputSurfaceMesh->GetPointData()->GetArray(dummyPointArrayName.c_str()) != nullptr,
      "Generated Tube Input contains point Data Array");

    REQUIRE_MESSAGE(
      inputSurfaceMesh->GetCellData()->GetArray(dummyCellArrayName.c_str()) != nullptr,
      "Generated Tube Input contains cell Data Array");

    featureEdges->BoundaryEdgesOn();
    featureEdges->FeatureEdgesOff();
    featureEdges->NonManifoldEdgesOff();
    featureEdges->ManifoldEdgesOff();

    featureEdges->SetInputData(inputSurfaceMesh);
    featureEdges->Update();

    REQUIRE_MESSAGE(featureEdges->GetOutput()->GetNumberOfCells() != 0,
      "Generated Tube Input contains boundary edges");
  }

  TEST_CASE("Close Tube")
  {
    cgalFillHoles->SetInputData(0, inputSurfaceMesh);
    cgalFillHoles->SetBoundaryCycleSelectionMethod(
      stkCGALFillHoles::BoundaryCycleSelectionMethods::ALL_CYCLES);
    cgalFillHoles->UseDelaunayTriangulationOn();
    cgalFillHoles->SetFillingType(stkCGALFillHoles::FillingTypes::FILLHOLES_REFINE_FAIR);
    cgalFillHoles->SetFairingContinuity(stkCGALFillHoles::FairingContinuityTypes::C1);
    cgalFillHoles->EnableMaxHoleBBDiagonalOff();
    cgalFillHoles->SkipSelfIntersectingPatchesOff();
    cgalFillHoles->SetPointArrayFillValueType(stkCGALFillHoles::ArrayFillValueTypes::NEG_INF);
    cgalFillHoles->SetCellArrayFillValueType(stkCGALFillHoles::ArrayFillValueTypes::NEG_INF);
    cgalFillHoles->Update();

    outputSurfaceMesh->ShallowCopy(cgalFillHoles->GetOutput());

    auto outputDummyPointArray =
      outputSurfaceMesh->GetPointData()->GetArray(dummyPointArrayName.c_str());

    CHECK_MESSAGE(outputDummyPointArray != nullptr, "Output must pass point Array");

    CHECK_MESSAGE(
      outputDummyPointArray->GetRange()[1] == dummyFillValue, "Output must pass point Array");

    auto outputDummyCellArray =
      outputSurfaceMesh->GetCellData()->GetArray(dummyCellArrayName.c_str());

    CHECK_MESSAGE(outputDummyCellArray != nullptr, "Output must pass cell Array");

    CHECK_MESSAGE(
      outputDummyCellArray->GetRange()[1] == dummyFillValue, "Output must pass cell Array");

    featureEdges->SetInputData(cgalFillHoles->GetOutput());
    featureEdges->Update();

    CHECK_MESSAGE(featureEdges->GetOutput()->GetNumberOfCells() == 0,
      "Output must not contain any boundary edges");
  }

  TEST_CASE("Close Tube at the Specified Boundary")
  {
    featureEdges->SetInputData(inputSurfaceMesh);
    featureEdges->Update();

    auto connectivity = vtkSmartPointer<vtkConnectivityFilter>::New();
    connectivity->SetInputData(featureEdges->GetOutput());
    connectivity->SetExtractionModeToSpecifiedRegions();
    connectivity->InitializeSpecifiedRegionList();
    connectivity->AddSpecifiedRegion(0);
    connectivity->Update();

    auto cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
    cleanFilter->SetInputData(connectivity->GetOutput());
    cleanFilter->Update();

    auto specifiedBoundriesPointSet = vtkSmartPointer<vtkPointSet>::New();
    specifiedBoundriesPointSet->ShallowCopy(cleanFilter->GetOutput());

    cgalFillHoles->SetInputData(0, inputSurfaceMesh);
    cgalFillHoles->SetInputData(1, specifiedBoundriesPointSet);
    cgalFillHoles->SetBoundaryCycleSelectionMethod(
      stkCGALFillHoles::BoundaryCycleSelectionMethods::FILL_SPECIFIED_CYCLES);
    cgalFillHoles->Update();

    outputSurfaceMesh->ShallowCopy(cgalFillHoles->GetOutput());

    featureEdges->SetInputData(outputSurfaceMesh);
    featureEdges->Update();

    connectivity->SetInputData(featureEdges->GetOutput());
    connectivity->SetExtractionModeToAllRegions();
    connectivity->Update();

    CHECK_MESSAGE(connectivity->GetNumberOfExtractedRegions() == 1,
      "Output must not contain 1 boundary loop");
  }
}
}