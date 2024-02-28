#include "doctest.h"

#include <vtkCellData.h>
#include <vtkCleanPolyData.h>
#include <vtkCylinderSource.h>
#include <vtkFeatureEdges.h>
#include <vtkIntArray.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

#include "stkCGALRegionFairingOperator.h"

#include <string>

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALRegionFairingOperator
// CMake variable STK_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALRegionFairingOperator(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALRegionFairingOperator*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALRegionFairingOperatorNS
{

TEST_SUITE("TestCGALRegionFairingOperator")
{
  int resolution = 8;
  int subdivisionLevel = 2;
  int dummyCellValue = 5;
  std::string dummyCellArrayName = "DummyCellData";
  int dummyPointValue = 18;
  std::string dummyPointArrayName = "DummyPointData";
  std::string vertaxMaskArrayName = "RegionMask";
  std::string fairedCellArrayName = "FairedCells";
  std::string fairedPointArrayName = "FairedPoints";

  TEST_CASE("Feature Loops on Cylinder Test")
  {
    auto cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
    cylinderSource->SetRadius(5.0);
    cylinderSource->SetHeight(2.0);
    cylinderSource->CappingOn();
    cylinderSource->SetResolution(resolution);
    cylinderSource->SetCenter(0.0, 0.0, 0.0);
    cylinderSource->Update();

    auto cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
    cleanFilter->SetInputData(cylinderSource->GetOutput());
    cleanFilter->PointMergingOn();
    cleanFilter->ConvertPolysToLinesOff();
    cleanFilter->ConvertStripsToPolysOff();
    cleanFilter->ConvertLinesToPointsOff();
    cleanFilter->Update();

    auto triangualteFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangualteFilter->SetInputData(cleanFilter->GetOutput());
    triangualteFilter->Update();

    auto subdivideFilter = vtkSmartPointer<vtkLinearSubdivisionFilter>::New();
    subdivideFilter->SetInputData(triangualteFilter->GetOutput());
    subdivideFilter->SetNumberOfSubdivisions(subdivisionLevel);
    subdivideFilter->Update();

    auto testInput = vtkSmartPointer<vtkPolyData>::New();
    testInput->ShallowCopy(subdivideFilter->GetOutput());

    REQUIRE(testInput != nullptr);
    REQUIRE(testInput->GetNumberOfPoints() != 0);
    REQUIRE(testInput->GetNumberOfCells() != 0);

    auto featuresEdgesFilter = vtkSmartPointer<vtkFeatureEdges>::New();
    featuresEdgesFilter->BoundaryEdgesOff();
    featuresEdgesFilter->NonManifoldEdgesOff();
    featuresEdgesFilter->ManifoldEdgesOff();
    featuresEdgesFilter->ColoringOff();
    featuresEdgesFilter->FeatureEdgesOn();
    featuresEdgesFilter->SetFeatureAngle(80.0);

    featuresEdgesFilter->SetInputData(testInput);
    featuresEdgesFilter->Update();

    // Top and Both Loop near the caps should be present in Input
    CHECK_MESSAGE(featuresEdgesFilter->GetOutput()->GetNumberOfCells() ==
        resolution * std::pow(subdivisionLevel, 2) * 2,
      "Input Cylinder does not contain expected feature edges");

    // Adding some dummy data to test input
    auto dummyCellArray = vtkSmartPointer<vtkIntArray>::New();
    dummyCellArray->SetNumberOfComponents(1);
    dummyCellArray->SetNumberOfTuples(testInput->GetNumberOfPoints());
    dummyCellArray->SetName(dummyCellArrayName.c_str());
    dummyCellArray->Fill(dummyCellValue);

    testInput->GetCellData()->AddArray(dummyCellArray);

    auto dummyPointArray = vtkSmartPointer<vtkIntArray>::New();
    dummyPointArray->SetNumberOfComponents(1);
    dummyPointArray->SetNumberOfTuples(testInput->GetNumberOfPoints());
    dummyPointArray->SetName(dummyPointArrayName.c_str());
    dummyPointArray->Fill(dummyPointValue);

    testInput->GetPointData()->AddArray(dummyPointArray);

    // Mask Array
    auto vertexMaskArray = vtkSmartPointer<vtkIntArray>::New();
    vertexMaskArray->SetNumberOfComponents(1);
    vertexMaskArray->SetNumberOfTuples(testInput->GetNumberOfPoints());
    vertexMaskArray->SetName(vertaxMaskArrayName.c_str());
    vertexMaskArray->Fill(0);

    for (int id = 0; id < testInput->GetNumberOfPoints(); id++)
    {
      if (testInput->GetPoints()->GetPoint(id)[1] > 0.0)
      {
        vertexMaskArray->SetTuple1(id, 1.0);
      }
    }

    testInput->GetPointData()->AddArray(vertexMaskArray);

    auto regionFairingOperatorFilter = vtkSmartPointer<stkCGALRegionFairingOperator>::New();
    regionFairingOperatorFilter->SetInputData(testInput);
    regionFairingOperatorFilter->SetRegionArrayName(vertaxMaskArrayName.c_str());
    regionFairingOperatorFilter->SetFairingContinuity(stkCGALRegionFairingOperator::C1);
    regionFairingOperatorFilter->GenerateFairedMaskArraysOn();
    regionFairingOperatorFilter->SetFairedPointMaskArrayName(fairedPointArrayName.c_str());
    regionFairingOperatorFilter->SetFairedCellMaskArrayName(fairedCellArrayName.c_str());
    regionFairingOperatorFilter->PassAllArraysOn();
    regionFairingOperatorFilter->ConsumeInputMaskArrayOn();
    regionFairingOperatorFilter->Update();

    auto fairedOutput = regionFairingOperatorFilter->GetOutput();

    featuresEdgesFilter->SetInputData(fairedOutput);
    featuresEdgesFilter->Update();

    // Only Loop near bottom cap should be present in Faired Output
    CHECK_MESSAGE(featuresEdgesFilter->GetOutput()->GetNumberOfCells() ==
        resolution * std::pow(subdivisionLevel, 2),
      "Faired Cylinder does not contain expected feature edges");

    // Checks for existance of array in the output. Data value are not being checked in this unit
    // test.
    CHECK_MESSAGE(fairedOutput->GetPointData()->HasArray(fairedPointArrayName.c_str()) == 1,
      "Output does not contain Faired Point Mask");

    CHECK_MESSAGE(fairedOutput->GetCellData()->HasArray(fairedCellArrayName.c_str()) == 1,
      "Output does not contain Faired Cell Mask");

    CHECK_MESSAGE(fairedOutput->GetPointData()->HasArray(vertaxMaskArrayName.c_str()) == 0,
      "Output does not consume Input Point Mask");

    CHECK_MESSAGE(fairedOutput->GetPointData()->HasArray(dummyPointArrayName.c_str()) == 1,
      "Output does not contain Dummy Point Data");

    CHECK_MESSAGE(fairedOutput->GetCellData()->HasArray(dummyCellArrayName.c_str()) == 1,
      "Output does not contained Dummy Cell Data");
  }
}
}