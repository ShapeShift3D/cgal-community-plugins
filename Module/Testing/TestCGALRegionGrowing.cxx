#include "doctest.h"

#include <vtkCylinderSource.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkMath.h>
#include <vtkCellData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

#include "stkCGALRegionGrowing.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALRegionGrowing
// CMake variable PARAVIEW_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALRegionGrowing(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALRegionGrowing*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALRegionGrowingNS
{

TEST_SUITE("TestCGALRegionGrowing")
{
  TEST_CASE("Identify Planes on a Cube")
  {
    int cylinder_resolution = 4;

    auto cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
    cylinderSource->SetRadius(5.0);
    cylinderSource->SetHeight(5.0);
    cylinderSource->CappingOn();
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

    auto testInput = vtkSmartPointer<vtkPolyData>::New();
    testInput->ShallowCopy(subdivideFilter->GetOutput());

    REQUIRE(testInput != nullptr);
    REQUIRE(testInput->GetNumberOfPoints() != 0);

    auto planeDetection = vtkSmartPointer<stkCGALRegionGrowing>::New();
    planeDetection->SetInputData(testInput);
    planeDetection->SetMinRegionSize(100);
    planeDetection->SetMaxDistanceToPlane(1);
    planeDetection->SetKernelValueToEPEC();
    planeDetection->SetMaxAcceptedAngle(5);
    planeDetection->Update();

    // All 6 planes of the cubes should have been detected by efficient RANSAC
    // i.e max index of "DetectedCubePlanes" must be 5
    CHECK(planeDetection->GetOutput()->GetCellData()->GetArray("Regions")->GetRange()[1] ==
      cylinder_resolution + 1.0);
  }
}
}