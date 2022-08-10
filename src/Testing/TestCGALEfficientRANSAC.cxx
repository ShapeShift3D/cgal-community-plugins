#include "doctest.h"

#include <vtkCylinderSource.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

#include "stkCGALEfficientRANSAC.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALEfficientRANSAC
// CMake variable PARAVIEW_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALEfficientRANSAC(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALEfficientRANSAC*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALEfficientRANSACNS
{

TEST_SUITE("TestCGALEfficientRANSAC")
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

    auto alignNormals = vtkSmartPointer<vtkPolyDataNormals>::New();
    alignNormals->SetInputData(subdivideFilter->GetOutput());
    alignNormals->AutoOrientNormalsOn();
    alignNormals->SplittingOff();
    alignNormals->ConsistencyOn();
    alignNormals->NonManifoldTraversalOn();
    alignNormals->FlipNormalsOff();
    alignNormals->ComputeCellNormalsOff();
    alignNormals->ComputeCellNormalsOn();
    alignNormals->Update();

    auto testInput = vtkSmartPointer<vtkPolyData>::New();
    testInput->ShallowCopy(alignNormals->GetOutput());

    REQUIRE(testInput != nullptr);
    REQUIRE(testInput->GetNumberOfPoints() != 0);
    REQUIRE(testInput->GetPointData()->GetArray("Normals") != nullptr);

    auto planeDetection = vtkSmartPointer<stkCGALEfficientRANSAC>::New();
    planeDetection->SetInputData(testInput);
    planeDetection->SetNumberOfRuns(3);
    planeDetection->SetPointNormalsArrayName("Normals");
    planeDetection->SetMinPointsInputTypeToPercentage();
    planeDetection->SetMinPoints(2.5);
    planeDetection->SetProbability(0.05);
    planeDetection->SetEpsilonInputTypeToValue();
    planeDetection->SetEpsilon(1);
    planeDetection->SetClusterEpsilonInputTypeToValue();
    planeDetection->SetClusterEpsilon(3);
    planeDetection->SetMaxNormalThreshold(0.95);
    planeDetection->SetRegionsArrayName("DetectedCubePlanes");
    planeDetection->CalculateDistanceFromPlaneOff();
    planeDetection->SetPointSearchTolerance(0.001);
    planeDetection->SetKernelValueToEPIC();
    planeDetection->Update();

    // All 6 planes of the cubes should have been detected by efficient RANSAC
    // i.e max index of "DetectedCubePlanes" must be 5
    CHECK(
      planeDetection->GetOutput()->GetPointData()->GetArray("DetectedCubePlanes")->GetRange()[1] ==
      cylinder_resolution + 1.0);
  }
}
}