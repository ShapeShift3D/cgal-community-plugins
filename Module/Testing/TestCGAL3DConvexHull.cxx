#include "doctest.h"

#include <vtkCylinderSource.h>
#include <vtkFeatureEdges.h>
#include <vtkSmartPointer.h>

#include "stkCGAL3DConvexHull.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGAL3DConvexHull
// CMake variable STK_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGAL3DConvexHull(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGAL3DConvexHull*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGAL3DConvexHullNS
{

TEST_SUITE("TestCGAL3DConvexHull")
{
  int cylinder_resolution = 8;
  auto cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();

  auto featureEdges = vtkSmartPointer<vtkFeatureEdges>::New();
  auto inputSurfaceMesh = vtkSmartPointer<vtkPolyData>::New();
  auto outputSurfaceMesh = vtkSmartPointer<vtkPolyData>::New();

  TEST_CASE("Generate Input")
  {
    cylinderSource->SetRadius(5.0);
    cylinderSource->SetHeight(5.0);
    cylinderSource->CappingOff();
    cylinderSource->SetResolution(cylinder_resolution);
    cylinderSource->SetCenter(0.0, 0.0, 0.0);
    cylinderSource->Update();

    inputSurfaceMesh->ShallowCopy(cylinderSource->GetOutput());

    REQUIRE_MESSAGE(inputSurfaceMesh != nullptr, "Input must not be NULL");

    REQUIRE_MESSAGE(inputSurfaceMesh->GetNumberOfPoints() != 0, "Input must be PointSet");

    featureEdges->BoundaryEdgesOn();
    featureEdges->FeatureEdgesOff();
    featureEdges->NonManifoldEdgesOff();
    featureEdges->ManifoldEdgesOff();
    featureEdges->SetInputData(inputSurfaceMesh);
    featureEdges->Update();

    REQUIRE_MESSAGE(featureEdges->GetOutput()->GetNumberOfCells() != 0,
      "Generated Tube Input must contain boundary edges");
  }

  TEST_CASE("Generate Convex HULL")
  {
    auto convexHullFilter = vtkSmartPointer<stkCGAL3DConvexHull>::New();
    convexHullFilter->SetInputData(inputSurfaceMesh);
    convexHullFilter->SetConvexHullMethod(stkCGAL3DConvexHull::ConvexHullMethods::QUICKHULL);
    convexHullFilter->Update();

    featureEdges->SetInputData(convexHullFilter->GetOutput());
    featureEdges->Update();

    CHECK_MESSAGE(featureEdges->GetOutput()->GetNumberOfCells() == 0,
      "Output must not contain any boundary edges");
  }
}
}