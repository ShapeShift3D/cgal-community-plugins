#include "doctest.h"

#include <vtkCylinderSource.h>
#include <vtkFeatureEdges.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

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
  TEST_CASE("Close Tube")
  {
    int cylinder_resolution = 8;

    auto cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
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

    auto featureEdges = vtkSmartPointer<vtkFeatureEdges>::New();
    featureEdges->BoundaryEdgesOn();
    featureEdges->FeatureEdgesOff();
    featureEdges->NonManifoldEdgesOff();
    featureEdges->ManifoldEdgesOff();

    featureEdges->SetInputData(subdivideFilter->GetOutput());
    featureEdges->Update();

    REQUIRE_MESSAGE(featureEdges->GetOutput()->GetNumberOfCells() != 0,
      "Generated Tube Input contains boundary edges");

    auto cgalFillHoles = vtkSmartPointer<stkCGALFillHoles>::New();
    cgalFillHoles->SetInputData(featureEdges->GetOutput());
    cgalFillHoles->UseDelaunayTriangulationOn();
    cgalFillHoles->SetFillingType(stkCGALFillHoles::FillingTypes::FILLHOLES_REFINE_FAIR);
    cgalFillHoles->SetFairingContinuity(stkCGALFillHoles::FairingContinuityTypes::C1);
    cgalFillHoles->EnableMaxHoleBBDiagonalOff();
    cgalFillHoles->SkipSelfIntersectingPatchesOff();
    cgalFillHoles->Update();

    featureEdges->SetInputData(cgalFillHoles->GetOutput());
    featureEdges->Update();

    CHECK_MESSAGE(featureEdges->GetOutput()->GetNumberOfCells() == 0,
      "Output must not contain any boundary edges");
  }
}
}