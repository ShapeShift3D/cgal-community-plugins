#include "doctest.h"

#include <vtkCylinderSource.h>
#include <vtkFieldData.h>
#include <vtkMath.h>
#include <vtkMeshQuality.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

#include "stkCGALIsotropicRemeshingFilter.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALIsotropicRemeshing
// CMake variable PARAVIEW_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALIsotropicRemeshing(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALIsotropicRemeshing*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALIsotropicRemeshingNS
{

TEST_SUITE("TestCGALIsotropicRemeshing")
{
  double cylinderHeight = 5.0;
  int cylinderRadius = 1.0;
  int cylinderResolution = 6;

  double smallestEdgeLength = 2.0 * vtkMath::Pi() * cylinderRadius / cylinderResolution;

  auto testSurface = vtkSmartPointer<vtkPolyData>::New();

  auto meshQuality = vtkSmartPointer<vtkMeshQuality>::New();

  TEST_CASE("Loops on Cylinder Test")
  {
    auto cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
    cylinderSource->SetRadius(1.0);
    cylinderSource->SetHeight(cylinderHeight);
    cylinderSource->CappingOff();
    cylinderSource->SetResolution(cylinderResolution);
    cylinderSource->SetCenter(0.0, 0.0, 0.0);
    cylinderSource->Update();

    auto triangualteFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangualteFilter->SetInputData(cylinderSource->GetOutput());
    triangualteFilter->Update();

    testSurface->ShallowCopy(triangualteFilter->GetOutput());

    REQUIRE(testSurface != nullptr);
    REQUIRE(testSurface->GetNumberOfCells() != 0);
    REQUIRE(testSurface->GetNumberOfPoints() != 0);

    meshQuality->SetInputData(testSurface);
    meshQuality->SetTriangleQualityMeasureToEdgeRatio();
    meshQuality->SaveCellQualityOff();
    meshQuality->Update();

    auto meshQualityArray =
      meshQuality->GetOutput()->GetFieldData()->GetArray("Mesh Triangle Quality");
    REQUIRE_MESSAGE(meshQualityArray != nullptr,
      "Expected a Field Array named Mesh Triangle Quality in the Test Input");
    // Check average of Edge Ratio
    CHECK(meshQualityArray->GetTuple(0)[1] ==
      doctest::Approx(cylinderHeight / smallestEdgeLength).epsilon(0.1));
  }

  TEST_CASE("Isotropic Remeshing")
  {
    auto remeshing = vtkSmartPointer<stkCGALIsotropicRemeshingFilter>::New();
    remeshing->SetInputData(testSurface);
    remeshing->ProtectConstraintsOn();
    remeshing->SetMeshingMaskArrayName("None");
    remeshing->SetTargetEdgeLength(smallestEdgeLength / 3.0);
    remeshing->SetNumberOfIterations(10);
    remeshing->Update();

    // Note : Masking Mode with User Defined is not yet covered in this Unit Test.

    auto remeshedOutput = vtkSmartPointer<vtkPolyData>::New();
    remeshedOutput->ShallowCopy(remeshing->GetOutput());

    CHECK(remeshedOutput != nullptr);
    CHECK(remeshedOutput->GetNumberOfCells() != 0);
    CHECK(remeshedOutput->GetNumberOfPoints() != 0);

    meshQuality->SetInputData(remeshedOutput);
    meshQuality->SetTriangleQualityMeasureToEdgeRatio();
    meshQuality->SaveCellQualityOff();
    meshQuality->Update();

    auto remeshedQualityArray =
      meshQuality->GetOutput()->GetFieldData()->GetArray("Mesh Triangle Quality");
    REQUIRE_MESSAGE(remeshedQualityArray != nullptr,
      "Expected a Field Array named Mesh Triangle Quality in the Test Input");
    // Check average of Edge Ratio
    CHECK_MESSAGE(remeshedQualityArray->GetTuple(0)[1] == doctest::Approx(1.5).epsilon(0.33),
      "Expected Average Edge Ratio to be between 1 to 2");
  }
}
}
