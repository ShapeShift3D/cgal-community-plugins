#include "doctest.h"

#include <vtkCylinderSource.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkMassProperties.h>
#include <vtkMath.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

#include "stkCGALAlphaWrapping.h"
// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALAlphaWrapping
// CMake variable PARAVIEW_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALAlphaWrapping(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALAlphaWrapping*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALAlphaWrappingNS
{

TEST_SUITE("TestCGALAlphaWrapping")
{
  double cylinderHeight = 5.0;
  int cylinderRadius = 1.0;
  int cylinderResolution = 6;

  auto testSurface = vtkSmartPointer<vtkPolyData>::New();

  TEST_CASE("Generate Cylinder with Fair Resolution")
  {
    auto cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
    cylinderSource->SetRadius(cylinderRadius);
    cylinderSource->SetHeight(cylinderHeight);
    cylinderSource->CappingOff();
    cylinderSource->SetResolution(cylinderResolution);
    cylinderSource->SetCenter(0.0, 0.0, 0.0);
    cylinderSource->Update();

    auto triangualteFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangualteFilter->SetInputData(cylinderSource->GetOutput());
    triangualteFilter->Update();

    auto subdivideFilter = vtkSmartPointer<vtkLinearSubdivisionFilter>::New();
    subdivideFilter->SetInputData(triangualteFilter->GetOutput());
    subdivideFilter->SetNumberOfSubdivisions(static_cast<int>(cylinderHeight / 2 * cylinderRadius));
    subdivideFilter->Update();

    testSurface->ShallowCopy(subdivideFilter->GetOutput());

    REQUIRE(testSurface != nullptr);
    REQUIRE(testSurface->GetNumberOfPoints() != 0);
  }

  TEST_CASE("3D Alpha Wrapping")
  {
    auto wrapping = vtkSmartPointer<stkCGALAlphaWrapping>::New();
    wrapping->SetInputData(0, testSurface);
    wrapping->SetAlpha(cylinderRadius * 2 + 1.0); // Alpha is kept bigger than largest 
    wrapping->SetOffset(0.1); // Keeping the Wrap really Tight
    wrapping->Update();

    auto wrap = vtkSmartPointer<vtkPolyData>::New();
    wrap->ShallowCopy(wrapping->GetOutput());

    auto bounds = wrap->GetBounds();
    // X Bounds 
    CHECK(bounds[1] - bounds[0] == doctest::Approx(2 * cylinderRadius).epsilon(0.2));
    // Y Bounds
    CHECK(bounds[3] - bounds[2] == doctest::Approx(cylinderHeight).epsilon(0.2));
    // Z Bounds
    CHECK(bounds[5] - bounds[4] == doctest::Approx(2 * cylinderRadius).epsilon(0.2));

    auto massProperties = vtkSmartPointer<vtkMassProperties>::New();
    massProperties->SetInputData(wrap);

    double calculatedVolume = massProperties->GetVolume();

    double expectedVolume = vtkMath::Pi() * std::pow(cylinderRadius, 2) * cylinderHeight;

    CHECK(calculatedVolume == doctest::Approx(expectedVolume).epsilon(0.1));
  }
}
}
