#include "doctest.h"

#include <vtkMassProperties.h>
#include <vtkMath.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>

#include "stkCGALARAPUVParametrization.h"
#include "stkCGALLSCMUVParametrization.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALUVParametrization
// CMake variable PARAVIEW_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALUVParametrization(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALUVParametrization*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALUVParametrizationNS
{

vtkSmartPointer<vtkPolyData> GenerateSemiSphere(double radius)
{
  auto sphereSource = vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetRadius(radius);
  sphereSource->SetThetaResolution(25); // Testing with generous resolution 
  sphereSource->SetPhiResolution(25); // Testing with generous resolution 
  sphereSource->SetStartTheta(0.0);
  sphereSource->SetEndTheta(360.0);
  sphereSource->SetStartPhi(0.0);
  sphereSource->SetEndPhi(90.0);
  sphereSource->Update();

  return sphereSource->GetOutput();
}

TEST_SUITE("TestCGALUVParametrization")
{
  double radius = 7.0;
  auto inputSurface = vtkSmartPointer<vtkPolyData>::New();

  auto surfaceAreaCalculator = vtkSmartPointer<vtkMassProperties>::New();

  double inputSurfaceArea = 0.0;

  TEST_CASE("Generate Inputs")
  {
    inputSurface = TestCGALUVParametrizationNS::GenerateSemiSphere(radius);

    REQUIRE(inputSurface != nullptr);

    surfaceAreaCalculator->SetInputData(inputSurface);
    surfaceAreaCalculator->Update();

    inputSurfaceArea = surfaceAreaCalculator->GetSurfaceArea();

    double expectedSurfaceArea = 2 * vtkMath::Pi() * std::pow(radius, 2);
    CHECK(inputSurfaceArea == doctest::Approx(expectedSurfaceArea).epsilon(0.25));
  }

  TEST_CASE("LSCM UV Mapping")
  {
    auto uvParameterization = vtkSmartPointer<stkCGALLSCMUVParametrization>::New();
    uvParameterization->SetInputData(inputSurface);

    uvParameterization->SetScalingModeToManual();
    uvParameterization->SetUVScaling(1);
    uvParameterization->Update();

    // Zmin and Zmax
    CHECK_MESSAGE(
      uvParameterization->GetOutput()->GetBounds()[5] == 0.0, "Surface Mesh is not in uv domain");
    CHECK_MESSAGE(
      uvParameterization->GetOutput()->GetBounds()[4] == 0.0, "Surface Mesh is not in uv domain");

    // Note : we can not check area for LSCM because it attempts to preserve angles.
  }

  TEST_CASE("ARAP UV Mapping")
  {
    auto arapParameterization = vtkSmartPointer<stkCGALARAPUVParametrization>::New();
    arapParameterization->SetInputData(inputSurface);
    arapParameterization->SetLambda(10000);
    arapParameterization->SetMaximumNumberOfIterations(10);
    arapParameterization->SetTolerance(1e-6);
    arapParameterization->SkipPostprocessOff();

    arapParameterization->SetScalingModeToManual();
    arapParameterization->SetUVScaling(1);
    arapParameterization->Update();

    // Zmin and Zmax
    CHECK_MESSAGE(
      arapParameterization->GetOutput()->GetBounds()[5] == 0.0, "Surface Mesh is not in uv domain");
    CHECK_MESSAGE(
      arapParameterization->GetOutput()->GetBounds()[4] == 0.0, "Surface Mesh is not in uv domain");

    surfaceAreaCalculator->SetInputData(arapParameterization->GetOutput());
    surfaceAreaCalculator->Update();

    CHECK_MESSAGE(
      surfaceAreaCalculator->GetSurfaceArea() == doctest::Approx(inputSurfaceArea).epsilon(0.1),
      "Surface Area of UV Parameterization is not within expected range");
  }
}
}