#include "doctest.h"

#include <vtkCellData.h>
#include <vtkIntegrateAttributes.h>
#include <vtkMath.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkUnstructuredGrid.h>

#include "stkCGALBoolean3DMesher.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALBoolean3DMesher
// CMake variable STK_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALBoolean3DMesher(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALBoolean3DMesher*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALBoolean3DMesherNS
{

vtkSmartPointer<vtkPolyData> GenerateSphere(double radius)
{
  auto sphereSource = vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetRadius(radius);
  sphereSource->SetThetaResolution(8);
  sphereSource->SetPhiResolution(8);
  sphereSource->SetStartTheta(0.0);
  sphereSource->SetEndTheta(360.0);
  sphereSource->SetStartPhi(0.0);
  sphereSource->SetEndPhi(180.0);
  sphereSource->Update();

  return sphereSource->GetOutput();
}

TEST_SUITE("TestCGALBoolean3DMesher")
{
  auto sphereA = vtkSmartPointer<vtkPolyData>::New();
  double radiusA = 5.0;

  auto sphereB = vtkSmartPointer<vtkPolyData>::New();
  double radiusB = 2.0;

  auto surfaceAreaCalculator = vtkSmartPointer<vtkIntegrateAttributes>::New();

  double areaSphereA = 0.0;
  double areaSphereB = 0.0;

  TEST_CASE("Generate Input Spline Loops")
  {
    sphereA = TestCGALBoolean3DMesherNS::GenerateSphere(radiusA);

    REQUIRE(sphereA != nullptr);

    surfaceAreaCalculator->SetDivideAllCellDataByVolume(false);
    surfaceAreaCalculator->SetInputDataObject(sphereA);
    surfaceAreaCalculator->Update();

    REQUIRE(surfaceAreaCalculator->GetOutput() != nullptr);
    REQUIRE(surfaceAreaCalculator->GetOutput()->GetNumberOfCells() != 0);
    REQUIRE(surfaceAreaCalculator->GetOutput()->GetCellData()->GetArray("Area") != nullptr);

    areaSphereA = surfaceAreaCalculator->GetOutput()->GetCellData()->GetArray("Area")->GetTuple1(0);
    REQUIRE(areaSphereA == doctest::Approx(4 * vtkMath::Pi() * std::pow(radiusA, 2)).epsilon(0.20));

    sphereB = TestCGALBoolean3DMesherNS::GenerateSphere(radiusB);

    REQUIRE(sphereB != nullptr);

    surfaceAreaCalculator->SetInputDataObject(sphereB);
    surfaceAreaCalculator->Update();

    REQUIRE(surfaceAreaCalculator->GetOutput() != nullptr);
    REQUIRE(surfaceAreaCalculator->GetOutput()->GetNumberOfCells() != 0);
    REQUIRE(surfaceAreaCalculator->GetOutput()->GetCellData()->GetArray("Area") != nullptr);
    areaSphereB = surfaceAreaCalculator->GetOutput()->GetCellData()->GetArray("Area")->GetTuple1(0);
    REQUIRE(areaSphereB == doctest::Approx(4 * vtkMath::Pi() * std::pow(radiusB, 2)).epsilon(0.20));
  }

  TEST_CASE("Boolean 3D")
  {
    auto boolean3D = vtkSmartPointer<stkCGALBoolean3DMesher>::New();
    boolean3D->SetInputData(0, sphereA);
    boolean3D->SetInputData(1, sphereB);

    surfaceAreaCalculator->SetInputConnection(boolean3D->GetOutputPort(0));

    SUBCASE("Union")
    {
      boolean3D->SetModeToUnion();

      surfaceAreaCalculator->Update();

      CHECK(surfaceAreaCalculator->GetOutput() != nullptr);
      CHECK(surfaceAreaCalculator->GetOutput()->GetNumberOfCells() != 0);
      CHECK(surfaceAreaCalculator->GetOutput()->GetCellData()->GetArray("Area") != nullptr);
      CHECK(surfaceAreaCalculator->GetOutput()->GetCellData()->GetArray("Area")->GetTuple1(0) ==
        doctest::Approx(areaSphereA).epsilon(0.01));
    }

    SUBCASE("Intersection")
    {
      boolean3D->SetModeToIntersection();

      surfaceAreaCalculator->Update();

      CHECK(surfaceAreaCalculator->GetOutput() != nullptr);
      CHECK(surfaceAreaCalculator->GetOutput()->GetNumberOfCells() != 0);
      CHECK(surfaceAreaCalculator->GetOutput()->GetCellData()->GetArray("Area") != nullptr);
      CHECK(surfaceAreaCalculator->GetOutput()->GetCellData()->GetArray("Area")->GetTuple1(0) ==
        doctest::Approx(areaSphereB).epsilon(0.01));
    }

    SUBCASE("Difference (A-B)")
    {
      boolean3D->SetModeToDifference();

      surfaceAreaCalculator->Update();

      CHECK(surfaceAreaCalculator->GetOutput() != nullptr);
      CHECK(surfaceAreaCalculator->GetOutput()->GetNumberOfCells() != 0);
      CHECK(surfaceAreaCalculator->GetOutput()->GetCellData()->GetArray("Area") != nullptr);
      CHECK(surfaceAreaCalculator->GetOutput()->GetCellData()->GetArray("Area")->GetTuple1(0) ==
        doctest::Approx(areaSphereA + areaSphereB).epsilon(0.01));
    }

    SUBCASE("Difference 2 (B-A)")
    {
      boolean3D->SetModeToDifference2();
      boolean3D->Update();

      CHECK(boolean3D->GetOutput()->GetNumberOfCells() == 0);
      CHECK(boolean3D->GetOutput()->GetNumberOfPoints() == 0);
    }
  }
}
}