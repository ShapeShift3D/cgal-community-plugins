#include "doctest.h"

#include <vtkMath.h>
#include <vtkPolyData.h>
#include <vtkPolyLineSource.h>

#include "stkCGALPolygonOrientOperator.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALPolygonOrientOperator
// CMake variable PARAVIEW_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALPolygonOrientOperator(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALPolygonOrientOperator*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALPolygonOrientOperatorNS
{

vtkSmartPointer<vtkPolyData> GenerateSquareLoop(double side)
{
  double halfSide = side / 2.0;
  auto loopSource = vtkSmartPointer<vtkPolyLineSource>::New();
  loopSource->SetNumberOfPoints(4);
  loopSource->SetPoint(0, halfSide, halfSide, 0.0);
  loopSource->SetPoint(1, -halfSide, halfSide, 0.0);
  loopSource->SetPoint(2, -halfSide, -halfSide, 0.0);
  loopSource->SetPoint(3, halfSide, -halfSide, 0.0);
  loopSource->ClosedOn();
  loopSource->Update();

  return loopSource->GetOutput();
}

void NormalizedCrossProduct(vtkPolyData* loop, double crossProduct[3])
{
  double point0[3] = { 0.0 };
  double point1[3] = { 0.0 };
  double point2[3] = { 0.0 };

  loop->GetPoint(0, point0);
  loop->GetPoint(1, point1);
  loop->GetPoint(2, point2);

  double vec1[3] = { 0.0 };
  double vec2[3] = { 0.0 };

  vtkMath::Subtract(point1, point0, vec1);
  vtkMath::Subtract(point2, point1, vec2);

  vtkMath::Cross(vec1, vec2, crossProduct);
  vtkMath::Normalize(crossProduct);
}

TEST_SUITE("TestCGALPolygonOrientOperator")
{
  auto squareLoop = vtkSmartPointer<vtkPolyData>::New();

  double crossProduct[3] = { 0.0 };

  double positiveZDirection[3] = { 0.0, 0.0, 1.0 };
  double negativeZDirection[3] = { 0.0, 0.0, -1.0 };

  TEST_CASE("Generate Input Spline Loops")
  {
    squareLoop = TestCGALPolygonOrientOperatorNS::GenerateSquareLoop(5.0);

    REQUIRE(squareLoop != nullptr);
    REQUIRE(squareLoop->GetNumberOfPoints() > 3);
    REQUIRE(squareLoop->GetNumberOfCells() == 1);

    TestCGALPolygonOrientOperatorNS::NormalizedCrossProduct(squareLoop, crossProduct);

    // Check the Original Loop Orientation is Counterclockwise
    REQUIRE(vtkMath::Dot(crossProduct, positiveZDirection) == doctest::Approx(1.0).epsilon(0.01));
  }

  TEST_CASE("Orient Loop")
  {
    auto orientOperator = vtkSmartPointer<stkCGALPolygonOrientOperator>::New();
    orientOperator->SetInputData(0, squareLoop);
    orientOperator->SetPlaneToXY();
    orientOperator->InvertPolyLineOrientationOff();
    orientOperator->ForcePolyLineOrientationOff();

    SUBCASE("Invert Orientation")
    {
      orientOperator->InvertPolyLineOrientationOn();
      orientOperator->Update();

      TestCGALPolygonOrientOperatorNS::NormalizedCrossProduct(
        orientOperator->GetOutput(), crossProduct);

      CHECK(vtkMath::Dot(crossProduct, negativeZDirection) == doctest::Approx(1.0).epsilon(0.01));
    }

    SUBCASE("Force Orientation : Clockwise")
    {
      orientOperator->ForcePolyLineOrientationOn();

      SUBCASE("Clockwise")
      {
        orientOperator->SetPolyLineAOrientationToClockwise();
        orientOperator->Update();

        TestCGALPolygonOrientOperatorNS::NormalizedCrossProduct(
          orientOperator->GetOutput(), crossProduct);

        CHECK(
          vtkMath::Dot(crossProduct, negativeZDirection) == doctest::Approx(1.0).epsilon(0.01));
      }

      SUBCASE("CounterClockwise")
      {
        orientOperator->SetPolyLineAOrientationToCounterclockwise();
        orientOperator->Update();

        TestCGALPolygonOrientOperatorNS::NormalizedCrossProduct(
          orientOperator->GetOutput(), crossProduct);

        CHECK(
          vtkMath::Dot(crossProduct, positiveZDirection) == doctest::Approx(1.0).epsilon(0.01));
      }
    }
  }
}
}