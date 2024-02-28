#include "doctest.h"

#include <vtkCellData.h>
#include <vtkIntegrateAttributes.h>
#include <vtkPolyData.h>
#include <vtkPolyLineSource.h>
#include <vtkUnstructuredGrid.h>

#include "stkCGALBoolean2DMesher.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALBoolean2DMesher
// CMake variable STK_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALBoolean2DMesher(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALBoolean2DMesher*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALBoolean2DMesherNS
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

TEST_SUITE("TestCGALBoolean2DMesher")
{
  auto squareLoopA = vtkSmartPointer<vtkPolyData>::New();
  double sideA = 5.0;

  auto squareLoopB = vtkSmartPointer<vtkPolyData>::New();
  double sideB = 2.0;

  auto lengthCalculator = vtkSmartPointer<vtkIntegrateAttributes>::New();

  TEST_CASE("Generate Input Spline Loops")
  {
    squareLoopA = TestCGALBoolean2DMesherNS::GenerateSquareLoop(sideA);

    REQUIRE(squareLoopA != nullptr);

    lengthCalculator->SetDivideAllCellDataByVolume(false);
    lengthCalculator->SetInputDataObject(squareLoopA);
    lengthCalculator->Update();

    REQUIRE(lengthCalculator->GetOutput() != nullptr);
    REQUIRE(lengthCalculator->GetOutput()->GetNumberOfCells() != 0);
    REQUIRE(lengthCalculator->GetOutput()->GetCellData()->GetArray("Length") != nullptr);
    REQUIRE(lengthCalculator->GetOutput()->GetCellData()->GetArray("Length")->GetTuple1(0) ==
      doctest::Approx(4 * sideA).epsilon(0.01));

    squareLoopB = TestCGALBoolean2DMesherNS::GenerateSquareLoop(sideB);

    REQUIRE(squareLoopB != nullptr);

    lengthCalculator->SetInputDataObject(squareLoopB);
    lengthCalculator->Update();

    REQUIRE(lengthCalculator->GetOutput() != nullptr);
    REQUIRE(lengthCalculator->GetOutput()->GetNumberOfCells() != 0);
    REQUIRE(lengthCalculator->GetOutput()->GetCellData()->GetArray("Length") != nullptr);
    REQUIRE(lengthCalculator->GetOutput()->GetCellData()->GetArray("Length")->GetTuple1(0) ==
      doctest::Approx(4 * sideB).epsilon(0.01));
  }

  TEST_CASE("Boolean 2D")
  {
    auto boolean2D = vtkSmartPointer<stkCGALBoolean2DMesher>::New();
    boolean2D->SetInputData(0, squareLoopA);
    boolean2D->SetInputData(1, squareLoopB);
    boolean2D->OneCellOff();
    boolean2D->SetPlaneToXY();

    lengthCalculator->SetInputConnection(boolean2D->GetOutputPort(0));

    SUBCASE("Add")
    {
      boolean2D->SetOperationModeToAdd();

      lengthCalculator->Update();

      CHECK(lengthCalculator->GetOutput() != nullptr);
      CHECK(lengthCalculator->GetOutput()->GetNumberOfCells() != 0);
      CHECK(lengthCalculator->GetOutput()->GetCellData()->GetArray("Length") != nullptr);
      CHECK(lengthCalculator->GetOutput()->GetCellData()->GetArray("Length")->GetTuple1(0) ==
        doctest::Approx(4 * sideA).epsilon(0.01));
    }

    SUBCASE("Intersect")
    {
      boolean2D->SetOperationModeToIntersect();

      lengthCalculator->Update();

      CHECK(lengthCalculator->GetOutput() != nullptr);
      CHECK(lengthCalculator->GetOutput()->GetNumberOfCells() != 0);
      CHECK(lengthCalculator->GetOutput()->GetCellData()->GetArray("Length") != nullptr);
      CHECK(lengthCalculator->GetOutput()->GetCellData()->GetArray("Length")->GetTuple1(0) ==
        doctest::Approx(4 * sideB).epsilon(0.01));
    }

    SUBCASE("A-B")
    {
      boolean2D->SetOperationModeToAMinusB();

      lengthCalculator->Update();

      CHECK(lengthCalculator->GetOutput() != nullptr);
      CHECK(lengthCalculator->GetOutput()->GetNumberOfCells() != 0);
      CHECK(lengthCalculator->GetOutput()->GetCellData()->GetArray("Length") != nullptr);
      CHECK(lengthCalculator->GetOutput()->GetCellData()->GetArray("Length")->GetTuple1(0) ==
        doctest::Approx(4 * (sideA + sideB)).epsilon(0.01));
    }

    // Following modes are not yet covered in the unit test
    // SetOperationModeToBMinusA
    // SetOperationModeToExcludeOverlap
    // SetOperationModeToComplement
    // SetOperationModeToIntersectComplement
    // SetOperationModeToExclusiveAdd
  }
}
}