#include "doctest.h"

#include <vtkCellData.h>
#include <vtkIntegrateAttributes.h>
#include <vtkPolyLineSource.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

#include "stkCGALPolygonOffset.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALPolygonOffset
// CMake variable PARAVIEW_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALPolygonOffset(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALPolygonOffset*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALPolygonOffsetNS
{

TEST_SUITE("TestCGALPolygonOffset")
{
  TEST_CASE("Square Polygon Offset")
  {
    double sideLength = 4.0;

    auto squarePolygonSource = vtkSmartPointer<vtkPolyLineSource>::New();
    squarePolygonSource->SetNumberOfPoints(4);
    squarePolygonSource->SetPoint(0, 0.0, 0.0, 0.0);
    squarePolygonSource->SetPoint(1, sideLength, 0.0, 0.0);
    squarePolygonSource->SetPoint(2, sideLength, sideLength, 0.0);
    squarePolygonSource->SetPoint(3, 0.0, sideLength, 0.0);
    squarePolygonSource->ClosedOn();
    squarePolygonSource->Update();

    REQUIRE_MESSAGE(squarePolygonSource->GetOutput()->GetNumberOfPoints() == 4,
      "Square must consist of atleast 4 Points");

    auto integratePolygonLength = vtkSmartPointer<vtkIntegrateAttributes>::New();
    integratePolygonLength->SetInputData(squarePolygonSource->GetOutput());
    integratePolygonLength->Update();

    REQUIRE_MESSAGE(
      integratePolygonLength->GetOutput()->GetCellData()->GetArray("Length")->GetTuple1(0) ==
        doctest::Approx(4 * sideLength).epsilon(0.05),
      "Total Length of Sqaure Polygon must be 4*SideLength");

    double offsetValue = 1.0;

    auto polygonOffset = vtkSmartPointer<stkCGALPolygonOffset>::New();
    polygonOffset->SetInputData(squarePolygonSource->GetOutput());
    polygonOffset->SetOffset(offsetValue);

    SUBCASE("Interior Offset")
    {
      polygonOffset->SetOffsetType(stkCGALPolygonOffset::OffsetTypes::INTERIOR);
      polygonOffset->Update();

      integratePolygonLength->SetInputData(polygonOffset->GetOutput());
      integratePolygonLength->Update();

      CHECK_MESSAGE(
        integratePolygonLength->GetOutput()->GetCellData()->GetArray("Length")->GetTuple1(0) ==
          doctest::Approx(4 * (sideLength - 2 * offsetValue)).epsilon(0.02),
        "Total Length of Sqaure Polygon must be 4*(SideLength-2*OffsetValue)");
    }

    SUBCASE("Exterior Offset")
    {
      polygonOffset->SetOffsetType(stkCGALPolygonOffset::OffsetTypes::EXTERIOR);
      polygonOffset->Update();

      integratePolygonLength->SetInputData(polygonOffset->GetOutput());
      integratePolygonLength->Update();

      CHECK_MESSAGE(
        integratePolygonLength->GetOutput()->GetCellData()->GetArray("Length")->GetTuple1(0) ==
          doctest::Approx(4 * (sideLength + 2 * offsetValue)).epsilon(0.02),
        "Total Length of Sqaure Polygon must be 4*(SideLength+2*OffsetValue)");
    }
  }
}
}