#include "doctest.h"

#include <vtkCellTypes.h>
#include <vtkPolyLineSource.h>
#include <vtkSmartPointer.h>

#include "stkCGAL2DConstrainedDelaunayTriangulationMesher.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGAL2DConstrainedDelaunayTriangulationMesher
// CMake variable STK_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGAL2DConstrainedDelaunayTriangulationMesher(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGAL2DConstrainedDelaunayTriangulationMesher*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGAL2DConstrainedDelaunayTriangulationMesherNS
{

TEST_SUITE("TestCGAL2DConstrainedDelaunayTriangulationMesher")
{
  TEST_CASE("Triangulate Square")
  {
    int numPointsLineSource = 4;

    double point0[3] = { 0.0, 0.0, 0.0 };
    double point1[3] = { 1.0, 0.0, 0.0 };
    double point2[3] = { 1.0, 1.0, 0.0 };
    double point3[3] = { 0.0, 1.0, 0.0 };

    auto input = vtkSmartPointer<vtkPolyLineSource>::New();
    input->SetNumberOfPoints(numPointsLineSource);
    input->SetPoint(0, point0[0], point0[1], point0[2]);
    input->SetPoint(1, point1[0], point1[1], point1[2]);
    input->SetPoint(2, point2[0], point2[1], point2[2]);
    input->SetPoint(3, point3[0], point3[1], point3[2]);
    input->ClosedOn();
    input->Update();

    auto CGAL2DConstrainedDelaunayTriangulationMesher =
      vtkSmartPointer<stkCGAL2DConstrainedDelaunayTriangulationMesher>::New();
    CGAL2DConstrainedDelaunayTriangulationMesher->SetInputData(input->GetOutput());
    CGAL2DConstrainedDelaunayTriangulationMesher->Update();

    bool triangleCellsCheck = false;

    int vtkCellType = VTKCellType::VTK_TRIANGLE;

    auto cellsType = vtkSmartPointer<vtkCellTypes>::New();

    CGAL2DConstrainedDelaunayTriangulationMesher->GetOutput()->GetCellTypes(cellsType);

    int numCellTypes = cellsType->GetNumberOfTypes();

    if (numCellTypes > 1 || numCellTypes == 0)
    {
      triangleCellsCheck = 0;
    }
    else if (cellsType->GetCellType(0) != vtkCellType)
    {
      triangleCellsCheck = 0;
    }
    else
    {
      triangleCellsCheck = 1;
    }

    CHECK_MESSAGE(
      CGAL2DConstrainedDelaunayTriangulationMesher->GetOutput()->GetNumberOfCells() == 2,
      "Output must contain 2 cells.");

    CHECK_MESSAGE(
      CGAL2DConstrainedDelaunayTriangulationMesher->GetOutput()->GetNumberOfPoints() == 4,
      "Output must contain 4 points.");

    CHECK_MESSAGE(triangleCellsCheck, "Output must only contain triangles.");
  }
}
}