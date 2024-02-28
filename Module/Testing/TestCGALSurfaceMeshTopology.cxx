#include "doctest.h"

#include <vtkCellData.h>
#include <vtkCylinderSource.h>
#include <vtkDoubleArray.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkMath.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

#include "stkCGALSurfaceMeshTopology.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALSurfaceMeshTopology
// CMake variable STK_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALSurfaceMeshTopology(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALSurfaceMeshTopology*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALSurfaceMeshTopologyNS
{

TEST_SUITE("TestCGALSurfaceMeshTopology")
{
  int numSubdivisions = 2;
  double cylinderRadius = 5.0;

  TEST_CASE("Loops on Cylinder Test")
  {
    auto cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
    cylinderSource->SetRadius(cylinderRadius);
    cylinderSource->SetHeight(5.0);
    cylinderSource->CappingOff();
    cylinderSource->SetResolution(30);
    cylinderSource->SetCenter(0.0, 0.0, 0.0);
    cylinderSource->Update();

    auto triangualteFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangualteFilter->SetInputData(cylinderSource->GetOutput());
    triangualteFilter->Update();

    auto subdivideFilter = vtkSmartPointer<vtkLinearSubdivisionFilter>::New();
    subdivideFilter->SetInputData(triangualteFilter->GetOutput());
    subdivideFilter->SetNumberOfSubdivisions(numSubdivisions);
    subdivideFilter->Update();

    auto testInput = vtkSmartPointer<vtkPolyData>::New();
    testInput->ShallowCopy(subdivideFilter->GetOutput());

    auto vertexMaskArray = vtkSmartPointer<vtkIntArray>::New();
    vertexMaskArray->SetNumberOfComponents(1);
    vertexMaskArray->SetNumberOfTuples(testInput->GetNumberOfPoints());
    vertexMaskArray->SetName("Base Point Mask Array");
    vertexMaskArray->Fill(1.0);

    testInput->GetPointData()->AddArray(vertexMaskArray);

    auto cycleGeneration = vtkSmartPointer<stkCGALSurfaceMeshTopology>::New();
    cycleGeneration->SetInputData(testInput);
    cycleGeneration->SetBasePointMaskArrayName("Base Point Mask Array");
    cycleGeneration->GenerateCycleIDsOn();
    cycleGeneration->SetCycleIDArrayName("Cycle No");
    cycleGeneration->CalculateCycleLengthOn();
    cycleGeneration->SetCycleLengthArrayName("Cycle Length");
    cycleGeneration->Update();

    auto testOutput = vtkSmartPointer<vtkPolyData>::New();
    testOutput->ShallowCopy(cycleGeneration->GetOutput());

    auto numGeneratedCycles = testOutput->GetCellData()->GetArray("Cycle No")->GetRange()[1] + 1;

    double expectedCycleLength = 2.0 * vtkMath::Pi() * cylinderRadius;

    for (int cycleId = 0; cycleId < numGeneratedCycles; cycleId++)
    {
      auto cycleLength = testOutput->GetCellData()->GetArray("Cycle Length")->GetTuple1(cycleId);
      CHECK(cycleLength == doctest::Approx(expectedCycleLength).epsilon(0.05));
    }
  }
}
}