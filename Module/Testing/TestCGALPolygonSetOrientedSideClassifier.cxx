#include "doctest.h"

#include <vtkAppendPolyData.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkPolyLineSource.h>

#include <string>

#include "stkCGALPolygonSetOrientedSideClassifier.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALPolygonSetOrientedSideClassifier
// CMake variable PARAVIEW_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALPolygonSetOrientedSideClassifier(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALPolygonSetOrientedSideClassifier*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALPolygonSetOrientedSideClassifierNS
{

vtkSmartPointer<vtkPolyData> GenerateCounterClockwiseSquareLoop(double side)
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

vtkSmartPointer<vtkPolyData> GenerateClockwiseSquareLoop(double side)
{
  double halfSide = side / 2.0;
  auto loopSource = vtkSmartPointer<vtkPolyLineSource>::New();
  loopSource->SetNumberOfPoints(4);
  loopSource->SetPoint(0, halfSide, halfSide, 0.0);
  loopSource->SetPoint(1, halfSide, -halfSide, 0.0);
  loopSource->SetPoint(2, -halfSide, -halfSide, 0.0);
  loopSource->SetPoint(3, -halfSide, halfSide, 0.0);

  loopSource->ClosedOn();
  loopSource->Update();

  return loopSource->GetOutput();
}

TEST_SUITE("TestCGALPolygonSetOrientedSideClassifier")
{
  auto loopSet = vtkSmartPointer<vtkPolyData>::New();

  double innerLoopSide = 2.0;
  double outerLoopSide = 5.0;

  std::string orientedSideArrayName = "OrientedSideResult";

  TEST_CASE("Generate Input Spline Loops")
  {
    // Sense Clockwise
    auto innerLoop =
      TestCGALPolygonSetOrientedSideClassifierNS::GenerateClockwiseSquareLoop(innerLoopSide);

    REQUIRE(innerLoop != nullptr);

    // Sense CounterClockwise
    auto outerLoop =
      TestCGALPolygonSetOrientedSideClassifierNS::GenerateCounterClockwiseSquareLoop(outerLoopSide);

    REQUIRE(outerLoop != nullptr);

    auto appendLoops = vtkSmartPointer<vtkAppendPolyData>::New();
    appendLoops->AddInputData(outerLoop);
    appendLoops->AddInputData(innerLoop);
    appendLoops->Update();

    loopSet->ShallowCopy(appendLoops->GetOutput());

    REQUIRE(loopSet != nullptr);
  }

  TEST_CASE("Oriented Side Classifier")
  {
    auto sideClassifier = vtkSmartPointer<stkCGALPolygonSetOrientedSideClassifier>::New();
    sideClassifier->SetInputData(1, loopSet);
    sideClassifier->SetPlaneToXY();
    sideClassifier->SetPwhIdArrayName("None");
    sideClassifier->SetOrientedSideArrayName(orientedSideArrayName);

    auto pointData = vtkSmartPointer<vtkPolyData>::New();

    auto pointSource = vtkSmartPointer<vtkPointSource>::New();
    pointSource->SetNumberOfPoints(1);
    pointSource->SetRadius(0.0);

    SUBCASE("Inside Inner Loop")
    {
      pointSource->SetCenter(0.0, 0.0, 0.0);
      pointSource->Update();

      pointData->ShallowCopy(pointSource->GetOutput());

      CHECK(pointData != nullptr);

      sideClassifier->SetInputData(0, pointData);
      sideClassifier->Update();

      auto dataArray =
        sideClassifier->GetOutput()->GetPointData()->GetArray(orientedSideArrayName.c_str());
      CHECK(dataArray != nullptr);

      CHECK(dataArray->GetTuple1(0) == -1.0);
    }

    SUBCASE("Between Inner and Outer Loop")
    {
      double betweenVal = (innerLoopSide/2.0 + outerLoopSide/2.0) / 2.0;
      pointSource->SetCenter(betweenVal, betweenVal, 0.0);
      pointSource->Update();

      pointData->ShallowCopy(pointSource->GetOutput());

      CHECK(pointData != nullptr);

      sideClassifier->SetInputData(0, pointData);
      sideClassifier->Update();

      auto dataArray =
        sideClassifier->GetOutput()->GetPointData()->GetArray(orientedSideArrayName.c_str());
      CHECK(dataArray != nullptr);

      CHECK(dataArray->GetTuple1(0) == 1.0);
    }

    SUBCASE("Outside Outer Loop")
    {
      double outsideVal = outerLoopSide/2.0 + 3.0;
      pointSource->SetCenter(outsideVal, outsideVal, 0.0);
      pointSource->Update();

      pointData->ShallowCopy(pointSource->GetOutput());

      CHECK(pointData != nullptr);

      sideClassifier->SetInputData(0, pointData);
      sideClassifier->Update();

      auto dataArray =
        sideClassifier->GetOutput()->GetPointData()->GetArray(orientedSideArrayName.c_str());
      CHECK(dataArray != nullptr);

      CHECK(dataArray->GetTuple1(0) == -1.0);
    }
  }
}
}