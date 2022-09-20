#include "doctest.h"
#include "stkCGALPoissonReconstructionMesher.h"
#include <vtkFeatureEdges.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGALPoissonReconstructionMesher
// CMake variable PARAVIEW_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALPoissonReconstructionMesher(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGALPoissonReconstructionMesher*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGALPoissonReconstructionMesherNS
{

TEST_SUITE("TestCGALPoissonReconstructionMesher")
{
  TEST_CASE("Sphere")
  {
    auto input = vtkSmartPointer<vtkSphereSource>::New();
    input->SetCenter(0, 0, 0);
    input->SetRadius(0.5);
    input->SetThetaResolution(100);
    input->SetStartTheta(0);
    input->SetEndTheta(300);
    input->SetPhiResolution(100);
    input->SetStartPhi(0);
    input->SetEndPhi(180);
    input->Update();

    auto CGALPoissonReconstructionMesher =
      vtkSmartPointer<stkCGALPoissonReconstructionMesher>::New();
    CGALPoissonReconstructionMesher->SetInputData(input->GetOutput());
    CGALPoissonReconstructionMesher->SetMinTriangleAngle(20);
    CGALPoissonReconstructionMesher->SetMaxTriangleSizeMultiplier(100);
    CGALPoissonReconstructionMesher->SetApproximationErrorMultiplier(0.25);
    CGALPoissonReconstructionMesher->SetUseNormalEstimation(false);
    CGALPoissonReconstructionMesher->Update();

    vtkSmartPointer<vtkFeatureEdges> boundaryEdges = vtkSmartPointer<vtkFeatureEdges>::New();
    boundaryEdges->SetInputData(CGALPoissonReconstructionMesher->GetOutput());
    boundaryEdges->BoundaryEdgesOn();
    boundaryEdges->FeatureEdgesOff();
    boundaryEdges->ManifoldEdgesOff();
    boundaryEdges->NonManifoldEdgesOff();
    boundaryEdges->Update();

    CHECK_MESSAGE(boundaryEdges->GetOutput()->GetNumberOfPoints() == 0, "Output must be closed.");
  }
}
}