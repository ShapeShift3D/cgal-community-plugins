#include "doctest.h"

#include <vtkCellData.h>
#include <vtkIntegrateAttributes.h>
#include <vtkMassProperties.h>
#include <vtkMath.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkUnstructuredGrid.h>
#include <vtkPolyData.h>

#include "stkCGAL3DPolyhedralMesher.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe TestCGAL3DPolyhedralMesher
// CMake variable STK_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGAL3DPolyhedralMesher(int argc, char** const argv)
{
  doctest::Context context;

  // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
  context.setOption("force-colors", true);
  context.setOption("duration", true);
  context.setOption("test-suite", "TestCGAL3DPolyhedralMesher*");

  // Command line can be used to override above parameters.
  context.applyCommandLine(argc, argv);

  return context.run();
}

namespace TestCGAL3DPolyhedralMesherNS
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

TEST_SUITE("TestCGAL3DPolyhedralMesher")
{
  double inputRadius = 7.0;
  auto inputMesh = vtkSmartPointer<vtkPolyData>::New();

  double domainRadius = 9.0;
  auto boundingDomain = vtkSmartPointer<vtkPolyData>::New();

  TEST_CASE("Generate Inputs")
  {
    // Small Sphere
    inputMesh = TestCGAL3DPolyhedralMesherNS::GenerateSphere(inputRadius);

    REQUIRE(inputMesh != nullptr);

    // Large Sphere
    boundingDomain = TestCGAL3DPolyhedralMesherNS::GenerateSphere(domainRadius);

    REQUIRE(boundingDomain != nullptr);
  }

  TEST_CASE("Generate Volume Mesh")
  {
    auto cgaltetvolmesher = vtkSmartPointer<stkCGAL3DPolyhedralMesher>::New();
    cgaltetvolmesher->SetInputData(0, inputMesh);
    cgaltetvolmesher->SetInputData(1, boundingDomain);
    cgaltetvolmesher->SetEdgeSize(5.0);
    cgaltetvolmesher->SetFacetAngle(25.0);
    cgaltetvolmesher->SetFacetSize(10.0);
    cgaltetvolmesher->SetFacetDistance(1.0);
    cgaltetvolmesher->SetCellRadiusEdgeRatio(3.0);
    cgaltetvolmesher->SetCellSize(10.0);

    cgaltetvolmesher->ConfineSurfacePointsOn();

    // Only case of Manifold Interior surface is being covered in this Unit Test
    cgaltetvolmesher->SetTopologicalStructureToManifold();

    // Not Covered in the Unit Test. It requires Tet Vol Signed Distance that is in libigl Non-Free
    // Module.
    cgaltetvolmesher->UseCustomSizingFieldOff();

    // Optimisers are also not covered in
    cgaltetvolmesher->LloydOff();
    cgaltetvolmesher->OdtOff();
    cgaltetvolmesher->PerturbOff();
    cgaltetvolmesher->ExudeOff();

    cgaltetvolmesher->Update();

    REQUIRE(cgaltetvolmesher->GetOutput() != nullptr);

    auto volumeCalculator = vtkSmartPointer<vtkIntegrateAttributes>::New();
    volumeCalculator->SetDivideAllCellDataByVolume(false);
    volumeCalculator->SetInputDataObject(cgaltetvolmesher->GetOutputDataObject(0));
    volumeCalculator->Update();

    REQUIRE(volumeCalculator->GetOutput() != nullptr);
    REQUIRE(volumeCalculator->GetOutput()->GetNumberOfCells() != 0);
    REQUIRE(volumeCalculator->GetOutput()->GetCellData()->GetArray("Volume") != nullptr);

    double calculatedVolume =
      volumeCalculator->GetOutput()->GetCellData()->GetArray("Volume")->GetTuple1(0);

    auto massProperties = vtkSmartPointer<vtkMassProperties>::New();
    massProperties->SetInputData(boundingDomain);

    double expectedVolume = massProperties->GetVolume();

    CHECK(calculatedVolume == doctest::Approx(expectedVolume).epsilon(0.1));
  }
}
}