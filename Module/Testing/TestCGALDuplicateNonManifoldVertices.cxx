#include <vtkCleanPolyData.h>
#include <vtkFeatureEdges.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTestUtilities.h>
#include <vtkXMLPolyDataReader.h>

#include "stkCGALDuplicateNonManifoldVertices.h"

// Use one of the following commands in Windows Powershell from SpecifX's Build folder to run this
// test: ctest -C RelWithDebInfo -R TestCGALDuplicateNonManifoldVertices CMake variable
// STK_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALDuplicateNonManifoldVertices(int argc, char** const argv)
{
  auto reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  char* inFile = vtkTestUtilities::ExpandDataFileName(argc, argv, "/non-manifold-mesh.vtp");
  reader->SetFileName(inFile);
  delete[] inFile;
  reader->Update();

  vtkPolyData* testInput = reader->GetOutput();

  if (testInput == nullptr)
  {
    std::cout << "Test Input is NULL or invalid";
    return EXIT_FAILURE;
  }

  auto checkNonManifoldEdges = vtkSmartPointer<vtkFeatureEdges>::New();
  checkNonManifoldEdges->SetInputData(testInput);
  checkNonManifoldEdges->ManifoldEdgesOff();
  checkNonManifoldEdges->FeatureEdgesOff();
  checkNonManifoldEdges->BoundaryEdgesOff();
  checkNonManifoldEdges->NonManifoldEdgesOn();
  checkNonManifoldEdges->Update();

  if (checkNonManifoldEdges->GetOutput()->GetNumberOfCells() == 0)
  {
    std::cout << "Expected Non-Manifold Edges in the Test Input";
    return EXIT_FAILURE;
  }

  auto cleanPolyDataAlgorithm = vtkSmartPointer<vtkCleanPolyData>::New();
  cleanPolyDataAlgorithm->PieceInvariantOn();
  cleanPolyDataAlgorithm->ToleranceIsAbsoluteOn();
  cleanPolyDataAlgorithm->SetAbsoluteTolerance(0.0);
  cleanPolyDataAlgorithm->ConvertLinesToPointsOff();
  cleanPolyDataAlgorithm->ConvertPolysToLinesOff();
  cleanPolyDataAlgorithm->ConvertStripsToPolysOff();
  cleanPolyDataAlgorithm->PointMergingOn();

  auto duplicateNonManifoldVertices = vtkSmartPointer<stkCGALDuplicateNonManifoldVertices>::New();
  duplicateNonManifoldVertices->SetInputData(0, reader->GetOutput());
  duplicateNonManifoldVertices->CleanOutputOn();
  duplicateNonManifoldVertices->SetCleanPolyDataAlgorithm(cleanPolyDataAlgorithm);
  duplicateNonManifoldVertices->Update();

  vtkPolyData* testOutput = duplicateNonManifoldVertices->GetOutput();

  checkNonManifoldEdges->SetInputData(testOutput);
  checkNonManifoldEdges->Update();

  if (checkNonManifoldEdges->GetOutput()->GetNumberOfCells() != 0)
  {
    std::cout << "Expected combinatorial repair Non-Manifold Edges in the Test Output";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
