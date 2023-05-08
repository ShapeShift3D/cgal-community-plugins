#include <vtkCellData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTestUtilities.h>
#include <vtkXMLPolyDataReader.h>

#include <string>

#include "stkCGALSelfIntersectionMeasurer.h"

// Use one of the following commands in Windows Powershell from SpecifX's Build folder to run this
// test: ctest -C RelWithDebInfo -R TestCGALSelfIntersectionMeasurer CMake variable
// PARAVIEW_BUILD_TESTING must be set to ON to be able to run this test.

int TestCGALSelfIntersectionMeasurer(int argc, char** const argv)
{
  auto reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  char* inFile = vtkTestUtilities::ExpandDataFileName(
    argc, argv, "STKCopyLeft/CGAL/src/Testing/Data/self-intersectionsData.vtp");
  reader->SetFileName(inFile);
  delete[] inFile;
  reader->Update();

  std::string selfIntersectionArrayName = "InsersectingPairs";

  auto selfIntersectionFilter = vtkSmartPointer<stkCGALSelfIntersectionMeasurer>::New();
  selfIntersectionFilter->SetInputData(0, reader->GetOutput());
  selfIntersectionFilter->IterateByConnectivityOn();
  selfIntersectionFilter->SetSelfIntersectionsArrayName(selfIntersectionArrayName);
  selfIntersectionFilter->VerboseOff();
  selfIntersectionFilter->PrintSelfIntersectingPairsOff();
  selfIntersectionFilter->OutputNullFaceMaskArrayOff();
  selfIntersectionFilter->RepairSelfIntersectionsOff();
  selfIntersectionFilter->Update();

  auto intersectionPairArray =
    selfIntersectionFilter->GetOutput()->GetCellData()->GetArray(selfIntersectionArrayName.c_str());

  if (intersectionPairArray == nullptr)
  {
    std::cout << "Self Intersection Output Data Array is NULL";
    return EXIT_FAILURE;
  }

  // Test Case , Check Detection of self-Intersections
  if (intersectionPairArray->GetRange()[1] == 0)
  {
    std::cout << "Expected Measurement of some self-intersection pairs";
    return EXIT_FAILURE;
  }

  selfIntersectionFilter->RepairSelfIntersectionsOn();
  selfIntersectionFilter->Update();

  intersectionPairArray =
    selfIntersectionFilter->GetOutput()->GetCellData()->GetArray(selfIntersectionArrayName.c_str());

  // Test Case , Check Repair of self-Intersections
  if (intersectionPairArray->GetRange()[1] != 0)
  {
    std::cout << "Expected Complete Repair of the expected self-intersection pairs";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
