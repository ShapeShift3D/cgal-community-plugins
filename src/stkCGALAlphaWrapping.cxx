#include "stkCGALAlphaWrapping.h"

//---------VTK----------------------------------
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPointSet.h>
#include <vtkPolyData.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>

//---------Module-------------------------------
#include <stkCGALUtilities.h>

#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_Mesh;

vtkStandardNewMacro(stkCGALAlphaWrapping);

// ----------------------------------------------------------------------------
int stkCGALAlphaWrapping::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** vtkNotUsed(inputVector), vtkInformationVector* outputVector)
{
  vtkPointSet* inputPointSet = this->GetInputPointSet();

  vtkPolyData* outputMesh = vtkPolyData::GetData(outputVector, 0);

  if (inputPointSet == nullptr || inputPointSet->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("There are no points in the input");
    return 0;
  }

  std::vector<Point_3> points;

  for (vtkIdType pointID = 0; pointID < inputPointSet->GetNumberOfPoints(); pointID++)
  {
    auto vtkPoint = inputPointSet->GetPoint(pointID);
    points.emplace_back(vtkPoint[0], vtkPoint[1], vtkPoint[2]);
  }

  if (this->Alpha <= 0.0)
  {
    vtkErrorMacro("Alpha value must be strictly positive");
    return 0;
  }

  if (this->Offset <= 0.0)
  {
    vtkErrorMacro("Offset value must be strictly positive");
    return 0;
  }

  Surface_Mesh wrap;

  try
  {
    CGAL::alpha_wrap_3(points, this->Alpha, this->Offset, wrap);
  }
  catch(...)
  {
    vtkErrorMacro("Failed to create Alpha Wrapping");
    return 0;
  }
  
  stkCGALUtilities::SurfaceMeshToPolyData(wrap, outputMesh);

  return 1;
}
