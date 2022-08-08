#include "stkCGALFillHoles.h"

//---------VTK----------------------------------
#include <vtkInformation.h>
#include <vtkInformationVector.h>

//---------CGAL---------------------------------

//--------Module-----------------------------
#include "stkCGALPolygonUtilities.h"


vtkStandardNewMacro(stkCGALFillHoles);

// ----------------------------------------------------------------------------
int stkCGALFillHoles::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkPolyData* input = this->GetInputPolyLine();

  vtkPolyData* output = vtkPolyData::GetData(outputVector, 0);

  if (input == nullptr)
  {
    vtkErrorMacro("Input is empty");
    return 0;
  }

  // Convert Input PolyData to Surface Mesh

  // Fill Holes Code

  // Convert Surface Mesh to Output PolyData

  return 1;
}
