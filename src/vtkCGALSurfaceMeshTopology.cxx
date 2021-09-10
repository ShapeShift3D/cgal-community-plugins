#include "vtkCGALSurfaceMeshTopology.h"

vtkStandardNewMacro(vtkCGALSurfaceMeshTopology);

//----------------------------------------------------------------------------
vtkCGALSurfaceMeshTopology::vtkCGALSurfaceMeshTopology()
{
    this->SetNumberOfInputPorts(1);
    //this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
int vtkCGALSurfaceMeshTopology::RequestData(vtkInformation* vtkNotUsed(request),
                                            vtkInformationVector** inputVector,
                                            vtkInformationVector* outputVector)
{
    return 0;
}
