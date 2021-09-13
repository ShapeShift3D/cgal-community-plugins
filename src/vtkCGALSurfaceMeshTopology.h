#pragma once

#include <stkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class STKCGAL_EXPORT vtkCGALSurfaceMeshTopology : public vtkPolyDataAlgorithm
{
public:
  static vtkCGALSurfaceMeshTopology* New();
  vtkTypeMacro(vtkCGALSurfaceMeshTopology, vtkPolyDataAlgorithm);

protected:
  vtkCGALSurfaceMeshTopology();
  ~vtkCGALSurfaceMeshTopology() {}

  virtual int RequestData(
    vtkInformation *,
    vtkInformationVector **,
    vtkInformationVector *) override;

private:
  vtkCGALSurfaceMeshTopology(const vtkCGALSurfaceMeshTopology&) = delete;
  void operator=(const vtkCGALSurfaceMeshTopology&) = delete;
};
