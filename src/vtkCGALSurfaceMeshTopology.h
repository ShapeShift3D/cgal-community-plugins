#pragma once

#include <vtkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class VTKCGAL_EXPORT vtkCGALSurfaceMeshTopology : public vtkPolyDataAlgorithm
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
