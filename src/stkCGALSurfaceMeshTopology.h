#pragma once

#include <stkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class STKCGAL_EXPORT stkCGALSurfaceMeshTopology : public vtkPolyDataAlgorithm
{
public:
  static stkCGALSurfaceMeshTopology* New();
  vtkTypeMacro(stkCGALSurfaceMeshTopology, vtkPolyDataAlgorithm);

protected:
  stkCGALSurfaceMeshTopology();
  ~stkCGALSurfaceMeshTopology() {}

  virtual int RequestData(
    vtkInformation *,
    vtkInformationVector **,
    vtkInformationVector *) override;

private:
  stkCGALSurfaceMeshTopology(const stkCGALSurfaceMeshTopology&) = delete;
  void operator=(const stkCGALSurfaceMeshTopology&) = delete;
};
