#pragma once

#include <stkCGALModule.h>
#include <stkCGALSurfaceMeshTopologyInterface.h>

class STKCGAL_EXPORT stkCGALSurfaceMeshTopology : public stkCGALSurfaceMeshTopologyInterface
{
public:
  static stkCGALSurfaceMeshTopology* New();
  vtkTypeMacro(stkCGALSurfaceMeshTopology, stkCGALSurfaceMeshTopologyInterface);

protected:
  stkCGALSurfaceMeshTopology() = default;
  ~stkCGALSurfaceMeshTopology() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALSurfaceMeshTopology(const stkCGALSurfaceMeshTopology&) = delete;
  void operator=(const stkCGALSurfaceMeshTopology&) = delete;
};
