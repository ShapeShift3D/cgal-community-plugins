#pragma once

#include <stkCGALARAPUVParametrizationInterface.h>
#include <stkCGALModule.h>

class STKCGAL_EXPORT stkCGALARAPUVParametrization : public stkCGALARAPUVParametrizationInterface
{
public:
  static stkCGALARAPUVParametrization* New();
  vtkTypeMacro(stkCGALARAPUVParametrization, stkCGALARAPUVParametrizationInterface);

protected:
  stkCGALARAPUVParametrization() = default;
  ~stkCGALARAPUVParametrization() = default;

  virtual int RequestData(vtkInformation* request, vtkInformationVector** inputVector,
    vtkInformationVector* outputVector) override;

private:
  stkCGALARAPUVParametrization(const stkCGALARAPUVParametrization&) = delete;
  void operator=(const stkCGALARAPUVParametrization&) = delete;

  template<typename SurfaceMesh, typename VertexUVMap>
  bool UVMapToPolyData(SurfaceMesh& sm, VertexUVMap& uv_map, vtkPolyData* polyData);
};
