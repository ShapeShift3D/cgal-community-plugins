#pragma once

#include <stkCGALLSCMUVParametrizationInterface.h>
#include <stkCGALModule.h>

class STKCGAL_EXPORT stkCGALLSCMUVParametrization : public stkCGALLSCMUVParametrizationInterface
{
public:
  static stkCGALLSCMUVParametrization* New();
  vtkTypeMacro(stkCGALLSCMUVParametrization, stkCGALLSCMUVParametrizationInterface);

protected:
  stkCGALLSCMUVParametrization() = default;
  ~stkCGALLSCMUVParametrization() = default;

  virtual int RequestData(vtkInformation* request, vtkInformationVector** inputVector,
    vtkInformationVector* outputVector) override;

private:
  stkCGALLSCMUVParametrization(const stkCGALLSCMUVParametrization&) = delete;
  void operator=(const stkCGALLSCMUVParametrization&) = delete;

  template<typename SurfaceMesh, typename VertexUVMap>
  bool UpdatePointCoordinates(
    SurfaceMesh& sm, VertexUVMap& uv_map, vtkPolyData* polyData);
};
