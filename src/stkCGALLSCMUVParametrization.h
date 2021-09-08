/**
 * @class stkCGALLSCMUVParametrization
 * @brief This filter flattens an open surface through a LSCM type free border parametrization.
 *
 * The LSCM UV parametrization is a free border iterative energy minimization process with angle
 * preservation. The process stops when the maximum number of iterations is attained or the energy
 * variation between latest successive iterations is under the defined tolerance.
 *
 *        Inputs: Mesh (port 0, vtkPolyData)
 *        Output: UV Parametrization (port 0, vtkPolyData)
 *
 * @sa
 * stkCGALLSCMUVParametrization
 */
#pragma once

#include <stkCGALLSCMUVParametrizationInterface.h>
#include <stkCGALModule.h>

/**
 * @ingroup stkCGAL
 *
 */
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
  bool UpdatePointCoordinates(SurfaceMesh& sm, VertexUVMap& uv_map, vtkPolyData* polyData);
};
