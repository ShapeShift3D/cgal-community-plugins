/**
 * @class stkCGALARAPUVParametrization
 * @brief This filter flattens an open surface through an ARAP type free border parametrization.
 *
 * The ARAP UV parametrization is a free border iterative energy minimization process with
 * prioritization control over angle and/or shape preservation. The process stops when the maximum
 * number of iterations is attained or the energy variation between latest successive iterations is
 * under the defined tolerance.
 *
 *        Inputs: Mesh (port 0, vtkPolyData)
 *        Output: UV Parametrization (port 0, vtkPolyData)
 *
 * @sa
 * stkCGALARAPUVParametrization
 */
#pragma once

#include <stkCGALARAPUVParametrizationInterface.h>
#include <stkCGALModule.h>

/**
 * @ingroup stkCGAL
 *
 */
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

  //@{
  /**
   * Converts the resulting UV map to a vtkPolyData.
   */
  template<typename SurfaceMesh, typename VertexUVMap>
  bool UVMapToPolyData(SurfaceMesh& sm, VertexUVMap& uv_map, vtkPolyData* polyData);
  //@}
};
