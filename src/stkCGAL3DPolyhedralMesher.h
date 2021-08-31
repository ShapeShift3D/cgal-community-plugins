/**
 * @class stkCGAL3DPolyhedralMesher
 * @brief Generates a polyhedral mesh of the bounding domain based on features related to the
 * interior surfaces. ODT and Lloyd optimization methods are supported. Exude and perturbe options
 * are also supported, but can have an impact on the refinement criteria. See the official CGAL
 * documentation for more information.
 *
 *        Inputs: Interior Surfaces (port 0, vtkPolyData), Bounding Domain (port 1, vtkPolyData)
 *        Output: Polyhedral Domain with features (port 0, vtkUnstructuredGrid)
 *
 * @sa
 * stkCGAL3DPolyhedralMesher
 */
#pragma once

#include <stkCGAL3DPolyhedralMesherInterface.h>
#include <stkCGALModule.h>

/**
 * @defgroup stkCGAL stkCGAL
 *
 */
/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGAL3DPolyhedralMesher : public stkCGAL3DPolyhedralMesherInterface
{
public:
  static stkCGAL3DPolyhedralMesher* New();
  vtkTypeMacro(stkCGAL3DPolyhedralMesher, stkCGAL3DPolyhedralMesherInterface);

protected:
  stkCGAL3DPolyhedralMesher() = default;
  ~stkCGAL3DPolyhedralMesher() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGAL3DPolyhedralMesher(const stkCGAL3DPolyhedralMesher&) = delete;
  void operator=(const stkCGAL3DPolyhedralMesher&) = delete;
};
