/**
 * @class stkCGALFillHoles
 * @brief
 *
 *  Fill the Holes of the Input Surface Mesh with the selected filling type.
 *
 *  Inputs: SurfaceMesh (port 0, vtkPolyData)
 *  Output: SurfaceMesh (port 0, vtkPolyData)
 *
 * @sa
 * stkCGALFillHolesInterface
 */
#pragma once

#include <stkCGALFillHolesInterface.h>
#include <stkCGALModule.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALFillHoles : public stkCGALFillHolesInterface
{
public:
  static stkCGALFillHoles* New();
  vtkTypeMacro(stkCGALFillHoles, stkCGALFillHolesInterface);

protected:
  stkCGALFillHoles() = default;
  ~stkCGALFillHoles() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALFillHoles(const stkCGALFillHoles&) = delete;
  void operator=(const stkCGALFillHoles&) = delete;
};