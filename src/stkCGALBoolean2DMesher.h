/**
 * @class stkCGALBoolean2DMesher
 * @brief Applies a boolean operation between two flattened 2D polygons.
 *
 *	Conditions for valid polygons:
 *
 *	Closed Boundary - the polygon's outer boundary must be a connected sequence of
 *  curves, that start and end at the same vertex. Simplicity - the polygon must be simple.
 *	Orientation - the polygon's outer boundary must be counter-clockwise oriented.
 *
 *  Inputs: Polygon A (port 0, vtkPolyData), Polygon B (port 1, vtkPolyData)
 *  Output: Polygon Result (port 0, vtkPolyData)
 *
 * @sa
 * stkCGALBoolean2DMesher
 */
#pragma once

#include <stkCGALBoolean2DMesherInterface.h>
#include <stkCGALModule.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALBoolean2DMesher : public stkCGALBoolean2DMesherInterface
{
public:
  static stkCGALBoolean2DMesher* New();
  vtkTypeMacro(stkCGALBoolean2DMesher, stkCGALBoolean2DMesherInterface);

protected:
  stkCGALBoolean2DMesher() = default;
  ~stkCGALBoolean2DMesher() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALBoolean2DMesher(const stkCGALBoolean2DMesher&) = delete;
  void operator=(const stkCGALBoolean2DMesher&) = delete;
};
