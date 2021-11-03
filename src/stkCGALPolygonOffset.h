/**
 * @class stkCGALPolygonOffset
 * @brief Offset the edges of a simple 2D polygon (closed curve) to interior or exterior regions
 * formed by the polygon.The polygon must belong to XY plane and input must be a closed curve.
 *
 *  Inputs: Polygon (port 0, vtkPolyData)
 *  Output: Offset Polygon (port 0, vtkPolyData)
 *
 * @sa
 * stkCGALPolygonOffsetInterface
 */
#pragma once

#include <stkCGALModule.h>
#include <stkCGALPolygonOffsetInterface.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALPolygonOffset : public stkCGALPolygonOffsetInterface
{
public:
  static stkCGALPolygonOffset* New();
  vtkTypeMacro(stkCGALPolygonOffset, stkCGALPolygonOffsetInterface);

protected:
  stkCGALPolygonOffset() = default;
  ~stkCGALPolygonOffset() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALPolygonOffset(const stkCGALPolygonOffset&) = delete;
  void operator=(const stkCGALPolygonOffset&) = delete;
};