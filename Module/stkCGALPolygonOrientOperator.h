/**
 * @class stkCGALPolygonOrientOperator
 * @brief Orients a polygon clockwise or counter-clockwise.
 *
 * Orients a polygon by detecting the orientation of the polygon and reverses it if necessary.
 * Points and cells of the polygon will be reordered. The polygon must be flat and contained in
 * either the XY, YZ or XZ plane.
 *
 *        Inputs: Polygon (port 0, vtkPolyData)
 *        Output: Oriented Polygon (port 0, vtkPolyData)
 *
 * @sa
 * stkCGALPolygonOrientOperator
 */
#pragma once

#include <stkCGALModule.h>
#include <stkCGALPolygonOrientOperatorInterface.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALPolygonOrientOperator : public stkCGALPolygonOrientOperatorInterface
{
public:
  static stkCGALPolygonOrientOperator* New();
  vtkTypeMacro(stkCGALPolygonOrientOperator, stkCGALPolygonOrientOperatorInterface);

protected:
  stkCGALPolygonOrientOperator() = default;
  ~stkCGALPolygonOrientOperator() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALPolygonOrientOperator(const stkCGALPolygonOrientOperator&) = delete;
  void operator=(const stkCGALPolygonOrientOperator&) = delete;
};
