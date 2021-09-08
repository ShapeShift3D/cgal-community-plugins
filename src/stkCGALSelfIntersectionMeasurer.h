/**
 * @class stkCGALSelfIntersectionMeasurer
 * @brief This filter evaluates self-intersections inside a PolyData. PolyData made of two
 * non-connected surfaces that intersect each other are counted as self-intersections.
 *
 * Inputs: inputMesh (port 0, vtkPolyData)
 * Output: output (port 0, vtkPolyData)
 *
 * @sa
 * stkCGALSelfIntersectionMeasurer
 */
#pragma once

#include <stkCGALModule.h>
#include <stkCGALSelfIntersectionMeasurerInterface.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALSelfIntersectionMeasurer
  : public stkCGALSelfIntersectionMeasurerInterface
{
public:
  static stkCGALSelfIntersectionMeasurer* New();
  vtkTypeMacro(stkCGALSelfIntersectionMeasurer, stkCGALSelfIntersectionMeasurerInterface);

protected:
  stkCGALSelfIntersectionMeasurer() = default;
  ~stkCGALSelfIntersectionMeasurer() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALSelfIntersectionMeasurer(const stkCGALSelfIntersectionMeasurer&) = delete;
  void operator=(const stkCGALSelfIntersectionMeasurer&) = delete;

  int ExecuteSelfIntersect(vtkPolyData* polyDataIn, vtkPolyData* polyDataOut);
};
