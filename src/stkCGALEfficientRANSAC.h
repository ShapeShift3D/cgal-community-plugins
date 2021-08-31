/**
 * @class stkCGALEfficientRANSAC
 * @brief Detects plane shaped regions within a vtkPolyData
 *
 * This filter uses the CGAL efficient RANSAC method.
 * It will append a point data array "regions" filled with the detected
 * region indices or -1 if the point is not assigned to a region.
 *
 *
 * In addition, a point data array "distances" filled with the distances to the
 * corresponding plane will be added to the output point data.
 *
 * TODO:
 *      - Wrap detect() using callbacks so the user can see the progress
 *        (see
 * https://doc.cgal.org/latest/Shape_detection/index.html#Shape_detection_RANSACExample_with_callback)
 *      - Add the choice of the kernel as a parameter (see stkCGALRegionGrowing::KernelValue)
 *
 * @warning The input dataset must contain point notmals.
 *
 * @sa
 * stkCGALEfficientRANSAC
 */

#pragma once

#include <stkCGALEfficientRANSACInterface.h>
#include <stkCGALModule.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALEfficientRANSAC : public stkCGALEfficientRANSACInterface
{
public:
  static stkCGALEfficientRANSAC* New();
  vtkTypeMacro(stkCGALEfficientRANSAC, stkCGALEfficientRANSACInterface);

protected:
  stkCGALEfficientRANSAC() = default;
  ~stkCGALEfficientRANSAC() = default;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALEfficientRANSAC(const stkCGALEfficientRANSAC&) = delete;
  void operator=(const stkCGALEfficientRANSAC&) = delete;

  template<class CGalKernel>
  int Detection(vtkPolyData* input, vtkPolyData* output);

  template<class CGalKernel, typename CGalOrientedPointVector>
  static bool vtkPolyDataToOrientedPoints(
    vtkPolyData* polyData, CGalOrientedPointVector& orientedPoints);
};
