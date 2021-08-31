/**
 * @class stkCGALPolygonSetOrientedSideClassifier
 * @brief Identifies the orientation of every polygon of a polygon set from a reference point.
 *
 * For every polygon of a polygon set, the classifier will evaluate if the reference point is inside
 * or not. If the point is inside, a value of 1 is attributed. Otherwise, it will have a value of
 * -1.
 *
 *        Inputs: Reference Point (port 0, vtkPolyData), Polygon Set (port 1, vtkPolyData)
 *        Output: Classified Polygon Set (port 0, vtkPolyData)
 *
 * @sa
 * stkCGALPolygonSetOrientedSideClassifier
 */
#pragma once

#include <stkCGALModule.h>
#include <stkCGALPolygonSetOrientedSideClassifierInterface.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALPolygonSetOrientedSideClassifier
  : public stkCGALPolygonSetOrientedSideClassifierInterface
{
public:
  static stkCGALPolygonSetOrientedSideClassifier* New();
  vtkTypeMacro(
    stkCGALPolygonSetOrientedSideClassifier, stkCGALPolygonSetOrientedSideClassifierInterface);

protected:
  stkCGALPolygonSetOrientedSideClassifier() = default;
  ~stkCGALPolygonSetOrientedSideClassifier() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALPolygonSetOrientedSideClassifier(const stkCGALPolygonSetOrientedSideClassifier&) = delete;
  void operator=(const stkCGALPolygonSetOrientedSideClassifier&) = delete;
};
