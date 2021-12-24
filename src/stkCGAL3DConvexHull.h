/**
 * @class stkCGAL3DConvexHull
 * @brief Generate 3D Convex Hull from a Point Set Data.
 * 
 * A subset S⊆R3 is convex if for any two points p and q in the set the line segment with endpoints p and q is contained in S.
 * The convex hull of a set S is the smallest convex set containing S.
 *
 * Inputs: inputMesh (port 0, vtkPointSet)
 * Output: output (port 0, vtkPolyData)
 *
 * @sa
 * stkCGAL3DConvexHull
 */
#pragma once

#include <stkCGALModule.h>
#include <stkCGAL3DConvexHullInterface.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGAL3DConvexHull
  : public stkCGAL3DConvexHullInterface
{
public:
  static stkCGAL3DConvexHull* New();
  vtkTypeMacro(stkCGAL3DConvexHull, stkCGAL3DConvexHullInterface);

protected:
  stkCGAL3DConvexHull() = default;
  ~stkCGAL3DConvexHull() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGAL3DConvexHull(const stkCGAL3DConvexHull&) = delete;
  void operator=(const stkCGAL3DConvexHull&) = delete;

};
