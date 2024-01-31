/**
 * @class stkCGALPolygonMeshSmoothingOperator
 * @brief Smooths a triangulated region of a polydata by attempting to make the triangle angle and
 * area distributions as uniform as possible by moving non-constrained vertices.
 *
 * Constraints are determined by the chosen dihedral angle of the edges.
 *
 * Angle-based smoothing does not change the combinatorial information of the mesh. Area-based
 * smoothing might change the combinatorial information, unless specified otherwise. It is also
 * possible to make the smoothing algorithm "safer" by rejecting moves that, when applied, would
 * worsen the quality of the mesh, e.g. that would decrease the value of the smallest angle around a
 * vertex or create self-intersections.
 *
 * To use the area-based smoothing, Ceres needs to be binded to the CGAL library. This implies the
 * need to also bind LAPACK and BLAS.
 *
 * @sa
 * stkCGALPolygonMeshSmoothingOperator
 */
#pragma once

#include <stkCGALModule.h>
#include <stkCGALPolygonMeshSmoothingOperatorInterface.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALPolygonMeshSmoothingOperator
  : public stkCGALPolygonMeshSmoothingOperatorInterface
{
public:
  static stkCGALPolygonMeshSmoothingOperator* New();
  vtkTypeMacro(stkCGALPolygonMeshSmoothingOperator, stkCGALPolygonMeshSmoothingOperatorInterface);

protected:
  stkCGALPolygonMeshSmoothingOperator() = default;
  ~stkCGALPolygonMeshSmoothingOperator() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALPolygonMeshSmoothingOperator(const stkCGALPolygonMeshSmoothingOperator&) = delete;
  void operator=(const stkCGALPolygonMeshSmoothingOperator&) = delete;
};
