/**
 * @class stkCGALDuplicateNonManifoldVertices
 * @brief
 *
 * Create a combinatorially manifold surface mesh by splitting any non-manifold vertex into as
 * many vertices as there are manifold sheets at this geometric position. Note however that the mesh
 * will still not be manifold from a geometric point of view, as the positions of the new vertices
 * introduced at a non-manifold vertex are identical to the input non-manifold vertex.
 *
 *  Inputs: SurfaceMesh (port 0, vtkPolyData)
 *  Output: SurfaceMesh (port 0, vtkPolyData)
 *
 * @sa
 * stkCGALDuplicateNonManifoldVerticesInterface
 */
#pragma once

#include <stkCGALDuplicateNonManifoldVerticesInterface.h>
#include <stkCGALModule.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALDuplicateNonManifoldVertices
  : public stkCGALDuplicateNonManifoldVerticesInterface
{
public:
  static stkCGALDuplicateNonManifoldVertices* New();
  vtkTypeMacro(stkCGALDuplicateNonManifoldVertices, stkCGALDuplicateNonManifoldVerticesInterface);

protected:
  stkCGALDuplicateNonManifoldVertices() = default;
  ~stkCGALDuplicateNonManifoldVertices() = default;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALDuplicateNonManifoldVertices(const stkCGALDuplicateNonManifoldVertices&) = delete;
  void operator=(const stkCGALDuplicateNonManifoldVertices&) = delete;
};