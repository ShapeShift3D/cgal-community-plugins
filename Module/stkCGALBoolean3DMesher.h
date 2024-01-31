/**
 * @class stkCGALBoolean3DMesher
 * @brief Applies a boolean operation between two 3D surfaces.
 *
 * This filter takes two inputs, inputMeshA and inputMeshB, of type vtkPolyData and applies
 * one of the four boolean operations (Union, Intersection, Difference1 (A - B) and Difference2 (B -
 * A)) between them.
 *
 * Inputs: Surface Mesh A (port 0, vtkPolyData), Surface Mesh B (port 1, vtkPolyData)
 * Output: Surface Mesh Result (port 0, vtkPolyData)
 *
 * @sa
 * stkCGALBoolean3DMesher
 */
#pragma once

#include <stkCGALBoolean3DMesherInterface.h>
#include <stkCGALModule.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALBoolean3DMesher : public stkCGALBoolean3DMesherInterface
{
public:
  static stkCGALBoolean3DMesher* New();
  vtkTypeMacro(stkCGALBoolean3DMesher, stkCGALBoolean3DMesherInterface);

protected:
  stkCGALBoolean3DMesher() = default;
  ~stkCGALBoolean3DMesher() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALBoolean3DMesher(const stkCGALBoolean3DMesher&) = delete;
  void operator=(const stkCGALBoolean3DMesher&) = delete;

  template<typename SurfaceMesh>
  SurfaceMesh* RunBooleanOperations(SurfaceMesh& tm1, SurfaceMesh& tm2, SurfaceMesh& operation);
};
