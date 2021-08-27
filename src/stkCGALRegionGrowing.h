/**
 * @class stkCGALRegionGrowing
 * @brief Detects plane shaped regions within a vtkPolyData
 *
 * This filter uses the CGAL region growing method.
 * It will append a cell data array "regions" filled with the detected
 * region indices or -1 if the cell is not assigned to a region.
 *
 * TODO:
 *      - Add extra kernels if applicable (Homogeneous and Simple_homogeneous)
 *
 * @sa
 * stkCGALEfficientRANSAC
 */

#pragma once

#include <stkCGALModule.h>
#include <stkCGALRegionGrowingInterface.h>

class STKCGAL_EXPORT stkCGALRegionGrowing : public stkCGALRegionGrowingInterface
{
public:
  static stkCGALRegionGrowing* New();
  vtkTypeMacro(stkCGALRegionGrowing, stkCGALRegionGrowingInterface);

protected:
  stkCGALRegionGrowing() = default;
  ~stkCGALRegionGrowing() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  template<class CGalKernel>
  int Detection(vtkPolyData*, vtkPolyData*);

  template<typename MeshType>
  static bool vtkPolyDataToPolygonMesh(vtkPolyData*, MeshType&);

private:
  stkCGALRegionGrowing(const stkCGALRegionGrowing&) = delete;
  void operator=(const stkCGALRegionGrowing&) = delete;
};
