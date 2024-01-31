/**
 * @class stkCGALRegionGrowing
 * @brief Detects plane shaped regions within a vtkPolyData
 *
 * This filter uses the CGAL region growing method.
 * It will append a cell data array "regions" filled with the detected
 * region indices or -1 if the cell is not assigned to a region.
 *
 * @sa
 * stkCGALRegionGrowing
 */
#pragma once

#include <stkCGALModule.h>
#include <stkCGALRegionGrowingInterface.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALRegionGrowing : public stkCGALRegionGrowingInterface
{
public:
  static stkCGALRegionGrowing* New();
  vtkTypeMacro(stkCGALRegionGrowing, stkCGALRegionGrowingInterface);

protected:
  stkCGALRegionGrowing() = default;
  ~stkCGALRegionGrowing() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALRegionGrowing(const stkCGALRegionGrowing&) = delete;
  void operator=(const stkCGALRegionGrowing&) = delete;

  template<class CGalKernel>
  int Detection(vtkPolyData*, vtkPolyData*);

  template<typename MeshType>
  static bool vtkPolyDataToPolygonMesh(vtkPolyData*, MeshType&);
};
