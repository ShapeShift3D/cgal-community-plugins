#pragma once

#include <stkCGALBoolean3DMesherInterface.h>
#include <stkCGALModule.h>

// Inherit from the desired filter
class STKCGAL_EXPORT stkCGALBoolean3DMesher : public stkCGALBoolean3DMesherInterface
{
public:
  // VTK requirements
  static stkCGALBoolean3DMesher* New();
  vtkTypeMacro(stkCGALBoolean3DMesher, stkCGALBoolean3DMesherInterface);

protected:
  stkCGALBoolean3DMesher() = default;
  ~stkCGALBoolean3DMesher() = default;

  int Mode;
  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  bool SkipPreconditions;
  bool ComputeSurfaceIntersection;

private:
  stkCGALBoolean3DMesher(const stkCGALBoolean3DMesher&) = delete;
  void operator=(const stkCGALBoolean3DMesher&) = delete;

  template<typename SurfaceMesh>
  SurfaceMesh* RunBooleanOperations(
	  SurfaceMesh& tm1,
	  SurfaceMesh& tm2,
	  SurfaceMesh& operation);
};
