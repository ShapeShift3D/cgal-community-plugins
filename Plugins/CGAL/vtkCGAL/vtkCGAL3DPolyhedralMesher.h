#pragma once

#include <stkCGAL3DPolyhedralMesherInterface.h>
#include <vtkCGALModule.h>

class VTKCGAL_EXPORT vtkCGAL3DPolyhedralMesher : public stkCGAL3DPolyhedralMesherInterface
{
public:
  static vtkCGAL3DPolyhedralMesher* New();
  vtkTypeMacro(vtkCGAL3DPolyhedralMesher, stkCGAL3DPolyhedralMesherInterface);

protected:
  vtkCGAL3DPolyhedralMesher();
  ~vtkCGAL3DPolyhedralMesher() {}

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  vtkCGAL3DPolyhedralMesher(const vtkCGAL3DPolyhedralMesher&) = delete;
  void operator=(const vtkCGAL3DPolyhedralMesher&) = delete;
};
