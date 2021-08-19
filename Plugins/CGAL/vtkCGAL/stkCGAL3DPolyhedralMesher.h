#pragma once

#include <stkCGAL3DPolyhedralMesherInterface.h>
#include <vtkCGALModule.h>

class VTKCGAL_EXPORT stkCGAL3DPolyhedralMesher : public stkCGAL3DPolyhedralMesherInterface
{
public:
  static stkCGAL3DPolyhedralMesher* New();
  vtkTypeMacro(stkCGAL3DPolyhedralMesher, stkCGAL3DPolyhedralMesherInterface);

protected:
  stkCGAL3DPolyhedralMesher();
  ~stkCGAL3DPolyhedralMesher() {}

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGAL3DPolyhedralMesher(const stkCGAL3DPolyhedralMesher&) = delete;
  void operator=(const stkCGAL3DPolyhedralMesher&) = delete;
};
