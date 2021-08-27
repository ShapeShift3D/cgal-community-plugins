#pragma once

#include <stkCGAL3DPolyhedralMesherInterface.h>
#include <stkCGALModule.h>

class STKCGAL_EXPORT stkCGAL3DPolyhedralMesher : public stkCGAL3DPolyhedralMesherInterface
{
public:
  static stkCGAL3DPolyhedralMesher* New();
  vtkTypeMacro(stkCGAL3DPolyhedralMesher, stkCGAL3DPolyhedralMesherInterface);

protected:
  stkCGAL3DPolyhedralMesher() = default;
  ~stkCGAL3DPolyhedralMesher() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGAL3DPolyhedralMesher(const stkCGAL3DPolyhedralMesher&) = delete;
  void operator=(const stkCGAL3DPolyhedralMesher&) = delete;
};
