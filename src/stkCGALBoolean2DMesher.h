#pragma once

#include <stkCGALBoolean2DMesherInterface.h>
#include <stkCGALModule.h>

// Inherit from the desired filter
class STKCGAL_EXPORT stkCGALBoolean2DMesher : public stkCGALBoolean2DMesherInterface
{
public:
  // VTK requirements
  static stkCGALBoolean2DMesher* New();
  vtkTypeMacro(stkCGALBoolean2DMesher, stkCGALBoolean2DMesherInterface);

protected:
  stkCGALBoolean2DMesher() = default;
  ~stkCGALBoolean2DMesher() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  // needed but not implemented
  stkCGALBoolean2DMesher(const stkCGALBoolean2DMesher&) = delete;
  void operator=(const stkCGALBoolean2DMesher&) = delete;
};
