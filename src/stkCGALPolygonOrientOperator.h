#pragma once

#include <stkCGALModule.h>
#include <stkCGALPolygonOrientOperatorInterface.h>

// Inherit from the desired filter
class STKCGAL_EXPORT stkCGALPolygonOrientOperator : public stkCGALPolygonOrientOperatorInterface
{
public:
  // VTK requirements
  static stkCGALPolygonOrientOperator* New();
  vtkTypeMacro(stkCGALPolygonOrientOperator, stkCGALPolygonOrientOperatorInterface);

protected:
  stkCGALPolygonOrientOperator() = default;
  ~stkCGALPolygonOrientOperator() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALPolygonOrientOperator(const stkCGALPolygonOrientOperator&) = delete;
  void operator=(const stkCGALPolygonOrientOperator&) = delete;
};
