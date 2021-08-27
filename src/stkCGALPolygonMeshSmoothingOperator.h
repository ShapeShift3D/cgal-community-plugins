#pragma once

#include <stkCGALPolygonMeshSmoothingOperatorInterface.h>
#include <stkCGALModule.h>

// Inherit from the desired filter
class STKCGAL_EXPORT stkCGALPolygonMeshSmoothingOperator : public stkCGALPolygonMeshSmoothingOperatorInterface
{
public:
  static stkCGALPolygonMeshSmoothingOperator* New();
  vtkTypeMacro(stkCGALPolygonMeshSmoothingOperator, stkCGALPolygonMeshSmoothingOperatorInterface);

protected:
  stkCGALPolygonMeshSmoothingOperator() = default;
  ~stkCGALPolygonMeshSmoothingOperator() = default;

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

private:
  // needed but not implemented
  stkCGALPolygonMeshSmoothingOperator(const stkCGALPolygonMeshSmoothingOperator&) = delete;
  void operator=(const stkCGALPolygonMeshSmoothingOperator&) = delete;
};
