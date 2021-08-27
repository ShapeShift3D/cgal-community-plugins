#pragma once

#include <stkCGALModule.h>
#include <stkCGALPolygonSetOrientedSideClassifierInterface.h>

// Inherit from the desired filter
class STKCGAL_EXPORT stkCGALPolygonSetOrientedSideClassifier
  : public stkCGALPolygonSetOrientedSideClassifierInterface
{
public:
  static stkCGALPolygonSetOrientedSideClassifier* New();
  vtkTypeMacro(
    stkCGALPolygonSetOrientedSideClassifier, stkCGALPolygonSetOrientedSideClassifierInterface);

protected:
  stkCGALPolygonSetOrientedSideClassifier() = default;
  ~stkCGALPolygonSetOrientedSideClassifier() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALPolygonSetOrientedSideClassifier(const stkCGALPolygonSetOrientedSideClassifier&) = delete;
  void operator=(const stkCGALPolygonSetOrientedSideClassifier&) = delete;
};
