#pragma once

#include <stkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

#include <string.h>

class STKCGAL_EXPORT stkCGALSurfaceMeshTopology : public vtkPolyDataAlgorithm
{
public:
  static stkCGALSurfaceMeshTopology* New();
  vtkTypeMacro(stkCGALSurfaceMeshTopology, vtkPolyDataAlgorithm);

  //@{
  /**
   * 
   */
  vtkGetMacro(VertexToCheckPointMaskName, std::string);
  vtkSetMacro(VertexToCheckPointMaskName, std::string);
  //@}

  //@{
  /**
   * 
   */
  vtkGetMacro(SquaredContraintSearchTolerance, double);
  vtkSetMacro(SquaredContraintSearchTolerance, double);
  //@}

protected:
  stkCGALSurfaceMeshTopology();
  ~stkCGALSurfaceMeshTopology() {}

  virtual int RequestData(
    vtkInformation *,
    vtkInformationVector **,
    vtkInformationVector *) override;

private:
  stkCGALSurfaceMeshTopology(const stkCGALSurfaceMeshTopology&) = delete;
  void operator=(const stkCGALSurfaceMeshTopology&) = delete;

  std::string VertexToCheckPointMaskName;

  double SquaredContraintSearchTolerance;
};
