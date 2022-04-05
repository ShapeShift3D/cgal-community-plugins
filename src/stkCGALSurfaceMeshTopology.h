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

  //@{
  /**
   * 
   */
  vtkSetMacro(ExtractSimpleCycles, bool);
  vtkGetMacro(ExtractSimpleCycles, bool);
  vtkBooleanMacro(ExtractSimpleCycles, bool);
  //@}

  //@{
  /**
   * 
   */
  vtkSetMacro(GenerateCycleIDs, bool);
  vtkGetMacro(GenerateCycleIDs, bool);
  vtkBooleanMacro(GenerateCycleIDs, bool);
  //@}

  //@{
  /**
   * 
   */
  vtkGetMacro(CycleIDArrayName, std::string);
  vtkSetMacro(CycleIDArrayName, std::string);
  //@}

  //@{
  /**
   * 
   */
  vtkSetMacro(CalculateCycleLength, bool);
  vtkGetMacro(CalculateCycleLength, bool);
  vtkBooleanMacro(CalculateCycleLength, bool);
  //@}

  //@{
  /**
   * 
   */
  vtkGetMacro(CycleLengthArrayName, std::string);
  vtkSetMacro(CycleLengthArrayName, std::string);
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

  bool ExtractSimpleCycles;

  bool GenerateCycleIDs;
  std::string CycleIDArrayName;

  bool CalculateCycleLength;
  std::string CycleLengthArrayName;
};
