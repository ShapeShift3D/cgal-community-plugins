#pragma once

#include "vtkPolyDataAlgorithm.h" 
#include <vtkCGALModule.h>

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALPolygonMeshSmoothingOperator : public vtkPolyDataAlgorithm
{
public:
  static vtkCGALPolygonMeshSmoothingOperator* New();
  vtkTypeMacro(vtkCGALPolygonMeshSmoothingOperator, vtkPolyDataAlgorithm);

  //@{
  /**
  * If true, angle-based smoothing will be used.
  */
  vtkGetMacro(UseAngleSmoothing, bool);
  vtkSetMacro(UseAngleSmoothing, bool);
  vtkBooleanMacro(UseAngleSmoothing, bool);
  //@}

  //@{
  /**
  * If true, area-based smoothing will be used. Requires linking to Ceres library to work.
  */
  vtkGetMacro(UseAreaSmoothing, bool);
  vtkSetMacro(UseAreaSmoothing, bool);
  vtkBooleanMacro(UseAreaSmoothing, bool);
  //@}

  //@{
  /**
  * Number of smoothing iterations
  */
  vtkGetMacro(NumberOfIterations, int);
  vtkSetMacro(NumberOfIterations, int);
  //@}

  //@{
  /**
  * Dihedral Angle under which satisfying edges will be constrained.
  */
  vtkGetMacro(DihedralAngleEdgeConstraint, double);
  vtkSetMacro(DihedralAngleEdgeConstraint, double);
  //@}

  //@{
  /**
  * If true, vertex moves that would worsen the mesh are ignored.
  */
  vtkGetMacro(UseSafetyConstraints, bool);
  vtkSetMacro(UseSafetyConstraints, bool);
  vtkBooleanMacro(UseSafetyConstraints, bool);
  //@}

  //@{
  /**
  * If true, points are projected onto the initial surface after each iteration.
  */
  vtkGetMacro(ProjectPointsOntoSurface, bool);
  vtkSetMacro(ProjectPointsOntoSurface, bool);
  vtkBooleanMacro(ProjectPointsOntoSurface, bool);
  //@}

protected:
  vtkCGALPolygonMeshSmoothingOperator();
  ~vtkCGALPolygonMeshSmoothingOperator(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;


  bool UseAngleSmoothing;
  bool UseAreaSmoothing;
  int NumberOfIterations;
  double DihedralAngleEdgeConstraint;
  bool UseSafetyConstraints;
  bool ProjectPointsOntoSurface;

private:
  // needed but not implemented
  vtkCGALPolygonMeshSmoothingOperator(const vtkCGALPolygonMeshSmoothingOperator&) = delete;
  void operator=(const vtkCGALPolygonMeshSmoothingOperator&) = delete;
};
