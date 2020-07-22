#ifndef vtkCGALSurfaceMeshSegmentation_h
#define vtkCGALSurfaceMeshSegmentation_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <vtkCGALModule.h>

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALSurfaceMeshSegmentation : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGALSurfaceMeshSegmentation* New();
  vtkTypeMacro(vtkCGALSurfaceMeshSegmentation, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputMesh();
 
  int FillOutputPortInformation(int, vtkInformation *info) override;

  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;

  vtkSetClampMacro(SmoothingLambda, double, 0.0, 1.0);
  vtkGetMacro(SmoothingLambda, double);

  vtkSetMacro(NumberOfClusters, int);
  vtkGetMacro(NumberOfClusters, int);

protected:
  vtkCGALSurfaceMeshSegmentation();
  ~vtkCGALSurfaceMeshSegmentation(){}
  
  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  int NumberOfClusters;
  double SmoothingLambda; // [0,1]

  vtkCGALSurfaceMeshSegmentation(const vtkCGALSurfaceMeshSegmentation&) = delete;
  void operator=(const vtkCGALSurfaceMeshSegmentation&) = delete;
};
#endif


