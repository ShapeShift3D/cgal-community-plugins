#ifndef vtkCGAL2DBoolean_h
#define vtkCGAL2DBoolean_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <vtkCGALModule.h>

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGAL2DBoolean : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGAL2DBoolean* New();
  vtkTypeMacro(vtkCGAL2DBoolean, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputMeshA();
  vtkPolyData* GetInputMeshB();
 
  int FillOutputPortInformation(int, vtkInformation *info) override;

  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;

  Surface_Mesh* RunBooleanOperations(
	  Surface_Mesh& tm1,
	  Surface_Mesh& tm2,
	  Surface_Mesh& operation
	  );

  vtkGetMacro(Mode, int);
  vtkSetMacro(Mode, int);

protected:
  vtkCGAL2DBoolean();
  ~vtkCGAL2DBoolean(){}
  int Mode;
  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*)override;

private:
  // needed but not implemented
  vtkCGAL2DBoolean(const vtkCGAL2DBoolean&) = delete;
  void operator=(const vtkCGAL2DBoolean&) = delete;
};
#endif
