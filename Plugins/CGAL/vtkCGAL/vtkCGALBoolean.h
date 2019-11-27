#ifndef vtkCGALBoolean_h
#define vtkCGALBoolean_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <vtkCGALModule.h>

class vtkPolyData;
class vtkUnstructuredGrid;


// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALBoolean : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGALBoolean* New();
  vtkTypeMacro(vtkCGALBoolean, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputMeshA();
  vtkPolyData* GetInputMeshB();
 
  int FillOutputPortInformation(int, vtkInformation *info)override;

  template<typename PM>
  vtkUnstructuredGrid* polygon_mesh_to_vtkUnstructured(const PM& pmesh,//PolygonMesh
	  vtkUnstructuredGrid* usg);

  template <typename TM>
  bool vtkPointSet_to_polygon_mesh(vtkPointSet* poly_data,
	  TM& tmesh);

  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Surface_mesh<K::Point_3> Surface_Mesh;

  Surface_Mesh* run_boolean_operations(
	  Surface_Mesh& tm1,
	  Surface_Mesh& tm2,
	  Surface_Mesh& operation,
	  int Mode
	  );

  enum Modes { UNION, INTERSECTION, TM1_MINUS_TM2, TM2_MINUS_TM1};

  vtkGetMacro(Mode, int);
  vtkSetMacro(Mode, int);

  void SetModeToUnion();
  void SetModeToIntersection();
  void SetModeToDifference();
  void SetModeToDifference2();

protected:
  vtkCGALBoolean();
  ~vtkCGALBoolean(){}
  int Mode;
  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*)override;
  

private:
  // needed but not implemented
  vtkCGALBoolean(const vtkCGALBoolean&) = delete;
  void operator=(const vtkCGALBoolean&) = delete;
};
#endif
