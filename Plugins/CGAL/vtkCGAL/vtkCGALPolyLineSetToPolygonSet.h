#ifndef vtkCGALPolyLineSetToPolygonSet_h
#define vtkCGALPolyLineSetToPolygonSet_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <vtkCGALModule.h>

#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_set_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel	K;
typedef K::Point_2											Point_2;
typedef CGAL::Polygon_2<K>									Polygon_2;
typedef CGAL::Polygon_with_holes_2<K>						Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>						Pwh_list_2;
typedef CGAL::Polygon_set_2<K>								Polygon_set_2;

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALPolyLineSetToPolygonSet : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGALPolyLineSetToPolygonSet* New();
  vtkTypeMacro(vtkCGALPolyLineSetToPolygonSet, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputPolyLineSet();
  Polygon_set_2* GetOutputPolygonSet();


  enum Planes {
      XY = 1,
      YZ,
      XZ
  };

  vtkGetMacro(Plane, int);
  vtkSetMacro(Plane, int);

  void SetPlaneToXY() { Plane = Planes::XY; }
  void SetPlaneToYZ() { Plane = Planes::YZ; }
  void SetPlaneToXZ() { Plane = Planes::XZ; }

protected:
  vtkCGALPolyLineSetToPolygonSet();
  ~vtkCGALPolyLineSetToPolygonSet(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  int Plane;

private:
  // needed but not implemented
  vtkCGALPolyLineSetToPolygonSet(const vtkCGALPolyLineSetToPolygonSet&) = delete;
  void operator=(const vtkCGALPolyLineSetToPolygonSet&) = delete;

  bool DebugMode;
  Polygon_set_2 PolygonSet;
};
#endif
