#ifndef vtkCGALPolygonSetToPolyLineSet_h
#define vtkCGALPolygonSetToPolyLineSet_h
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
class VTKCGAL_EXPORT vtkCGALPolygonSetToPolyLineSet : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGALPolygonSetToPolyLineSet* New();
  vtkTypeMacro(vtkCGALPolygonSetToPolyLineSet, vtkPolyDataAlgorithm);
  
  void SetInputPolygonSet(Polygon_set_2& polygonSet);
  vtkPolyData* GetOutputPolyLineSet();

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

  //@{
  /**
  * Outputs Polylines having one cell per polygon
  */
  vtkGetMacro(OneCell, bool);
  vtkSetMacro(OneCell, bool);
  vtkBooleanMacro(OneCell, bool);
  //@}

  //@{
  /**
  * Enables debug printing
  */
  vtkGetMacro(DebugMode, bool);
  vtkSetMacro(DebugMode, bool);
  vtkBooleanMacro(DebugMode, bool);
  //@}

  //@{
  /**
  * Print point coordinates if debug printing is true
  */
  vtkGetMacro(PrintPoints, bool);
  vtkSetMacro(PrintPoints, bool);
  vtkBooleanMacro(PrintPoints, bool);
  //@}

protected:
  vtkCGALPolygonSetToPolyLineSet();
  ~vtkCGALPolygonSetToPolyLineSet(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  int Plane;
  bool OneCell;
  bool DebugMode;
  bool PrintPoints;

private:
  // needed but not implemented
  vtkCGALPolygonSetToPolyLineSet(const vtkCGALPolygonSetToPolyLineSet&) = delete;
  void operator=(const vtkCGALPolygonSetToPolyLineSet&) = delete;

  Polygon_set_2 PolygonSet;
};
#endif
