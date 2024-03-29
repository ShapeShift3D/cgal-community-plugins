/**
 * @class stkCGALPolyLineSetToPolygonSet
 * @brief Converts a VTK polyline set to a CGAL polygon set
 *
 * Converter class used for other filters.
 *
 * @sa
 * stkCGALPolyLineSetToPolygonSet
 */
#pragma once

#include "vtkPolyDataAlgorithm.h"
#include <stkCGALModule.h>

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Surface_mesh.h>

#include <string>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2> Pwh_list_2;
typedef CGAL::Polygon_set_2<K> Polygon_set_2;

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALPolyLineSetToPolygonSet : public vtkPolyDataAlgorithm
{
public:
  static stkCGALPolyLineSetToPolygonSet* New();
  vtkTypeMacro(stkCGALPolyLineSetToPolygonSet, vtkPolyDataAlgorithm);

  vtkPolyData* GetInputPolyLineSet();
  Polygon_set_2* GetOutputPolygonSet();

  enum Planes
  {
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
   * Name of the Polygon With Holes ID array.
   */
  vtkSetMacro(PwhIdArrayName, std::string);
  vtkGetMacro(PwhIdArrayName, std::string);
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
  stkCGALPolyLineSetToPolygonSet();
  ~stkCGALPolyLineSetToPolygonSet() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  int Plane;
  std::string PwhIdArrayName;
  bool DebugMode;
  bool PrintPoints;

private:
  stkCGALPolyLineSetToPolygonSet(const stkCGALPolyLineSetToPolygonSet&) = delete;
  void operator=(const stkCGALPolyLineSetToPolygonSet&) = delete;

  bool ProcessPwh(vtkPolyData* pwhPoly, int firstCoord, int secondCoord);

  Polygon_set_2 PolygonSet;
};
