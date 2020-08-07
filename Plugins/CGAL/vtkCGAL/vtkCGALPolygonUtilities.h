#ifndef vtkCGALPolygonUtilities_h
#define vtkCGALPolygonUtilities_h

#include "vtkObject.h"
#include <vtkCGALModule.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_set_2.h>

#include <string>

class vtkPointSet;
class vtkUnstructuredGrid;
class vtkPolyData;

class VTKCGAL_EXPORT vtkCGALPolygonUtilities : public vtkObject
{
public:
    static vtkCGALPolygonUtilities* New();
    vtkTypeMacro(vtkCGALPolygonUtilities, vtkObject);

    typedef CGAL::Exact_predicates_exact_constructions_kernel	K;
    typedef K::Point_2											Point_2;
    typedef CGAL::Polygon_2<K>									Polygon_2;
    typedef CGAL::Polygon_with_holes_2<K>						Polygon_with_holes_2;
    typedef std::list<Polygon_with_holes_2>						Pwh_list_2;
    typedef CGAL::Polygon_set_2<K>								Polygon_set_2;

    static bool vtkPolyDataToPolygon2(vtkPointSet* polydata, Polygon_2& tmesh, int& coordinate0, int& coordinate1);

    static bool PwhList2ToPolyData(const Pwh_list_2& pmesh, vtkPolyData* polydata, std::string pwhIdArrayName = "PolygonWithHolesId", bool oneCell = false);

    static bool PolygonWithHoles2ToPolyData(const Polygon_with_holes_2& pmesh, vtkPolyData* polydata, std::string pwhIdArrayName = "PolygonWithHolesId", int pwhId = 0, bool oneCell = false);

    static void PrintPolygonSet2Properties(const Polygon_set_2& pmesh, std::string message, bool printPoints);

    static void PrintPwhList2Properties(const Pwh_list_2& pmesh, std::string message, bool printPoints);

    static void PrintPolygonWithHoles2Properties(const Polygon_with_holes_2& pmesh, std::string message, bool printPoints);

    static bool Polygon2ToPolyLine(const Polygon_2& pmesh, vtkPolyData* polyline, bool oneCell = false);

    static void PrintPolygonProperties(const Polygon_2& pmesh, std::string message, bool printPoints);

protected:
    vtkCGALPolygonUtilities();
    ~vtkCGALPolygonUtilities();

private:
    vtkCGALPolygonUtilities(const vtkCGALPolygonUtilities&) = delete;
    void operator=(const vtkCGALPolygonUtilities&) = delete;
};
#endif
