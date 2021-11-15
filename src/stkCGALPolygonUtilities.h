/**
 * @class stkCGALPolygonUtilities
 * @brief Set of CGAL-based utility functions usable by Polygon-related classes in this module.
 *
 * @sa
 * stkCGALPolygonUtilities
 */
#pragma once

#include "vtkObject.h"
#include <stkCGALModule.h>

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_set_2.h>

#include <string>

#include <vtkCleanPolyData.h>
#include <vtkContourLoopExtraction.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkTriangleFilter.h>

class vtkPointSet;
class vtkUnstructuredGrid;

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALPolygonUtilities : public vtkObject
{
public:
  static stkCGALPolygonUtilities* New();
  vtkTypeMacro(stkCGALPolygonUtilities, vtkObject);

  typedef CGAL::Exact_predicates_exact_constructions_kernel K;
  typedef K::Point_2 Point_2;
  typedef CGAL::Polygon_2<K> Polygon_2;
  typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;
  typedef std::list<Polygon_with_holes_2> Pwh_list_2;
  typedef CGAL::Polygon_set_2<K> Polygon_set_2;

  template<typename Kernal,typename Point>
  static bool vtkPolyDataToPolygon2(
    vtkPointSet* polydata, CGAL::Polygon_2<Kernal>& tmesh, int coordinate0, int coordinate1);

  template<typename Kernal>
  static bool Polygon2ToVtkPolyLine(
    const CGAL::Polygon_2<Kernal>& pmesh, vtkPolyData* polyline, bool oneCell = false);

  static bool PwhList2ToPolyData(const Pwh_list_2& pmesh, vtkPolyData* polydata,
    std::string pwhIdArrayName = "PolygonWithHolesId", bool oneCell = false);

  static bool PolygonWithHoles2ToPolyData(const Polygon_with_holes_2& pmesh, vtkPolyData* polydata,
    std::string pwhIdArrayName = "PolygonWithHolesId", int pwhId = 0, bool oneCell = false);

  static void PrintPolygonSet2Properties(
    const Polygon_set_2& pmesh, std::string message, bool printPoints);

  static void PrintPwhList2Properties(
    const Pwh_list_2& pmesh, std::string message, bool printPoints);

  static void PrintPolygonWithHoles2Properties(
    const Polygon_with_holes_2& pmesh, std::string message, bool printPoints);

  static bool Polygon2ToPolyLine(
    const Polygon_2& pmesh, vtkPolyData* polyline, bool oneCell = false);

  static void PrintPolygonProperties(const Polygon_2& pmesh, std::string message, bool printPoints);

  static void GetPolygonPointCoordinates(
    CGAL::Polygon_2<CGAL::Exact_predicates_inexact_constructions_kernel>::Vertex_const_iterator
      vertex_iterator,
    double& x, double& y);
  static void GetPolygonPointCoordinates(
    CGAL::Polygon_2<CGAL::Exact_predicates_exact_constructions_kernel>::Vertex_const_iterator
      vertex_iterator,
    double& x, double& y);

protected:
  stkCGALPolygonUtilities() = default;
  ~stkCGALPolygonUtilities() = default;

private:
  stkCGALPolygonUtilities(const stkCGALPolygonUtilities&) = delete;
  void operator=(const stkCGALPolygonUtilities&) = delete;
};

//-------------------------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into Polygon 2 (CGAL).
 *          This method does not write into our PolyData structure.
 *		   Hence, we do not need to copy them before calling this function.
 *
 *  @param polyData The input PolyData
 *  @param tmesh The resulting Polygon 2 Mesh
 *  @param T Contruction Kernal for the CGAL Polygon
 *  @return bool Success (true) or failure (false)
 */
template<typename Kernal,typename Point>
bool stkCGALPolygonUtilities::vtkPolyDataToPolygon2(
  vtkPointSet* polyData, CGAL::Polygon_2<Kernal>& tmesh, int coordinate0, int coordinate1)
{
  vtkNew<vtkContourLoopExtraction> loopExtractionFilter;
  loopExtractionFilter->SetInputData(polyData);
  loopExtractionFilter->SetLoopClosureToBoundary();
  loopExtractionFilter->SetOutputModeToPolylines();
  loopExtractionFilter->Update();

  if (loopExtractionFilter->GetOutput()->GetNumberOfCells() != 1)
  {
    // There is more/less than one curve in the input
    return false;
  }

  vtkNew<vtkCleanPolyData> cleanFilter;
  cleanFilter->SetInputData(loopExtractionFilter->GetOutput());
  cleanFilter->ConvertPolysToLinesOff();
  cleanFilter->ConvertStripsToPolysOff();
  cleanFilter->ConvertPolysToLinesOff();
  cleanFilter->PointMergingOn();
  cleanFilter->Update();

  vtkNew<vtkPolyData> orderedVtkData;
  orderedVtkData->ShallowCopy(cleanFilter->GetOutput());
  // get nb of points and cells
  vtkIdType nb_points = orderedVtkData->GetNumberOfPoints();

  // Extract points
  for (vtkIdType i = 0; i < nb_points; ++i)
  {
    double coords[3];
    orderedVtkData->GetPoint(i, coords);
    tmesh.push_back(Point(coords[coordinate0], coords[coordinate1]));
  }

  return true;
}

//----------------------------------------------------------------------------

/** @brief Converts a Polygon 2 (CGAL) into a PolyData (VTK).
 *
 *  @param pmesh The input Polygon 2
 *  @param polyline The output PolyData
 *  @param T Contruction Kernal for the CGAL Polygon
 *  @return bool Success (true) or failure (false)
 *
 */
template<typename Kernal>
bool stkCGALPolygonUtilities::Polygon2ToVtkPolyLine(
  const CGAL::Polygon_2<Kernal>& pmesh, vtkPolyData* polyline, bool oneCell)
{
  vtkNew<vtkPoints> vtk_points;

  typename CGAL::Polygon_2<Kernal>::Vertex_const_iterator vertex_iterator;

  for (vertex_iterator = pmesh.vertices_begin(); vertex_iterator != pmesh.vertices_end();
       ++vertex_iterator)
  {
    double x = 0.0, y = 0.0;
    stkCGALPolygonUtilities::GetPolygonPointCoordinates(vertex_iterator, x, y);
    vtk_points->InsertNextPoint(x, y, 0.0);
  }

  polyline->SetPoints(vtk_points);

  if (polyline->GetLines() == nullptr || polyline->GetNumberOfLines() == 0)
  {
    // Not elegant but functional
    vtkNew<vtkCellArray> vtk_cells;

    if (oneCell)
    {
      if (pmesh.size() > 1)
      {
        if (pmesh.size() == 2)
        {
          vtk_cells->InsertNextCell(vtk_points->GetNumberOfPoints());
          for (vtkIdType i = 0; i < vtk_points->GetNumberOfPoints(); ++i)
          {
            vtk_cells->InsertCellPoint(i);
          }
        }
        else
        {
          vtk_cells->InsertNextCell(vtk_points->GetNumberOfPoints() + 1);
          for (vtkIdType i = 0; i < vtk_points->GetNumberOfPoints(); ++i)
          {
            vtk_cells->InsertCellPoint(i);
          }
          vtk_cells->InsertCellPoint(0);
        }
      }
    }
    else
    {
      if (pmesh.size() > 1)
      {
        for (vtkIdType i = 0; i < vtk_points->GetNumberOfPoints() - 1; ++i)
        {
          vtk_cells->InsertNextCell(2);
          vtk_cells->InsertCellPoint(i);
          vtk_cells->InsertCellPoint(i + 1);
        }

        if (pmesh.size() > 2)
        {
          vtk_cells->InsertNextCell(2);
          vtk_cells->InsertCellPoint(vtk_points->GetNumberOfPoints() - 1);
          vtk_cells->InsertCellPoint(0);
        }
      }
    }

    polyline->SetLines(vtk_cells);
  }

  return true;
}