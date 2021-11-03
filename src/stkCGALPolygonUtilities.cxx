#include "stkCGALPolygonUtilities.h"

#include <vtkObjectFactory.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCellData.h>
#include <vtkIdTypeArray.h>

#include <vtkAppendPolyData.h>

vtkStandardNewMacro(stkCGALPolygonUtilities);

//----------------------------------------------------------------------------

/** @brief Converts a Polygon with holes list 2 (CGAL) into a PolyData (VTK).
*
*  @param pmesh The input Polygon with holes list
*  @param usg The output PolyData
*  @return bool Success (true) or failure (false)
*/
bool stkCGALPolygonUtilities::PwhList2ToPolyData(const Pwh_list_2& pmesh, vtkPolyData* polydata, std::string pwhIdArrayName /* = "PolygonWithHolesId" */, bool oneCell /* = false */)
{
	vtkNew<vtkAppendPolyData> appendFilter;

	typename Pwh_list_2::const_iterator polygon_iterator;
	typename CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hole_iterator;

	int id = 0;

	for (polygon_iterator = pmesh.begin(); polygon_iterator != pmesh.end(); ++polygon_iterator)
	{
		vtkNew<vtkPolyData> polygonWithHoles;
		stkCGALPolygonUtilities::PolygonWithHoles2ToPolyData(*polygon_iterator, polygonWithHoles, pwhIdArrayName, id, oneCell);
		appendFilter->AddInputData(polygonWithHoles);

		++id;
	}

	if (appendFilter->GetNumberOfInputConnections(0) > 0)
	{
		appendFilter->Update();
		polydata->ShallowCopy(appendFilter->GetOutput());
	}

	return true;
}

//----------------------------------------------------------------------------

/** @brief Converts a Polygon with holes 2 (CGAL) into a PolyData (VTK).
*
*  @param pmesh The input Polygon with holes
*  @param usg The output PolyData
*  @return bool Success (true) or failure (false)
*/
bool stkCGALPolygonUtilities::PolygonWithHoles2ToPolyData(const Polygon_with_holes_2& pmesh, vtkPolyData* polydata, std::string pwhIdArrayName /* = "PolygonWithHolesId" */, int pwhId /* = 0 */, bool oneCell /* = false */)
{
	vtkNew<vtkAppendPolyData> appendFilter;

	typename CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hole_iterator;

	vtkNew<vtkIdTypeArray> pwhIdArray;
	pwhIdArray->SetName(pwhIdArrayName.c_str());
	pwhIdArray->SetNumberOfComponents(1);

	vtkNew<vtkPolyData> boundary;
	stkCGALPolygonUtilities::Polygon2ToVtkPolyLine<K>(pmesh.outer_boundary(), boundary, oneCell);
	appendFilter->AddInputData(boundary);

	for (hole_iterator = pmesh.holes_begin(); hole_iterator != pmesh.holes_end(); ++hole_iterator)
	{
		vtkNew<vtkPolyData> hole;
		stkCGALPolygonUtilities::Polygon2ToVtkPolyLine<K>(*hole_iterator, hole, oneCell);
		appendFilter->AddInputData(hole);
	}

	if (appendFilter->GetNumberOfInputConnections(0) > 0)
	{
		appendFilter->Update();
		polydata->ShallowCopy(appendFilter->GetOutput());

		// Setup ID array
		pwhIdArray->SetNumberOfTuples(polydata->GetNumberOfCells());
		pwhIdArray->Fill(pwhId);
		polydata->GetCellData()->AddArray(pwhIdArray);
	}

	return true;
}

//----------------------------------------------------------------------------

void stkCGALPolygonUtilities::PrintPolygonSet2Properties(const Polygon_set_2& pmesh, std::string message, bool printPoints)
{
	Pwh_list_2 pmeshList;
	pmesh.polygons_with_holes(std::back_inserter(pmeshList));

	stkCGALPolygonUtilities::PrintPwhList2Properties(pmeshList, message, printPoints);
}

//----------------------------------------------------------------------------

void stkCGALPolygonUtilities::PrintPwhList2Properties(const Pwh_list_2& pmesh, std::string message, bool printPoints)
{
	cout << "========= " << message << " =========" << endl;

	typename Pwh_list_2::const_iterator polygon_iterator;

	for (polygon_iterator = pmesh.begin(); polygon_iterator != pmesh.end(); ++polygon_iterator)
	{
		stkCGALPolygonUtilities::PrintPolygonWithHoles2Properties(*polygon_iterator, "", printPoints);
	}
}

//----------------------------------------------------------------------------

void stkCGALPolygonUtilities::PrintPolygonWithHoles2Properties(const Polygon_with_holes_2& pmesh, std::string message, bool printPoints)
{
	cout << "========= " << message << " =========" << endl;

	stkCGALPolygonUtilities::PrintPolygonProperties(pmesh.outer_boundary(), "Outer Boundary", printPoints);

	typename CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hole_iterator;
	int i = 0;
	for (hole_iterator = pmesh.holes_begin(); hole_iterator != pmesh.holes_end(); ++hole_iterator)
	{
		stkCGALPolygonUtilities::PrintPolygonProperties(*hole_iterator, "Hole " + std::to_string(i), printPoints);
		++i;
	}
}

//----------------------------------------------------------------------------

void stkCGALPolygonUtilities::PrintPolygonProperties(const Polygon_2& pmesh, std::string message, bool printPoints)
{
	cout << "========= " << message << " =========" << endl;

	typename CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hole_iterator;

	if (pmesh.size() > 0)
	{
		cout << "The polygon is " <<
			(pmesh.is_simple() ? "" : "not ") << "simple." << endl;
		// check if the polygon is convex
		cout << "The polygon is " <<
			(pmesh.is_convex() ? "" : "not ") << "convex." << endl;
		cout << "The polygon is " <<
			(pmesh.is_clockwise_oriented() ? "" : "not ") << "clockwise." << endl;
		cout << "Signed area: " << pmesh.area() << endl;
		cout << "The origin is " <<
			(pmesh.bounded_side(Point_2(0, 0)) ? "" : "not ") << "inside the polygon." << endl;

		typename Polygon_2::Vertex_const_iterator vertex_iterator;
		int i = 0;
		for (vertex_iterator = pmesh.vertices_begin(); vertex_iterator != pmesh.vertices_end(); ++vertex_iterator)
			++i;

		cout << "Number of Points: " << i << endl;

		if (printPoints)
		{
			cout << "Points: " << endl;

			for (vertex_iterator = pmesh.vertices_begin(); vertex_iterator != pmesh.vertices_end(); ++vertex_iterator)
            {
                std::cout << "\t" << CGAL::to_double(vertex_iterator->x().exact()) << ", " << CGAL::to_double(vertex_iterator->y().exact()) << endl;
            }
		}
	}
	else
	{
		cout << "The polygon is empty." << endl;
	}
}

//----------------------------------------------------------------------------
void stkCGALPolygonUtilities::GetPolygonPointCoordinates(
  CGAL::Polygon_2<CGAL::Exact_predicates_inexact_constructions_kernel>::Vertex_const_iterator
    vertex_iterator,
  double& x, double& y)
{
  x = CGAL::to_double(vertex_iterator->x());
  y = CGAL::to_double(vertex_iterator->y());
};

void stkCGALPolygonUtilities::GetPolygonPointCoordinates(
  CGAL::Polygon_2<CGAL::Exact_predicates_exact_constructions_kernel>::Vertex_const_iterator
    vertex_iterator,
  double& x, double& y)
{
  x = CGAL::to_double(vertex_iterator->x().exact());
  y = CGAL::to_double(vertex_iterator->y().exact());
};
