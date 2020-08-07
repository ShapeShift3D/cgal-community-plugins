/**
* \class vtkCGALPolygonUtilities
*
* \brief Set of CGAL utility functions usable by other classes in this module.
*        
*/


#include "vtkCGALPolygonUtilities.h"

#include <vtkObjectFactory.h>
#include <vtkUnstructuredGrid.h>
#include <vtkPolyData.h>
#include <vtkCellData.h>
#include <vtkIdTypeArray.h>

#include <vtkAppendPolyData.h>

vtkStandardNewMacro(vtkCGALPolygonUtilities);

// ----------------------------------------------------------------------------
vtkCGALPolygonUtilities::vtkCGALPolygonUtilities()
{}

// ----------------------------------------------------------------------------
vtkCGALPolygonUtilities::~vtkCGALPolygonUtilities()
{}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into Polygon 2 (CGAL).
*          This method does not write into our PolyData structure.
*		   Hence, we do not need to copy them before calling this function.
*
*  @param polyData The input PolyData
*  @param tmesh The resulting Polygon 2 Mesh
*  @return bool Success (true) or failure (false)
*/
bool vtkCGALPolygonUtilities::vtkPolyDataToPolygon2(vtkPointSet* polyData, Polygon_2& tmesh, int& coordinate0, int& coordinate1)
{
	// get nb of points and cells
	vtkIdType nb_points = polyData->GetNumberOfPoints();

	// Extract points
	for (vtkIdType i = 0; i < nb_points; ++i)
	{
		double coords[3];
		polyData->GetPoint(i, coords);

		tmesh.push_back(Point_2(coords[coordinate0], coords[coordinate1]));
	}

	return true;
}

//----------------------------------------------------------------------------

/** @brief Converts a Polygon with holes list 2 (CGAL) into a PolyData (VTK).
*
*  @param pmesh The input Polygon with holes list
*  @param usg The output PolyData
*  @return bool Success (true) or failure (false)
*/
bool vtkCGALPolygonUtilities::PwhList2ToPolyData(const Pwh_list_2& pmesh, vtkPolyData* polydata, std::string pwhIdArrayName /* = "PolygonWithHolesId" */, bool oneCell /* = false */)
{
	vtkNew<vtkAppendPolyData> appendFilter;

	typename Pwh_list_2::const_iterator polygon_iterator;
	typename CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hole_iterator;

	int id = 0;

	for (polygon_iterator = pmesh.begin(); polygon_iterator != pmesh.end(); ++polygon_iterator)
	{
		vtkNew<vtkPolyData> polygonWithHoles;
		vtkCGALPolygonUtilities::PolygonWithHoles2ToPolyData(*polygon_iterator, polygonWithHoles, pwhIdArrayName, id, oneCell);
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
bool vtkCGALPolygonUtilities::PolygonWithHoles2ToPolyData(const Polygon_with_holes_2& pmesh, vtkPolyData* polydata, std::string pwhIdArrayName /* = "PolygonWithHolesId" */, int pwhId /* = 0 */, bool oneCell /* = false */)
{
	vtkNew<vtkAppendPolyData> appendFilter;

	typename CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hole_iterator;

	vtkNew<vtkIdTypeArray> pwhIdArray;
	pwhIdArray->SetName(pwhIdArrayName.c_str());
	pwhIdArray->SetNumberOfComponents(1);

	vtkNew<vtkPolyData> boundary;
	vtkCGALPolygonUtilities::Polygon2ToPolyLine(pmesh.outer_boundary(), boundary, oneCell);
	appendFilter->AddInputData(boundary);

	for (hole_iterator = pmesh.holes_begin(); hole_iterator != pmesh.holes_end(); ++hole_iterator)
	{
		vtkNew<vtkPolyData> hole;
		vtkCGALPolygonUtilities::Polygon2ToPolyLine(*hole_iterator, hole, oneCell);
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

/** @brief Converts a Polygon 2 (CGAL) into a PolyData (VTK).
*
*  @param pmesh The input Polygon 2
*  @param usg The output PolyData
*  @return bool Success (true) or failure (false)
*/
bool vtkCGALPolygonUtilities::Polygon2ToPolyLine(const Polygon_2& pmesh, vtkPolyData* polyline, bool oneCell /* = false */)
{
	vtkNew<vtkPoints> vtk_points;

	typename Polygon_2::Vertex_const_iterator vertex_iterator;

	for (vertex_iterator = pmesh.vertices_begin(); vertex_iterator != pmesh.vertices_end(); ++vertex_iterator)
	{
		vtk_points->InsertNextPoint(vertex_iterator->x().exact().to_double(),
			vertex_iterator->y().exact().to_double(),
			0);
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

//----------------------------------------------------------------------------

void vtkCGALPolygonUtilities::PrintPolygonSet2Properties(const Polygon_set_2& pmesh, std::string message, bool printPoints)
{
	Pwh_list_2 pmeshList;
	pmesh.polygons_with_holes(std::back_inserter(pmeshList));

	vtkCGALPolygonUtilities::PrintPwhList2Properties(pmeshList, message, printPoints);
}

//----------------------------------------------------------------------------

void vtkCGALPolygonUtilities::PrintPwhList2Properties(const Pwh_list_2& pmesh, std::string message, bool printPoints)
{
	cout << "========= " << message << " =========" << endl;

	typename Pwh_list_2::const_iterator polygon_iterator;

	for (polygon_iterator = pmesh.begin(); polygon_iterator != pmesh.end(); ++polygon_iterator)
	{
		vtkCGALPolygonUtilities::PrintPolygonWithHoles2Properties(*polygon_iterator, "", printPoints);
	}
}

//----------------------------------------------------------------------------

void vtkCGALPolygonUtilities::PrintPolygonWithHoles2Properties(const Polygon_with_holes_2& pmesh, std::string message, bool printPoints)
{
	cout << "========= " << message << " =========" << endl;

	vtkCGALPolygonUtilities::PrintPolygonProperties(pmesh.outer_boundary(), "Outer Boundary", printPoints);

	typename CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hole_iterator;
	int i = 0;
	for (hole_iterator = pmesh.holes_begin(); hole_iterator != pmesh.holes_end(); ++hole_iterator)
	{
		vtkCGALPolygonUtilities::PrintPolygonProperties(*hole_iterator, "Hole " + std::to_string(i), printPoints);
		++i;
	}
}

//----------------------------------------------------------------------------

void vtkCGALPolygonUtilities::PrintPolygonProperties(const Polygon_2& pmesh, std::string message, bool printPoints)
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
				std::cout << "\t" << vertex_iterator->x().exact().to_double() << ", " << vertex_iterator->y().exact().to_double() << endl;
		}
	}
	else
	{
		cout << "The polygon is empty." << endl;
	}
}
