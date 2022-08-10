#include <stkCGALConstrainedDelaunayTriangulation.h>
#include <stkCGALUtilities.h>
#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkGeometryFilter.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPolyData.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkSmartPointer.h>
#include <vtkThreshold.h>
#include <vtkTimerLog.h>
#include <vtkUnstructuredGrid.h>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <iostream>

vtkStandardNewMacro(stkCGALConstrainedDelaunayTriangulation);

namespace PMP = CGAL::Polygon_mesh_processing;

typedef CDT::Finite_vertices_iterator Finite_vertices_iterator;
typedef CDT::Finite_faces_iterator Finite_faces_iterator;

//-----------------------------------------------------------------------------
int stkCGALConstrainedDelaunayTriangulation::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  // Get the input and output data objects.
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  vtkPolyData* input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));

  vtkPolyData* output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  CDT cdt;

  vtkNew<vtkPolyDataConnectivityFilter> connectivityFilter;
  connectivityFilter->SetInputData(input);
  connectivityFilter->SetExtractionModeToAllRegions();
  connectivityFilter->Update();

  int numberOfRegions = connectivityFilter->GetNumberOfExtractedRegions();

  for (int l = 0; l < numberOfRegions; l++)
  {
    vtkNew<vtkPolyDataConnectivityFilter> connectivityFilter2;
    connectivityFilter2->SetInputData(input);
    connectivityFilter2->SetExtractionModeToSpecifiedRegions();
    connectivityFilter2->InitializeSpecifiedRegionList();
    connectivityFilter2->AddSpecifiedRegion(l);
    connectivityFilter2->Update();

    vtkNew<vtkCleanPolyData> clean;
    clean->SetInputData(connectivityFilter2->GetOutput());
    clean->Update();

    vtkNew<vtkPolyData> region_i;
    region_i->DeepCopy(clean->GetOutput());

    Polygon_2 polygon1;
    double x = 0;
    double y = 0;
    for (int n = 0; n < region_i->GetNumberOfPoints(); n++)
    {
      x = region_i->GetPoint(n)[0];
      y = region_i->GetPoint(n)[1];
      polygon1.push_back(Point(x, y));
    }

    cdt.insert_constraint(polygon1.vertices_begin(), polygon1.vertices_end(), true);
  }

  //  Mark facets that are inside the domain bounded by the polygon
  markDomains(cdt);

  vtkNew<vtkPoints> vtk_points;
  vtkNew<vtkCellArray> vtk_cells;
  vtkNew<vtkUnstructuredGrid> outputUG;

  std::map<Point, vtkIdType> pointsMap;
  vtkIdType i = 0;
  Point p;

  for (Finite_vertices_iterator it = cdt.finite_vertices_begin(); it != cdt.finite_vertices_end();
       ++it)
  {
    p = it->point();
    std::cout << p << std::endl;
    vtk_points->InsertNextPoint(
      CGAL::to_double(it->point().x()), CGAL::to_double(it->point().y()), 0);
    pointsMap[p] = i;
    i++;
  }

  int count = 0;
  Point P;
  for (Finite_faces_iterator f = cdt.finite_faces_begin(); f != cdt.finite_faces_end(); ++f)
  {
    if (f->info().in_domain())
    {
      ++count;
      vtkIdList* cell = vtkIdList::New();
      for (int k = 0; k < 3; k++)
      {
        P = f->vertex(k)->point();
        std::cout << P << std::endl;
        cell->InsertNextId(pointsMap.find(P)->second);
      }
      vtk_cells->InsertNextCell(cell);
      cell->Delete();
    }
  }
  std::cout << "There are " << count << " facets in the domain." << std::endl;

  outputUG->SetPoints(vtk_points);
  outputUG->SetCells(5, vtk_cells);

  vtkNew<vtkGeometryFilter> UGToPolyFilter;
  UGToPolyFilter->SetInputData(outputUG);
  UGToPolyFilter->Update();

  output->ShallowCopy(UGToPolyFilter->GetOutput());

  return 1;
}

//-----------------------------------------------------------------------------
void stkCGALConstrainedDelaunayTriangulation::PrintSelf(std::ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
void stkCGALConstrainedDelaunayTriangulation::mark_domains(
  CDT& ct, Face_handle start, int index, std::list<CDT::Edge>& border)
{
  if (start->info().nesting_level != -1)
  {
    return;
  }
  std::list<Face_handle> queue;
  queue.push_back(start);
  while (!queue.empty())
  {
    Face_handle fh = queue.front();
    queue.pop_front();
    if (fh->info().nesting_level == -1)
    {
      fh->info().nesting_level = index;
      for (int i = 0; i < 3; i++)
      {
        CDT::Edge e(fh, i);
        Face_handle n = fh->neighbor(i);
        if (n->info().nesting_level == -1)
        {
          if (ct.is_constrained(e))
            border.push_back(e);
          else
            queue.push_back(n);
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
void stkCGALConstrainedDelaunayTriangulation::markDomains(CDT& cdt)
{
  for (CDT::Face_handle f : cdt.all_face_handles())
  {
    f->info().nesting_level = -1;
  }
  std::list<CDT::Edge> border;
  mark_domains(cdt, cdt.infinite_face(), 0, border);
  while (!border.empty())
  {
    CDT::Edge e = border.front();
    border.pop_front();
    Face_handle n = e.first->neighbor(e.second);
    if (n->info().nesting_level == -1)
    {
      mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
    }
  }
}
