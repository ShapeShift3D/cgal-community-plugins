#include <stkCGALConstrainedDelaunayTriangulation.h>
#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkThreshold.h>
#include <vtkTimerLog.h>

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

//-----------------------------------------------------------------------------
int stkCGALConstrainedDelaunayTriangulation::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  // Get the input and output data objects.
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  vtkPolyData* input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));

  vtkPolyData* output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  // construct two non-intersecting nested polygons
  Polygon_2 polygon1;
  polygon1.push_back(Point(0, 0));
  polygon1.push_back(Point(2, 0));
  polygon1.push_back(Point(2, 2));
  polygon1.push_back(Point(0, 2));
  Polygon_2 polygon2;
  polygon2.push_back(Point(0.5, 0.5));
  polygon2.push_back(Point(1.5, 0.5));
  polygon2.push_back(Point(1.5, 1.5));
  polygon2.push_back(Point(0.5, 1.5));
  // Insert the polygons into a constrained triangulation
  CDT cdt;
  cdt.insert_constraint(polygon1.vertices_begin(), polygon1.vertices_end(), true);
  cdt.insert_constraint(polygon2.vertices_begin(), polygon2.vertices_end(), true);
  // Mark facets that are inside the domain bounded by the polygon
  markDomains(cdt);
  int count = 0;
  for (Face_handle f : cdt.finite_face_handles())
  {
    if (f->info().in_domain())
      ++count;
  }
  std::cout << "There are " << count << " facets in the domain." << std::endl;

  // output->SetPoints(vtk_points);
  // output->SetPolys(vtk_cells);
  output->Squeeze();

  return 1;
}

//-----------------------------------------------------------------------------
void stkCGALConstrainedDelaunayTriangulation::PrintSelf(std::ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
/** @brief Computes the bbox's diagonal length to set the default target edge length.
 *
 *  @param vtkNotUsed(request)
 *  @param inputVector
 *  @tparam inputVector
 *
 *  @return int Success (1) or failure (0)
 */
int stkCGALConstrainedDelaunayTriangulation::RequestInformation(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  // Sets the bounds of the output.
  outInfo->Set(vtkDataObject::BOUNDING_BOX(), inInfo->Get(vtkDataObject::BOUNDING_BOX()), 6);

  vtkPolyData* input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));

  // Computes the initial target length:
  double* bounds = input->GetBounds();
  double diagonal = std::sqrt((bounds[0] - bounds[1]) * (bounds[0] - bounds[1]) +
    (bounds[2] - bounds[3]) * (bounds[2] - bounds[3]) +
    (bounds[4] - bounds[5]) * (bounds[4] - bounds[5]));

  this->TargetEdgeLengthInfo = 0.01 * diagonal;

  return 1;
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
