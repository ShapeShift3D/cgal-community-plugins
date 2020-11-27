#include <vtkCGALIsotropicRemeshingFilter.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkTimerLog.h>

#include <CGAL/Surface_mesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>

vtkStandardNewMacro(vtkCGALIsotropicRemeshingFilter);

namespace PMP = CGAL::Polygon_mesh_processing;

typedef CGAL::Simple_cartesian<double>                              K;
typedef CGAL::Surface_mesh<K::Point_3>                              Mesh;
typedef boost::property_map<Mesh, CGAL::vertex_point_t>::type       VPMap;
typedef boost::property_map_value<Mesh, CGAL::vertex_point_t>::type Point_3;
typedef boost::graph_traits<Mesh>::vertex_descriptor                vertex_descriptor;
typedef boost::graph_traits<Mesh>::edge_descriptor                  edge_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor                  face_descriptor;
typedef boost::graph_traits<Mesh>::halfedge_descriptor              halfedge_descriptor;

//-----------------------------------------------------------------------------
struct halfedge2edge
{
  halfedge2edge(const Mesh& m, std::vector<edge_descriptor>& edges)
    : m_mesh(m), m_edges(edges)
  {}
  void operator()(const halfedge_descriptor& h) const
  {
    m_edges.push_back(edge(h, m_mesh));
  }
  const Mesh& m_mesh;
  std::vector<edge_descriptor>& m_edges;
};

//-----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
vtkCGALIsotropicRemeshingFilter::vtkCGALIsotropicRemeshingFilter()
{
  this->TargetEdgeLength = 0.0;
  this->TargetEdgeLengthInfo = 0.0;
  this->NumberOfIterations = 1;
  this->PreserveBorder = 1;
  this->MeshingMaskArrayName = nullptr;
}

//-----------------------------------------------------------------------------
vtkCGALIsotropicRemeshingFilter::~vtkCGALIsotropicRemeshingFilter()
{
  if (this->MeshingMaskArrayName)
  {
    delete this->MeshingMaskArrayName;
    this->MeshingMaskArrayName = nullptr;
  }
}

//-----------------------------------------------------------------------------
// Gets the input
// Creates CGAL::Surface_mesh from vtkPolydata
// Calls the CGAL::isotropic_remeshing algorithm
// Fills the output vtkPolyData from the result.
int vtkCGALIsotropicRemeshingFilter::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector,
  vtkInformationVector* outputVector)
{
  // Get the input and output data objects.
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  vtkPolyData* inputMesh = vtkPolyData::SafeDownCast(
    inInfo->Get(vtkDataObject::DATA_OBJECT()));

  /********************************************
   * Create a SurfaceMesh from the input mesh *
   ********************************************/
  vtkTimerLog::MarkStartEvent("Converting VTK -> CGAL");

  Mesh mesh;
  VPMap vpmap = get(CGAL::vertex_point, mesh);

  // Get nb of points and cells
  vtkIdType nb_points = inputMesh->GetNumberOfPoints();
  vtkIdType nb_cells = inputMesh->GetNumberOfCells();

  // Extract points
  std::vector<vertex_descriptor> vertex_map(nb_points);
  for (vtkIdType i = 0; i < nb_points; ++i)
  {
    double coords[3];
    inputMesh->GetPoint(i, coords);
    vertex_descriptor v = add_vertex(mesh);
    put(vpmap, v, K::Point_3(coords[0], coords[1], coords[2]));
    vertex_map[i] = v;
  }

  // Extract cells
  for (vtkIdType i = 0; i < nb_cells; ++i)
  {
    vtkCell* cell_ptr = inputMesh->GetCell(i);
    vtkIdType nb_vertices = cell_ptr->GetNumberOfPoints();
    std::vector<vertex_descriptor> vr(nb_vertices);
    for (vtkIdType k = 0; k < nb_vertices; ++k)
    {
      vr[k] = vertex_map[cell_ptr->GetPointId(k)];
    }
    CGAL::Euler::add_face(vr, mesh);
  }

  std::vector<vertex_descriptor> isolated_vertices;
  for(Mesh::vertex_iterator vit = mesh.vertices_begin();
      vit != mesh.vertices_end(); ++vit)
  {
    if (mesh.is_isolated(*vit))
    {
      isolated_vertices.push_back(*vit);
    }
  }

  for (std::size_t i = 0; i < isolated_vertices.size(); ++i)
  {
    mesh.remove_vertex(isolated_vertices[i]);
  }

  if(!is_triangle_mesh(mesh))
  {
    vtkErrorMacro("The input mesh must be triangulated ");
    return 0;
  }

  vtkTimerLog::MarkEndEvent("Converting VTK -> CGAL");

  /*****************************
   * Process border edges *
   *****************************/
  if (this->PreserveBorder)
  {
    vtkTimerLog::MarkStartEvent("CGAL splitting border");

    // From https://doc.cgal.org/latest/Polygon_mesh_processing/group__PMP__meshing__grp.html
    // Precondition: if constraints protection is activated,
    // the constrained edges must not be longer than 4/3 * TargetEdgeLength
    double constrainedEdgeLength = 4.0 * this->TargetEdgeLength / 3.0;

    std::vector<edge_descriptor> border;
    PMP::border_halfedges(faces(mesh), mesh,
      boost::make_function_output_iterator(halfedge2edge(mesh, border)));
    PMP::split_long_edges(border, constrainedEdgeLength, mesh);

    vtkTimerLog::MarkEndEvent("CGAL splitting border");
  }

  /*****************************
   * Apply Isotropic remeshing *
   *****************************/
  vtkTimerLog::MarkStartEvent("CGAL isotropic remeshing");

  if (this->PreserveBorder)
  {
    PMP::isotropic_remeshing(faces(mesh), this->TargetEdgeLength, mesh,
      PMP::parameters::number_of_iterations(this->NumberOfIterations).protect_constraints(true));
  }
  else
  {
    PMP::isotropic_remeshing(faces(mesh), this->TargetEdgeLength, mesh,
      PMP::parameters::number_of_iterations(this->NumberOfIterations));
  }

  vtkTimerLog::MarkEndEvent("CGAL isotropic remeshing");

  /**********************************
   * Pass the mesh data to the output *
   **********************************/
  vtkTimerLog::MarkStartEvent("Converting CGAL -> VTK");

  vtkPolyData* output = vtkPolyData::SafeDownCast(
    outInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkNew<vtkPoints> const vtk_points;
  vtkNew<vtkCellArray> const vtk_cells;
  vtk_points->Allocate(mesh.number_of_vertices());
  vtk_cells->Allocate(mesh.number_of_faces());
  std::vector<vtkIdType> Vids(num_vertices(mesh));
  vtkIdType inum = 0;

  for(vertex_descriptor v : vertices(mesh))
  {
    const K::Point_3& p = get(vpmap, v);
    vtk_points->InsertNextPoint(CGAL::to_double(p.x()),
                                CGAL::to_double(p.y()),
                                CGAL::to_double(p.z()));
    Vids[v] = inum++;
  }

  for(face_descriptor f : faces(mesh))
  {
    vtkNew<vtkIdList> cell;
    for(halfedge_descriptor h : halfedges_around_face(halfedge(f, mesh), mesh))
      cell->InsertNextId(Vids[target(h, mesh)]);

    vtk_cells->InsertNextCell(cell);
  }

  output->SetPoints(vtk_points);
  output->SetPolys(vtk_cells);
  output->Squeeze();

  vtkTimerLog::MarkEndEvent("Converting CGAL -> VTK");

  return 1;
}

//-----------------------------------------------------------------------------
void vtkCGALIsotropicRemeshingFilter::PrintSelf(
  std::ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  os << indent << "TargetEdgeLength     : " << this->TargetEdgeLength << std::endl;
  os << indent << "TargetEdgeLengthInfo : " << this->TargetEdgeLengthInfo << std::endl;
  os << indent << "NumberOfIterations   : " << this->NumberOfIterations << std::endl;
  os << indent << "PreserveBorder       : " << this->PreserveBorder << std::endl;
  os << indent << "MeshingMaskArrayName : "
    << (this->MeshingMaskArrayName ? this->MeshingMaskArrayName : "(none)") << std::endl;
}

//------------------------------------------------------------------------------
int vtkCGALIsotropicRemeshingFilter::RequestInformation(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector,
  vtkInformationVector* outputVector)
{
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  // Sets the bounds of the output.
  outInfo->Set(vtkDataObject::BOUNDING_BOX(),
               inInfo->Get(vtkDataObject::BOUNDING_BOX()), 6);

  vtkPolyData* input= vtkPolyData::SafeDownCast(
        inInfo->Get(vtkDataObject::DATA_OBJECT()));

  // Computes the initial target length:
  double* bounds = input->GetBounds();
  double diagonal = std::sqrt((bounds[0]-bounds[1]) * (bounds[0]-bounds[1]) +
                              (bounds[2]-bounds[3]) * (bounds[2]-bounds[3]) +
                              (bounds[4]-bounds[5]) * (bounds[4]-bounds[5]));

  this->TargetEdgeLengthInfo = 0.01 * diagonal;

  return 1;
}
