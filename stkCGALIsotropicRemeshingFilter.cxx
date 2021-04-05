#include <vtkAppendPolyData.h>
#include <stkCGALIsotropicRemeshingFilter.h>
#include <vtkCleanPolyData.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkThreshold.h>
#include <vtkTimerLog.h>

#include <CGAL/Surface_mesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>

vtkStandardNewMacro(stkCGALIsotropicRemeshingFilter);

namespace PMP = CGAL::Polygon_mesh_processing;

typedef CGAL::Simple_cartesian<double>                                      K;
typedef CGAL::Surface_mesh<K::Point_3>                                      SurfaceMesh;
typedef boost::property_map<SurfaceMesh, CGAL::vertex_point_t>::type        VPMap;
typedef boost::property_map_value<SurfaceMesh, CGAL::vertex_point_t>::type  Point_3;
typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor                 vertex_descriptor;
typedef boost::graph_traits<SurfaceMesh>::edge_descriptor                   edge_descriptor;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor                   face_descriptor;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor               halfedge_descriptor;

struct halfedge2edge
{
  halfedge2edge(const SurfaceMesh& m, std::vector<edge_descriptor>& edges)
    : m_mesh(m), m_edges(edges)
  {}
  void operator()(const halfedge_descriptor& h) const
  {
    m_edges.push_back(edge(h, m_mesh));
  }
  const SurfaceMesh& m_mesh;
  std::vector<edge_descriptor>& m_edges;
};

//-----------------------------------------------------------------------------
stkCGALIsotropicRemeshingFilter::stkCGALIsotropicRemeshingFilter()
{
  this->TargetEdgeLength = 0.0;
  this->TargetEdgeLengthInfo = 0.0;
  this->NumberOfIterations = 1;
  this->ProtectConstraints = 1;
  this->MeshingMaskArrayName = nullptr;
}

//-----------------------------------------------------------------------------
stkCGALIsotropicRemeshingFilter::~stkCGALIsotropicRemeshingFilter()
{
  if (this->MeshingMaskArrayName)
  {
    delete this->MeshingMaskArrayName;
    this->MeshingMaskArrayName = nullptr;
  }
}

//-----------------------------------------------------------------------------
int stkCGALIsotropicRemeshingFilter::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector,
  vtkInformationVector* outputVector)
{
  // Get the input and output data objects.
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  vtkPolyData* input = vtkPolyData::SafeDownCast(
    inInfo->Get(vtkDataObject::DATA_OBJECT()));

  vtkPolyData* output = vtkPolyData::SafeDownCast(
    outInfo->Get(vtkDataObject::DATA_OBJECT()));

  /********************************************
   * Preprocess input
   ********************************************/
  vtkTimerLog::MarkStartEvent("Mesh preprocessing");

  bool masking = false;

  auto innerMesh = vtkSmartPointer<vtkPolyData>::New();
  auto outerMesh = vtkSmartPointer<vtkPolyData>::New();
  if (this->MeshingMaskArrayName &&
    std::strcmp(this->MeshingMaskArrayName, "None") != 0)
  {
    masking = true;

    auto threshold = vtkSmartPointer<vtkThreshold>::New();
    threshold->SetInputData(input);
    threshold->SetInputArrayToProcess(
      0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_CELLS, this->MeshingMaskArrayName);
    threshold->ThresholdByLower(0.0);

    auto surface = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
    surface->SetInputConnection(threshold->GetOutputPort());

    surface->Update();
    outerMesh->DeepCopy(surface->GetOutput());

    threshold->InvertOn();
    surface->Update();
    innerMesh->DeepCopy(surface->GetOutput());
  }
  else
  {
    innerMesh->DeepCopy(input);
  }

  vtkTimerLog::MarkEndEvent("Mesh preprocessing");

  /********************************************
   * Create a SurfaceMesh from the input mesh *
   ********************************************/
  vtkTimerLog::MarkStartEvent("Converting VTK -> CGAL");

  SurfaceMesh mesh;
  VPMap vpmap = get(CGAL::vertex_point, mesh);

  // Get nb of points and cells
  vtkIdType nb_points = innerMesh->GetNumberOfPoints();
  vtkIdType nb_cells = innerMesh->GetNumberOfCells();

  // Extract points
  std::vector<vertex_descriptor> vertex_map(nb_points);
  for (vtkIdType i = 0; i < nb_points; ++i)
  {
    double coords[3];
    innerMesh->GetPoint(i, coords);
    vertex_descriptor v = add_vertex(mesh);
    put(vpmap, v, K::Point_3(coords[0], coords[1], coords[2]));
    vertex_map[i] = v;
  }

  // Extract cells
  for (vtkIdType i = 0; i < nb_cells; ++i)
  {
    vtkCell* cell_ptr = innerMesh->GetCell(i);
    vtkIdType nb_vertices = cell_ptr->GetNumberOfPoints();
    std::vector<vertex_descriptor> vr(nb_vertices);
    for (vtkIdType k = 0; k < nb_vertices; ++k)
    {
      vr[k] = vertex_map[cell_ptr->GetPointId(k)];
    }
    CGAL::Euler::add_face(vr, mesh);
  }

  std::vector<vertex_descriptor> isolated_vertices;
  for(SurfaceMesh::vertex_iterator vit = mesh.vertices_begin();
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
  if (!masking && this->ProtectConstraints)
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

  // When masking is used, we want to preserve the border topology
  if (masking || this->ProtectConstraints)
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

  vtkTimerLog::MarkEndEvent("Converting CGAL -> VTK");

  if (masking)
  {
    auto remeshed = vtkSmartPointer<vtkPolyData>::New();
    remeshed->SetPoints(vtk_points);
    remeshed->SetPolys(vtk_cells);
    remeshed->Squeeze();

    auto append = vtkSmartPointer<vtkAppendPolyData>::New();
    append->AddInputData(outerMesh);
    append->AddInputData(remeshed);

    auto clean = vtkSmartPointer<vtkCleanPolyData>::New();
    clean->SetInputConnection(append->GetOutputPort());
    clean->Update();

    output->DeepCopy(clean->GetOutput());
  }
  else
  {
    output->SetPoints(vtk_points);
    output->SetPolys(vtk_cells);
    output->Squeeze();
  }

  return 1;
}

//-----------------------------------------------------------------------------
void stkCGALIsotropicRemeshingFilter::PrintSelf(
  std::ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  os << indent << "TargetEdgeLength     : " << this->TargetEdgeLength << std::endl;
  os << indent << "TargetEdgeLengthInfo : " << this->TargetEdgeLengthInfo << std::endl;
  os << indent << "NumberOfIterations   : " << this->NumberOfIterations << std::endl;
  os << indent << "ProtectConstraints   : " << this->ProtectConstraints << std::endl;
  os << indent << "MeshingMaskArrayName : "
    << (this->MeshingMaskArrayName ? this->MeshingMaskArrayName : "(none)") << std::endl;
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
int stkCGALIsotropicRemeshingFilter::RequestInformation(
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
