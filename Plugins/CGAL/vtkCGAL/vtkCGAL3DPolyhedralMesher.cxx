/**
* \class vtkCGAL3DPolyhedralMesher
*
* \brief Generates a polyhedral mesh of the bounding domain based on features related to the interior surfaces.
*        ODT and Lloyd optimization methods are supported. Exude and perturbe options are also supported, but can
*        have an impact on the refinement criteria. See the official CGAL documentation for more information.
*        
*        Inputs: Interior Surfaces (port 0, vtkPolyData), Bounding Domain (port 1, vtkPolyData)
*        Output: Polyhedral Domain with features (port 0, vtkUnstructuredGrid)
*/

#include "vtkCGAL3DPolyhedralMesher.h"

// VTK
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkUnstructuredGrid.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkProbeFilter.h>
#include <vtkStaticPointLocator.h>

#include <vtkTimerLog.h>

#include <vtkPointCloudScalarSizingField.h>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_polyhedron_3.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Mesh_constant_domain_field_3.h>
#include <CGAL/Labeled_mesh_domain_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/refine_mesh_3.h>
#include <CGAL/IO/Complex_3_in_triangulation_3_to_vtk.h>

#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

vtkStandardNewMacro(vtkCGAL3DPolyhedralMesher);

// ----------------------------------------------------------------------------
vtkCGAL3DPolyhedralMesher::vtkCGAL3DPolyhedralMesher()
{
  this->SetNumberOfInputPorts(3);
  this->SetNumberOfOutputPorts(1);
  
  // Mesh Criteria
  this->EdgeSize = 0.025;
  this->FacetAngle = 25;
  this->FacetSize = 0.05;
  this->FacetDistance = 0.005;
  this->CellRadiusEdgeRatio = 3;
  this->CellSize = 0.05;

  // Constraint flags
  this->TopologicalStructure = vtkCGAL3DPolyhedralMesher::TopologicalStructures::MANIFOLD;
  this->ConfineSurfacePoints = true;
  this->UseCustomSizingField = false;
  this->CustomSizingFieldArrayName = "";

  // Optimizer flags
  this->Lloyd = true;
  this->Odt = true;
  this->Perturb = true;
  this->Exude = true;
}

//----------------------------------------------------------------------------
void vtkCGAL3DPolyhedralMesher::PrintSelf(ostream& os, vtkIndent indent)
{
  // TODO: Write PrintSelf
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------

/** @brief Getter for interior surface
*
*  @return vtkPolyData* The interior surface features
*/
vtkPolyData* vtkCGAL3DPolyhedralMesher::GetInteriorSurfaces()
{
  if (this->GetNumberOfInputConnections(0) < 1) {
    return nullptr;
  }

  return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

//----------------------------------------------------------------------------

/** @brief Getter for Bounding Domain
*
*  @return vtkPolyData*
*/
vtkPolyData* vtkCGAL3DPolyhedralMesher::GetBoundingDomain()
{
  if (this->GetNumberOfInputConnections(1) < 1) {
    return nullptr;
  }

  return vtkPolyData::SafeDownCast(this->GetInputDataObject(1, 0));
}

//----------------------------------------------------------------------------

/** @brief Getter for Sizing Field
*
*  @return vtkPointSet*
*/
vtkPointSet* vtkCGAL3DPolyhedralMesher::GetSizingField()
{
    if (this->GetNumberOfInputConnections(2) < 1) {
        return nullptr;
    }

    return vtkPointSet::SafeDownCast(this->GetInputDataObject(2, 0));
}

//----------------------------------------------------------------------------

int vtkCGAL3DPolyhedralMesher::RequestData(vtkInformation* vtkNotUsed(request),
                      vtkInformationVector** inputVector,
                      vtkInformationVector* outputVector)
{
  vtkPolyData* inputInteriorSurfaces = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));
  vtkPolyData* inputBoundingDomain = vtkPolyData::GetData(inputVector[1]->GetInformationObject(0));
  vtkPointSet* inputSizingField = nullptr;
  if (this->GetNumberOfInputConnections(2) > 0)
    inputSizingField = vtkPointSet::GetData(inputVector[2]->GetInformationObject(0));

  vtkUnstructuredGrid* output = vtkUnstructuredGrid::GetData(outputVector->GetInformationObject(0));
    
  if (inputInteriorSurfaces == nullptr)
  {
    vtkErrorMacro("Interior Surfaces input is null.");
    return 0;
  }

  if (inputInteriorSurfaces->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("Interior Surfaces input does not contain any points.");
    return 0;
  }

  if (inputInteriorSurfaces->GetPolys() == nullptr)
  {
    vtkErrorMacro("Interior Surfaces input does not contain any cell structure.");
    return 0;
  }

  if (inputBoundingDomain == nullptr)
  {
    vtkErrorMacro("Bounding Domain input is null.");
    return 0;
  }

  if (inputBoundingDomain->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("Bounding Domain input does not contain any points.");
    return 0;
  }

  if (inputBoundingDomain->GetPolys() == nullptr)
  {
    vtkErrorMacro("Bounding Domain input does not cell structure.");
    return 0;
  }
  
  // Check for sizing field
  vtkDataArray* sizingFieldArray = nullptr;

  if (this->UseCustomSizingField)
  {
    if (inputSizingField == nullptr)
    {
      vtkErrorMacro("Cannot use custom sizing field if sizing field input is undefined.");
      return 0;
    }

    vtkAbstractArray* sizingFieldAbstractArray = inputSizingField->GetPointData()->GetAbstractArray(this->CustomSizingFieldArrayName.c_str());
    if (sizingFieldAbstractArray == nullptr)
    {
      vtkErrorMacro("Sizing field array not found.");
      return 0;
    }

    sizingFieldArray = vtkArrayDownCast<vtkDataArray>(sizingFieldAbstractArray);
    if (sizingFieldArray == nullptr)
    {
      vtkErrorMacro("Sizing Field array does not contain numeric data.");
      return 0;
    }

    if (sizingFieldArray->GetNumberOfComponents() != 1)
    {
      vtkErrorMacro("Sizing Field array must only have 1 component.");
      return 0;
    }
  }

  // Start
  this->SetProgressText("CGAL Polyhedral Mesher");
  this->UpdateProgress(0);

  // Convert from VTK to CGAL
  vtkTimerLog::MarkStartEvent("VTK To CGAL conversion");

  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Mesh_polyhedron_3<K>::type Polyhedron;
  Polyhedron* interiorSurfaces = new Polyhedron();
  Polyhedron* boundingDomain = new Polyhedron();
  this->vtkPolyDataToPolygonMesh(inputInteriorSurfaces, *interiorSurfaces);
  this->vtkPolyDataToPolygonMesh(inputBoundingDomain, *boundingDomain);
  
  vtkTimerLog::MarkEndEvent("VTK To CGAL conversion");
  this->UpdateProgress(0.1);

  // Generate mesh
  vtkTimerLog::MarkStartEvent("Mesh Generation");

  // Domain
  typedef CGAL::Polyhedral_mesh_domain_with_features_3<K> Mesh_domain;
  Mesh_domain domain(*interiorSurfaces, *boundingDomain);

  // Sizing Field
  stk::PointCloudScalarSizingField size;


  if (this->UseCustomSizingField)
  {
    //vtkNew<vtkPointLocator> pointLocator;
    //pointLocator->SetDataSet(inputSizingField);
    //pointLocator->BuildLocator();

    vtkDataArray* sizingFieldArray = vtkArrayDownCast<vtkDataArray>(inputSizingField->GetPointData()->GetAbstractArray(this->CustomSizingFieldArrayName.c_str()));

    size.SetUseDefaultCellSize(false);
    //size.SetPointLocator(pointLocator);
    size.SetPointCloud(inputSizingField);
    size.SetSizingFieldArray(sizingFieldArray);
  }
  else
  {
    size.SetUseDefaultCellSize(true);
    size.SetDefaultCellSize(this->CellSize);
  }

  // Criteria
  typedef CGAL::Mesh_triangulation_3<Mesh_domain, CGAL::Default, Concurrency_tag>::type Tr;
  typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;

  Mesh_criteria criteria( CGAL::parameters::edge_size = this->EdgeSize,
              CGAL::parameters::facet_angle = this->FacetAngle,
              CGAL::parameters::facet_size = this->FacetSize,
              CGAL::parameters::facet_distance = this->FacetDistance,
              CGAL::parameters::cell_radius_edge_ratio = this->CellRadiusEdgeRatio,
              CGAL::parameters::cell_size = size);


  // C3t3 init
  typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr, Mesh_domain::Corner_index, Mesh_domain::Curve_index> C3t3;
  C3t3 c3t3;
  
  if (this->ConfineSurfacePoints)
  {
    CGAL::Mesh_3::internal::init_c3t3_with_features(c3t3, domain, criteria);
    typedef C3t3::Triangulation::Point Weighted_point;
    double pt[3] = { 0.0, 0.0, 0.0 };
    for (vtkIdType i = 0; i < inputInteriorSurfaces->GetNumberOfPoints(); ++i)
    {
      inputInteriorSurfaces->GetPoint(i, pt);
      const Weighted_point p(Weighted_point::Point(pt[0], pt[1], pt[2]));
      Tr::Vertex_handle vh = c3t3.triangulation().insert(p);
      c3t3.add_to_complex(vh, 0);
    }
  }
  else
  {
    domain.detect_features();
    c3t3 = CGAL::make_mesh_3<C3t3, Mesh_domain, Mesh_criteria>(domain, criteria);
  }

  CGAL::parameters::internal::Manifold_options manifoldOption;
  switch (this->TopologicalStructure) {
  case vtkCGAL3DPolyhedralMesher::TopologicalStructures::MANIFOLD:
  {
    manifoldOption = CGAL::parameters::manifold();
    break;
  }
  case vtkCGAL3DPolyhedralMesher::TopologicalStructures::MANIFOLD_WITH_BOUNDARY:
  {
    manifoldOption = CGAL::parameters::manifold_with_boundary();
    break;
  }
  case vtkCGAL3DPolyhedralMesher::TopologicalStructures::NON_MANIFOLD:
  {
    manifoldOption = CGAL::parameters::non_manifold();
    break;
  }
  default:
  {
    vtkErrorMacro("Unknown option.");
    return 0;
  }
  }

  CGAL::refine_mesh_3<C3t3, Mesh_domain, Mesh_criteria>(c3t3, domain, criteria,
            CGAL::parameters::internal::Lloyd_options(this->Lloyd),
            CGAL::parameters::internal::Odt_options(this->Odt),
            CGAL::parameters::internal::Perturb_options(this->Perturb), 
            CGAL::parameters::internal::Exude_options(this->Exude),
            CGAL::parameters::internal::Manifold_options(manifoldOption)
  );

  vtkTimerLog::MarkEndEvent("Mesh Generation");

  // Convert back to VTK
  vtkTimerLog::MarkStartEvent("CGAL to VTK conversion");
  vtkNew<vtkUnstructuredGrid> result;
  CGAL::output_c3t3_to_vtk_unstructured_grid(c3t3, result);
  output->ShallowCopy(result);
  vtkTimerLog::MarkEndEvent("CGAL to VTK conversion");

  // Cleaning
  delete interiorSurfaces;
  delete boundingDomain;
  interiorSurfaces = nullptr;
  boundingDomain = nullptr;

  return 1;
}

//----------------------------------------------------------------------------
int vtkCGAL3DPolyhedralMesher::FillInputPortInformation(int port, vtkInformation* info)
{
    if (port == 0 || port == 1)
    {
        info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPolyData");
    }
    else if (port == 2)
    {
        info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPointSet");
        info->Set(vtkAlgorithm::INPUT_IS_OPTIONAL(), 1);
    }

    return 1;
}


// ------------------------------------------------------------------------------
int vtkCGAL3DPolyhedralMesher::FillOutputPortInformation(int, vtkInformation *info)
{
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkUnstructuredGrid");
  return 1;
}

//----------------------------------------------------------------------------

/** @brief Converts a vtkPolyData (VTK) into a Polygonal Mesh (CGAL). Code taken from the
*          Polyhedron demo located at https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/IO/VTK_io_plugin.cpp
*          This method does not write into our PolyData structure. Hence, we do not need to copy them.
*
*  @param polyData The input PolyData
*  @param tmesh The resulting Polygon Mesh
*  @return bool Success (true) or failure (false)
*/
template <typename TM>
bool vtkCGAL3DPolyhedralMesher::vtkPolyDataToPolygonMesh(vtkPointSet* polyData, TM& tmesh)
{
  typedef typename boost::property_map<TM, CGAL::vertex_point_t>::type VPMap;
  typedef typename boost::property_map_value<TM, CGAL::vertex_point_t>::type Point_3;
  typedef typename boost::graph_traits<TM>::vertex_descriptor vertex_descriptor;

  VPMap vpmap = get(CGAL::vertex_point, tmesh);

  // get nb of points and cells
  vtkIdType nb_points = polyData->GetNumberOfPoints();
  vtkIdType nb_cells = polyData->GetNumberOfCells();

  //extract points
  std::vector<vertex_descriptor> vertex_map(nb_points);
  for (vtkIdType i = 0; i < nb_points; ++i)
  {
    double coords[3];
    polyData->GetPoint(i, coords);

    vertex_descriptor v = add_vertex(tmesh);
    put(vpmap, v, Point_3(coords[0], coords[1], coords[2]));
    vertex_map[i] = v;
  }

  //extract cells
  for (vtkIdType i = 0; i < nb_cells; ++i)
  {
    if (polyData->GetCellType(i) != 5
      && polyData->GetCellType(i) != 7
      && polyData->GetCellType(i) != 9) //only supported cells are triangles, quads and polygons
      continue;
    vtkCell* cell_ptr = polyData->GetCell(i);

    vtkIdType nb_vertices = cell_ptr->GetNumberOfPoints();
    if (nb_vertices < 3)
      return false;
    std::vector<vertex_descriptor> vr(nb_vertices);
    for (vtkIdType k = 0; k < nb_vertices; ++k)
      vr[k] = vertex_map[cell_ptr->GetPointId(k)];

    CGAL::Euler::add_face(vr, tmesh);
  }

  return true;
}
