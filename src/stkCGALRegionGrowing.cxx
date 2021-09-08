#include <stkCGALRegionGrowing.h>

// -- VTK
#include <vtkCellData.h>
#include <vtkFieldData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkTimerLog.h>

// -- CGAL
#include <CGAL/Cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_kth_root.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_root_of.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Iterator_range.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/memory.h>
//#include <CGAL/Homogeneous.h>
//#include <CGAL/Simple_homogeneous.h>

#include <CGAL/Surface_mesh.h>

#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_polygon_mesh.h>

// -- STL
#include <cstdlib>
#include <iterator>
#include <vector>

//----------------------------------------------------------------------------
vtkStandardNewMacro(stkCGALRegionGrowing);

//----------------------------------------------------------------------------
int stkCGALRegionGrowing::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  // Get the info objects
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  // Get the input and output
  vtkPolyData* input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkPolyData* output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  // Copy input data to the output
  output->CopyStructure(input);
  output->GetCellData()->PassData(input->GetCellData());
  output->GetFieldData()->PassData(input->GetFieldData());
  output->GetPointData()->PassData(input->GetPointData());

  switch (this->KernelValue)
  {
    case KernelEnum::EPEC:
    {
      return this->Detection<CGAL::Exact_predicates_exact_constructions_kernel>(input, output);
    }
    case KernelEnum::EPEC_SQRT:
    {
      return this->Detection<CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt>(
        input, output);
    }
    case KernelEnum::EPEC_KTH_ROOT:
    {
      return this->Detection<CGAL::Exact_predicates_exact_constructions_kernel_with_kth_root>(
        input, output);
    }
    case KernelEnum::EPEC_ROOT_OF:
    {
      return this->Detection<CGAL::Exact_predicates_exact_constructions_kernel_with_root_of>(
        input, output);
    }
    case KernelEnum::EPIC:
    {
      return this->Detection<CGAL::Exact_predicates_inexact_constructions_kernel>(input, output);
    }
    case KernelEnum::Cartesian:
    {
      return this->Detection<CGAL::Cartesian<double> >(input, output);
    }
    case KernelEnum::Simple_cartesian:
    {
      return this->Detection<CGAL::Simple_cartesian<double> >(input, output);
    }
    /*
    case KernelEnum::Homogeneous:
    {
      return this->Detection<CGAL::Homogeneous<double>>(input, output);
    }
    case KernelEnum::Simple_homogeneous:
    {
      return this->Detection<CGAL::Simple_homogeneous<double>>(input, output);
    }
    */
    default:
    {
      vtkErrorMacro("Wrong kernel value.");
    }
  }

  return 1;
}

//----------------------------------------------------------------------------
/** @brief Proceed to detect the regions contained within the input dataset.
 *
 *  @param input       source data structure
 *  @param output      copy of the input data with relevant data arrays appended
 *  @tparam CGalKernel must be a CGAL kernel compatible type
 *
 *  @return int Success (1) or failure (0)
 */
template<class CGalKernel>
int stkCGALRegionGrowing::Detection(vtkPolyData* input, vtkPolyData* output)
{
  using FT = typename CGalKernel::FT;
  using Point_3 = typename CGalKernel::Point_3;

  using Polygon_mesh = CGAL::Surface_mesh<Point_3>;
  using Face_range = typename Polygon_mesh::Face_range;

  using Neighbor_query = CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<Polygon_mesh>;
  using Region_type =
    CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<CGalKernel, Polygon_mesh>;
  using Sorting = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<CGalKernel,
    Polygon_mesh, Neighbor_query>;

  using Region = std::vector<std::size_t>;
  using Regions = std::vector<Region>;

  using Vertex_to_point_map = typename Region_type::Vertex_to_point_map;

  using Region_growing = CGAL::Shape_detection::Region_growing<Face_range, Neighbor_query,
    Region_type, typename Sorting::Seed_map>;

  Polygon_mesh polygon_mesh;
  stkCGALRegionGrowing::vtkPolyDataToPolygonMesh<Polygon_mesh>(input, polygon_mesh);

  const Face_range face_range = faces(polygon_mesh);

  // Default parameter values for the data file polygon_mesh.off.
  const FT max_distance_to_plane = FT(this->MaxDistanceToPlane);
  const FT max_accepted_angle = FT(this->MaxAcceptedAngle);
  const std::size_t min_region_size = this->MinRegionSize;

  // Create instances of the classes Neighbor_query and Region_type.
  Neighbor_query neighbor_query(polygon_mesh);

  vtkTimerLog::MarkStartEvent("Sorting");

  const Vertex_to_point_map vertex_to_point_map(get(CGAL::vertex_point, polygon_mesh));

  Region_type region_type(
    polygon_mesh, max_distance_to_plane, max_accepted_angle, min_region_size, vertex_to_point_map);

  // Sort face indices.
  Sorting sorting(polygon_mesh, neighbor_query, vertex_to_point_map);
  sorting.sort();

  vtkTimerLog::MarkEndEvent("Sorting");

  vtkTimerLog::MarkStartEvent("Detection");

  // Create an instance of the region growing class.
  Region_growing region_growing(face_range, neighbor_query, region_type, sorting.seed_map());

  // Run the algorithm.
  Regions regions;
  region_growing.detect(std::back_inserter(regions));

  Region unassigned_cells;
  region_growing.unassigned_items(std::back_inserter(unassigned_cells));

  // Print number of detected shapes and algorithm coverage
  vtkWarningMacro(<< regions.end() - regions.begin() << " shapes detected "
                  << "|" << unassigned_cells.size() << " unassigned cells "
                  << "|"
                  << "Algorithm Coverage: "
                  << ((input->GetNumberOfCells() - unassigned_cells.size()) /
                       static_cast<double>(input->GetNumberOfCells()) * 100.00)
                  << " %");

  vtkTimerLog::MarkEndEvent("Detection");

  vtkIdType numPolys = input->GetNumberOfPolys();
  auto regionsArray = vtkSmartPointer<vtkIntArray>::New();
  regionsArray->SetName("Regions");
  regionsArray->SetNumberOfComponents(1);
  regionsArray->SetNumberOfTuples(numPolys);
  regionsArray->Fill(-1);

  int regionIndex = 0;
  for (const auto& region : regions)
  {
    // Iterate through all region items.
    for (const auto index : region)
    {
      regionsArray->SetValue(static_cast<vtkIdType>(index), regionIndex);
    }
    regionIndex++;
  }

  output->GetCellData()->AddArray(regionsArray);

  return 1;
}

//----------------------------------------------------------------------------
/** @brief Convert a polydata to a CGAL compatible data structure
 *
 *  @param polyData  input data structure
 *  @param tmesh     output data structure
 *  @tparam MeshType must be a CGAL surface mesh
 *
 *  @return bool Success (true) or failure (false)
 */
template<typename MeshType>
bool stkCGALRegionGrowing::vtkPolyDataToPolygonMesh(vtkPolyData* polyData, MeshType& tmesh)
{
  typedef typename boost::property_map<MeshType, CGAL::vertex_point_t>::type VPMap;
  typedef typename boost::property_map_value<MeshType, CGAL::vertex_point_t>::type Point;
  typedef typename boost::graph_traits<MeshType>::vertex_descriptor VertexDescriptor;

  VPMap vpmap = get(CGAL::vertex_point, tmesh);

  // get nb of points and cells
  vtkIdType nb_points = polyData->GetNumberOfPoints();
  vtkIdType nb_cells = polyData->GetNumberOfCells();

  // extract points
  std::vector<VertexDescriptor> vertex_map(nb_points);
  for (vtkIdType i = 0; i < nb_points; ++i)
  {
    double coords[3];
    polyData->GetPoint(i, coords);

    VertexDescriptor v = add_vertex(tmesh);
    put(vpmap, v, Point(coords[0], coords[1], coords[2]));
    vertex_map[i] = v;
  }

  // extract cells
  for (vtkIdType i = 0; i < nb_cells; ++i)
  {
    if (polyData->GetCellType(i) != 5 && polyData->GetCellType(i) != 7 &&
      polyData->GetCellType(i) != 9) // only supported cells are triangles, quads and polygons
      continue;
    vtkCell* cell_ptr = polyData->GetCell(i);

    vtkIdType nb_vertices = cell_ptr->GetNumberOfPoints();
    if (nb_vertices < 3)
      return false;
    std::vector<VertexDescriptor> vr(nb_vertices);
    for (vtkIdType k = 0; k < nb_vertices; ++k)
      vr[k] = vertex_map[cell_ptr->GetPointId(k)];

    CGAL::Euler::add_face(vr, tmesh);
  }

  return true;
}
