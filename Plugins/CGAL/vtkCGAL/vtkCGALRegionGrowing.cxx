/**
* \class vtkCGALRegionGrowing
* 
* \brief todo
* 
* 
* 
* Inputs: todo
* Output: todo
* 
*/

#include <vtkCGALRegionGrowing.h>

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
#include <CGAL/memory.h>
#include <CGAL/Iterator_range.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_kth_root.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_root_of.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Simple_cartesian.h>
//#include <CGAL/Homogeneous.h>
//#include <CGAL/Simple_homogeneous.h>

#include <CGAL/Surface_mesh.h>

#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_polygon_mesh.h>

// -- STL
#include <vector>
#include <cstdlib>
#include <iterator>

// -----------------------------------------------------------------------------
// Declare the plugin
vtkStandardNewMacro(vtkCGALRegionGrowing);

// -----------------------------------------------------------------------------
// Constructor
// TODO: description
vtkCGALRegionGrowing::vtkCGALRegionGrowing()
{
  this->KernelValue = vtkCGALRegionGrowing::EPEC;
  this->MaxDistanceToPlane = 1.0;
  this->MaxAcceptedAngle = 45.0;
  this->MinRegionSize = 5;
}

// -----------------------------------------------------------------------------
// TODO: description
int vtkCGALRegionGrowing::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // Get the info objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);

  // Get the input and output
  vtkPolyData *input = vtkPolyData::SafeDownCast(
    inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkPolyData *output = vtkPolyData::SafeDownCast(
    outInfo->Get(vtkDataObject::DATA_OBJECT()));

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
      return this->Detection<CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt>(input, output);
    }
    case KernelEnum::EPEC_ROOT:
    {
      return this->Detection<CGAL::Exact_predicates_exact_constructions_kernel_with_kth_root>(input, output);
    }
    case KernelEnum::EPEC_ROOT_OF:
    {
      return this->Detection<CGAL::Exact_predicates_exact_constructions_kernel_with_root_of>(input, output);
    }
    case KernelEnum::EPIC:
    {
      return this->Detection<CGAL::Exact_predicates_inexact_constructions_kernel>(input, output);
    }
    case KernelEnum::Cartesian:
    {
      return this->Detection<CGAL::Cartesian<double>>(input, output);
    }
    case KernelEnum::Simple_cartesian:
    {
      return this->Detection<CGAL::Simple_cartesian<double>>(input, output);
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

template <class CGalKernel>
int vtkCGALRegionGrowing::Detection(vtkPolyData* input, vtkPolyData* output)
{
  using FT = typename CGalKernel::FT;
  using Point_3 = typename CGalKernel::Point_3;

  using Polygon_mesh = CGAL::Surface_mesh<Point_3>;
  using Face_range = typename Polygon_mesh::Face_range;

  using Neighbor_query =
    CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<Polygon_mesh>;
  using Region_type =
    CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<CGalKernel, Polygon_mesh>;
  using Sorting =
    CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<CGalKernel, Polygon_mesh, Neighbor_query>;

  using Region = std::vector<std::size_t>;
  using Regions = std::vector<Region>;

  using Vertex_to_point_map = typename Region_type::Vertex_to_point_map;

  using Region_growing =
    CGAL::Shape_detection::Region_growing<Face_range, Neighbor_query, Region_type, typename Sorting::Seed_map>;

  Polygon_mesh polygon_mesh;
  vtkCGALRegionGrowing::vtkPolyDataToPolygonMesh<Polygon_mesh>(input, polygon_mesh);

  const Face_range face_range = faces(polygon_mesh);

  // Default parameter values for the data file polygon_mesh.off.
  const FT          max_distance_to_plane = FT(this->MaxDistanceToPlane);
  const FT          max_accepted_angle = FT(this->MaxAcceptedAngle);
  const std::size_t min_region_size = this->MinRegionSize;

  // Create instances of the classes Neighbor_query and Region_type.
  Neighbor_query neighbor_query(polygon_mesh);

  vtkTimerLog::MarkStartEvent("Sorting");

  const Vertex_to_point_map vertex_to_point_map(
    get(CGAL::vertex_point, polygon_mesh));

  Region_type region_type(polygon_mesh,
    max_distance_to_plane, max_accepted_angle, min_region_size,
    vertex_to_point_map);

  // Sort face indices.
  Sorting sorting(
    polygon_mesh, neighbor_query, vertex_to_point_map);
  sorting.sort();

  vtkTimerLog::MarkEndEvent("Sorting");

  vtkTimerLog::MarkStartEvent("Detection");

  // Create an instance of the region growing class.
  Region_growing region_growing(
    face_range, neighbor_query, region_type, sorting.seed_map());

  // Run the algorithm.
  Regions regions;
  region_growing.detect(std::back_inserter(regions));

  vtkTimerLog::MarkEndEvent("Detection");

  vtkIdType numPolys = input->GetNumberOfPolys();
  auto regionsArray = vtkIntArray::New();
  regionsArray->SetName("regions");
  regionsArray->SetNumberOfComponents(1);
  regionsArray->SetNumberOfTuples(numPolys);
  regionsArray->Fill(-1);

  int regionIndex = 0;
  for (const auto& region : regions)
  {
    // Iterate through all region items.
    for (const auto index : region)
    {
      regionsArray->SetValue(
        static_cast<vtkIdType>(index), regionIndex);
    }
    regionIndex++;
  }

  output->GetCellData()->SetScalars(regionsArray);
  regionsArray->Delete();

  return 1;
}

template <typename MeshType>
bool vtkCGALRegionGrowing::vtkPolyDataToPolygonMesh(vtkPolyData* poly_data, MeshType& tmesh)
{
  typedef typename boost::property_map<MeshType, CGAL::vertex_point_t>::type VPMap;
  typedef typename boost::property_map_value<MeshType, CGAL::vertex_point_t>::type Point;
  typedef typename boost::graph_traits<MeshType>::vertex_descriptor VertexDescriptor;

  VPMap vpmap = get(CGAL::vertex_point, tmesh);

  // get nb of points and cells
  vtkIdType nb_points = poly_data->GetNumberOfPoints();
  vtkIdType nb_cells = poly_data->GetNumberOfCells();

  //extract points
  std::vector<VertexDescriptor> vertex_map(nb_points);
  for (vtkIdType i = 0; i < nb_points; ++i)
  {
    double coords[3];
    poly_data->GetPoint(i, coords);

    VertexDescriptor v = add_vertex(tmesh);
    put(vpmap, v, Point(coords[0], coords[1], coords[2]));
    vertex_map[i] = v;
  }

  //extract cells
  for (vtkIdType i = 0; i < nb_cells; ++i)
  {
    if (poly_data->GetCellType(i) != 5
      && poly_data->GetCellType(i) != 7
      && poly_data->GetCellType(i) != 9) //only supported cells are triangles, quads and polygons
      continue;
    vtkCell* cell_ptr = poly_data->GetCell(i);

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
