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
#include <vtkCGALUtilities.h>
#include <vtkFieldData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>

// -- CGAL
#include <CGAL/memory.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Iterator_range.h>
#include <CGAL/HalfedgeDS_vector.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_polygon_mesh.h>

// -- STL
#include <vector>
#include <cstdlib>
#include <iostream>
#include <iterator>

// Type declarations.
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;

using FT = typename Kernel::FT;
using Point_3 = typename Kernel::Point_3;

using Color = CGAL::Color;

// Choose the type of a container for a polygon mesh.
#define USE_SURFACE_MESH

#if defined(USE_SURFACE_MESH)
  using Polygon_mesh = CGAL::Surface_mesh<Point_3>;
  using Face_range = typename Polygon_mesh::Face_range;

  using Neighbor_query =
    CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<Polygon_mesh>;
  using Region_type =
    CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<Kernel, Polygon_mesh>;
  using Sorting =
    CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<Kernel, Polygon_mesh, Neighbor_query>;
#else
  using Polygon_mesh =
    CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_3, CGAL::HalfedgeDS_vector>;
  using Face_range =
    typename CGAL::Iterator_range<typename boost::graph_traits<Polygon_mesh>::face_iterator>;

  using Neighbor_query =
    CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<Polygon_mesh, Face_range>;
  using Region_type =
    CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<Kernel, Polygon_mesh, Face_range>;
  using Sorting =
    CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<Kernel, Polygon_mesh, Neighbor_query, Face_range>;
#endif

using Region = std::vector<std::size_t>;
using Regions = std::vector<Region>;

using Vertex_to_point_map = typename Region_type::Vertex_to_point_map;

using Region_growing = CGAL::Shape_detection::Region_growing<Face_range, Neighbor_query, Region_type, typename Sorting::Seed_map>;

// -----------------------------------------------------------------------------
// Declare the plugin
vtkStandardNewMacro(vtkCGALRegionGrowing);

// -----------------------------------------------------------------------------
// Constructor
// Todo.
vtkCGALRegionGrowing::vtkCGALRegionGrowing()
{
  this->MaxDistanceToPlane = 1;
  this->MaxAcceptedAngle = 45;
  this->MinRegionSize = 5;
}

// -----------------------------------------------------------------------------
// Todo.
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

  output->CopyStructure(input);
  output->GetCellData()->PassData(input->GetCellData());
  output->GetFieldData()->PassData(input->GetFieldData());
  output->GetPointData()->PassData(input->GetPointData());

  Polygon_mesh polygon_mesh;
  vtkCGALUtilities::vtkPolyDataToPolygonMesh(input, polygon_mesh);

  std::cout << std::endl <<
    "region_growing_on_polygon_mesh example started with "
    << std::endl << "\tMaxDistanceToPlane" << "\t" << this->MaxDistanceToPlane
    << std::endl << "\tMaxAcceptedAngle" << "\t" << this->MaxAcceptedAngle
    << std::endl << "\tMinRegionSize" << "\t" << this->MinRegionSize
    << std::endl << std::endl;

  const Face_range face_range = faces(polygon_mesh);
  std::cout << "* polygon mesh with "
    << face_range.size() << " faces is loaded" << std::endl;

  // Default parameter values for the data file polygon_mesh.off.
  const FT          max_distance_to_plane = FT(this->MaxDistanceToPlane);
  const FT          max_accepted_angle = FT(this->MaxAcceptedAngle);
  const std::size_t min_region_size = this->MinRegionSize;

  // Create instances of the classes Neighbor_query and Region_type.
  Neighbor_query neighbor_query(polygon_mesh);

  const Vertex_to_point_map vertex_to_point_map(
    get(CGAL::vertex_point, polygon_mesh));

  Region_type region_type(
    polygon_mesh,
    max_distance_to_plane, max_accepted_angle, min_region_size,
    vertex_to_point_map);

  // Sort face indices.
  Sorting sorting(
    polygon_mesh, neighbor_query,
    vertex_to_point_map);
  sorting.sort();

  // Create an instance of the region growing class.
  Region_growing region_growing(
    face_range, neighbor_query, region_type,
    sorting.seed_map());

  // Run the algorithm.
  Regions regions;
  region_growing.detect(std::back_inserter(regions));

  // Print the number of found regions.
  std::cout << "* " << regions.size()
    << " regions have been found" << std::endl;

  vtkIdType numPolys = input->GetNumberOfPolys();

  auto colors = vtkIntArray::New();
  colors->SetName("regions");
  colors->SetNumberOfComponents(1);
  colors->SetNumberOfTuples(numPolys);
  colors->Fill(-1);

#if defined(USE_SURFACE_MESH)
  int regionIndex = 0;

  for (const auto& region : regions)
  {
    // Iterate through all region items.
    for (const auto index : region)
    {
      colors->SetValue(
        static_cast<vtkIdType>(index), regionIndex);
    }

    regionIndex++;
  }
#endif // USE_SURFACE_MESH

  std::cout
    << std::endl << "region_growing_on_polygon_mesh example finished"
    << std::endl << std::endl;

  output->GetCellData()->SetScalars(colors);
  colors->Delete();

  return 1;
}
