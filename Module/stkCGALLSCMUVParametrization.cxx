#include "stkCGALLSCMUVParametrization.h"

//---------VTK----------------------------------
#include <vtkCommand.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkObjectFactory.h>
#include <vtkStreamingDemandDrivenPipeline.h>

#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

//---------CGAL---------------------------------
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_parameterization/Error_code.h>
#include <CGAL/Surface_mesh_parameterization/LSCM_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>

//---------Module-------------------------------
#include <stkCGALUVParametrizationUtilities.h>
#include <stkCGALUtilities.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Kernel::Point_3> Surface_Mesh;

typedef boost::graph_traits<Surface_Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Surface_Mesh>::halfedge_descriptor halfedge_descriptor;

typedef Surface_Mesh::Property_map<vertex_descriptor, Point_2> UV_pmap;

namespace SMP = CGAL::Surface_mesh_parameterization;

vtkStandardNewMacro(stkCGALLSCMUVParametrization);

//---------------------------------------------------
int stkCGALLSCMUVParametrization::RequestData(vtkInformation*,
  vtkInformationVector** vtkNotUsed(inputVector), vtkInformationVector* outputVector)
{
  //  Get the input and output data objects.
  //  Get the info objects
  vtkPolyData* inputMesh = this->GetInputMesh();

  if (inputMesh == nullptr || inputMesh->GetNumberOfPoints() == 0)
  {
    vtkErrorMacro("No points found in input mesh.");
    return 0;
  }

  vtkPolyData* output = vtkPolyData::GetData(outputVector->GetInformationObject(0));

  Surface_Mesh surfaceMesh;
  if (!stkCGALUtilities::vtkPolyDataToPolygonMesh(inputMesh, surfaceMesh))
  {
    vtkErrorMacro("Failed to read input mesh.");
    return 0;
  }

  if (!CGAL::is_triangle_mesh(surfaceMesh))
  {
    vtkErrorMacro("Mesh is not triangular.");
    return 0;
  }

  // a halfedge on the border
  halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(
    surfaceMesh, CGAL::Polygon_mesh_processing::parameters::all_default())
                              .first;

  UV_pmap uv_map = surfaceMesh.add_property_map<vertex_descriptor, Point_2>("h:uv").first;

  typedef SMP::Two_vertices_parameterizer_3<Surface_Mesh> Border_Parameterizer;
  typedef SMP::LSCM_parameterizer_3<Surface_Mesh, Border_Parameterizer> Parameterizer;

  SMP::Error_code err = SMP::parameterize(surfaceMesh, Parameterizer(), bhd, uv_map);

  if (err != SMP::OK)
  {
    vtkErrorMacro("Error: " << SMP::get_error_message(err));
    return 0;
  }

  vtkNew<vtkPolyData> out;
  out->DeepCopy(inputMesh);

  if (!UpdatePointCoordinates<Surface_Mesh, UV_pmap>(surfaceMesh, uv_map, out))
  {
    vtkErrorMacro("Failed to update point coordinates.");
    return 0;
  }

  if (!stkCGALUVParametrizationUtilities::ScaleUV(
        inputMesh, out, this->ScalingMode, this->UVScaling, this))
  {
    vtkErrorMacro("Failed to scale the UV map.");
    return 0;
  }
  output->ShallowCopy(out);

  return 1;
}

//---------------------------------------------------
template<typename SurfaceMesh, typename VertexUVMap>
bool stkCGALLSCMUVParametrization::UpdatePointCoordinates(
  SurfaceMesh& sm, VertexUVMap& uv_map, vtkPolyData* polyData)
{
  std::size_t vertices_counter = 0;
  typedef boost::unordered_map<vertex_descriptor, std::size_t> Vertex_index_map;
  Vertex_index_map vium;
  boost::associative_property_map<Vertex_index_map> vimap(vium);

  vtkPoints* pts = polyData->GetPoints();
  vtkDataArray* ptArray = pts->GetData();

  // Process
  typename boost::graph_traits<SurfaceMesh>::vertex_iterator vit, vend;
  boost::tie(vit, vend) = CGAL::vertices(sm);

  while (vit != vend)
  {
    vertex_descriptor& vd = *vit++;
    const Point_2& coord = uv_map[vd];
    ptArray->SetTuple3(vertices_counter, coord[0], coord[1], 0);

    put(vimap, vd, vertices_counter++);
  }

  if (vertices_counter != sm.number_of_vertices())
  {
    vtkErrorMacro("Mismatch on number of treated UV points.");
    return false;
  }
  else
    return true;
}
