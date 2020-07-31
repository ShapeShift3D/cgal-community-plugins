/**
* \class vtkCGALNefPolyhedronTest
*
* \brief This filter takes two inputs, inputMeshA and inputMeshB, of type vtkPolyData and applies one of the four boolean operations (Union, Intersection, Difference1 (A - B) and Difference2 (B - A))
*        to them. The user will have the option to select one of the four operations from a drop down menu. The two inputs will be converted to CGAL Polygon Mesh class since vtkPolyData is not a valid input. 
*		 The converted inputs will then be fed into the appropriate function for execution. The result of the function will be converted to a vtkUnstructuredGrid and be outputted as such. 
*        
*		 
*        
* Inputs: inputMeshA (port == 0, vtkPolyData), inputMeshB (port == 1, vtkPolyData)
* Output: output (port == 0, vtkUnstructuredGrid)
* 
*/

//---------VTK----------------------------------
#include "vtkCGALNefPolyhedronTest.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>

#include <vtkCGALUtilities.h>

//---------CGAL---------------------------------
#include <CGAL/Exact_integer.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>

//----------
// Declare the plugin
vtkStandardNewMacro(vtkCGALNefPolyhedronTest);

typedef CGAL::Exact_integer RT;
typedef CGAL::Homogeneous<CGAL::Exact_integer>  Kernel;
typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron;
typedef CGAL::Polyhedron_3<Kernel>  Polyhedron;
typedef CGAL::Surface_mesh<Kernel::Point_3> Surface_Mesh;

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
vtkCGALNefPolyhedronTest::vtkCGALNefPolyhedronTest()
{
  this->SetNumberOfInputPorts(2);
  this->SetNumberOfOutputPorts(1);
}

//---------------------------------------------------
vtkPolyData* vtkCGALNefPolyhedronTest::GetInputMeshA()
{
	if (this->GetNumberOfInputConnections(0) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

//----------------------------------------------------
vtkPolyData* vtkCGALNefPolyhedronTest::GetInputMeshB()
{
	if (this->GetNumberOfInputConnections(1) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(1, 0));
}

// ----------------------------------------------------------------------------
// Gets the input
// Creates CGAL::Surface_mesh from vtkPolydata
// Calls the CGAL::RunBooleanOperations
// Fills the output vtkUnstructuredGrid from the result.
int vtkCGALNefPolyhedronTest::RequestData(vtkInformation *,
                                vtkInformationVector **inputVector,
                                vtkInformationVector *outputVector)
{
	//  Get the input and output data objects.
	//  Get the info objects
	vtkPolyData* inputMeshA = this->GetInputMeshA();
	vtkPolyData* inputMeshB = this->GetInputMeshB();

	if (inputMeshA == nullptr)
	{
		vtkErrorMacro("Input Mesh A is empty.");
		return 0;
	}

	if (inputMeshA->GetPolys() == nullptr)
	{
		vtkErrorMacro("Input Mesh A does not contain any cell structure.");
		return 0;
	}

	if (inputMeshA->GetNumberOfCells() == 0)
	{
		vtkErrorMacro("Input Mesh A contains no cells.");
		return 0;
	}

	if (inputMeshA->GetPoints() == nullptr)
	{
		vtkErrorMacro("Input Mesh A does not contain any point structure.");
		return 0;
	}

	if (inputMeshA->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("Input Mesh A contains no points.");
		return 0;
	}

	if (inputMeshB == nullptr)
	{
		vtkErrorMacro("Input Mesh B is empty.");
		return 0;
	}

	if (inputMeshB->GetPolys() == nullptr)
	{
		vtkErrorMacro("Input Mesh B does not contain any cell structure.");
		return 0;
	}

	if (inputMeshB->GetNumberOfCells() == 0)
	{
		vtkErrorMacro("Input Mesh B contains no cells.");
		return 0;
	}

	if (inputMeshB->GetPoints() == nullptr)
	{
		vtkErrorMacro("Input Mesh B does not contain any point structure.");
		return 0;
	}

	if (inputMeshB->GetNumberOfPoints() == 0)
	{
		vtkErrorMacro("Input Mesh B contains no points.");
		return 0;
	}

	vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));

	Polyhedron polyA;
	vtkCGALUtilities::vtkPolyDataToPolygonMesh(inputMeshA, polyA);
	Nef_polyhedron NefPolyA(polyA);

	Polyhedron polyB;
	vtkCGALUtilities::vtkPolyDataToPolygonMesh(inputMeshA, polyB);
	Nef_polyhedron NefPolyB(polyB);

	Nef_polyhedron NefPoly = NefPolyA.intersection(NefPolyB);
	Surface_Mesh resultPoly;
	CGAL::convert_nef_polyhedron_to_polygon_mesh(NefPoly, resultPoly);
	//vtkCGALUtilities::PolyhedronToPolyData(resultPoly, output0);
	vtkCGALUtilities::SurfaceMeshToPolyData(resultPoly, output0);

	return 1;
}

////----------------------------------------------------
//void vtkCGALNefPolyhedronTest::explore(std::string s, const Nef_polyhedron& poly)
//{
//	std::cout << "Explore: " << s << std::endl;
//	CGAL::Explorer explorer = poly.explorer();
//	int i = 0;
//	for (Face_const_iterator fit = explorer.faces_begin(); fit != explorer.faces_end(); ++fit, i++) {
//		std::cout << "\nFace " << i << std::endl
//			<< (explorer.mark(fit) ? "* is" : "* is not") << " marked" << std::endl;
//		// explore the outer face cycle if it exists
//		Halfedge_around_face_const_circulator hafc = explorer.face_cycle(fit);
//		if (hafc == Halfedge_around_face_const_circulator()) {
//			std::cout << "* has no outer face cycle" << std::endl;
//		}
//		else {
//			std::cout << "* outer face cycle:\n";
//			std::cout << "  - halfedges around the face: ";
//			Halfedge_around_face_const_circulator done(hafc);
//			do {
//				char c = (explorer.is_frame_edge(hafc)) ? 'f' : 'e';
//				std::cout << c;
//				++hafc;
//			} while (hafc != done);
//			std::cout << " ( f = frame edge, e = ordinary edge)" << std::endl;
//			std::cout << "  - vertices around the face:\n";
//			do {
//				Vertex_const_handle vh = explorer.target(hafc);
//				if (explorer.is_standard(vh)) {
//					std::cout << "      " << explorer.point(vh) << std::endl;
//				}
//				else {
//					std::cout << "      " << explorer.ray(vh) << std::endl;
//				}
//				++hafc;
//			} while (hafc != done);
//		}
//		// explore the holes if the face has holes
//		Hole_const_iterator hit = explorer.holes_begin(fit), end = explorer.holes_end(fit);
//		if (hit == end) {
//			std::cout << "* has no hole" << std::endl;
//		}
//		else {
//			std::cout << "* has holes" << std::endl;
//			for (; hit != end; hit++) {
//				Halfedge_around_face_const_circulator hafc(hit), done(hit);
//				std::cout << "  - halfedges around the hole: ";
//				do {
//					char c = (explorer.is_frame_edge(hafc)) ? 'f' : 'e';
//					std::cout << c;
//					++hafc;
//				} while (hafc != done);
//				std::cout << " ( f = frame edge, e = ordinary edge)" << std::endl;
//			}
//		}
//	}
//	std::cout << "done\n" << std::endl;
//}