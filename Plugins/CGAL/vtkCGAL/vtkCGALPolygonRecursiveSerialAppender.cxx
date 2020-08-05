/**
* \class vtkCGALPolygonRecursiveSerialAppender
*
* \brief 
*        
*		 Conditions for valid polygons:
*
*		 Closed Boundary - the polygon's outer boundary must be a connected sequence of curves, that start and end at the same vertex.
*		 Simplicity - the polygon must be simple.
*		 Orientation - the polygon's outer boundary must be counter-clockwise oriented.
*        
* Inputs: inputMeshA (port == 0, vtkPolyData), inputMeshB (port == 1, vtkPolyData)
* Output: output (port == 0, vtkUnstructuredGrid)
* 
*/

//---------VTK----------------------------------
#include "vtkCGALPolygonRecursiveSerialAppender.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkObjectFactory.h>
#include <vtkSmartPointer.h>

#include <vtkPoints.h>
#include <vtkPolyData.h>

//----------
// Declare the plugin
vtkStandardNewMacro(vtkCGALPolygonRecursiveSerialAppender);
vtkCxxSetObjectMacro(vtkCGALPolygonRecursiveSerialAppender, Boolean2DMesher, vtkPolyDataAlgorithm);

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
vtkCGALPolygonRecursiveSerialAppender::vtkCGALPolygonRecursiveSerialAppender()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);

  this->Boolean2DMesher = nullptr;
}

//----------------------------------------------------------------------------
vtkCGALPolygonRecursiveSerialAppender::~vtkCGALPolygonRecursiveSerialAppender()
{
	if (this->Boolean2DMesher != nullptr)
	{
		this->Boolean2DMesher->Delete();
		this->SetBoolean2DMesher(nullptr);
	}
}

//----------------------------------------------------------------------------
int vtkCGALPolygonRecursiveSerialAppender::FillInputPortInformation(int port, vtkInformation* info)
{
	if (!this->Superclass::FillInputPortInformation(port, info))
	{
		return 0;
	}

	info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPolyData");
	info->Set(vtkAlgorithm::INPUT_IS_REPEATABLE(), 1);
	info->Set(vtkAlgorithm::INPUT_IS_OPTIONAL(), 1);
	return 1;
}


// ----------------------------------------------------------------------------
// Gets the input
// Creates CGAL::Surface_mesh from vtkPolydata
// Calls the CGAL::RunBooleanOperations
// Fills the output vtkUnstructuredGrid from the result.
int vtkCGALPolygonRecursiveSerialAppender::RequestData(vtkInformation *,
                                vtkInformationVector **inputVector,
                                vtkInformationVector *outputVector)
{
	int numConnections = this->GetNumberOfInputConnections(0);

	for (int i = 0; i < numConnections; ++i)
	{
		vtkPolyData* input = vtkPolyData::GetData(inputVector[0]->GetInformationObject(i));

		if (input == nullptr)
		{
			vtkErrorMacro("Input " << i << " is empty.");
			return 0;
		}

		if (input->GetPoints() == nullptr)
		{
			vtkErrorMacro("Input PolyLine " << i << " does not contain any point structure.");
			return 0;
		}

		if (input->GetNumberOfPoints() == 0)
		{
			vtkErrorMacro("Input PolyLine " << i << " contains no points.");
			return 0;
		}
	}

	vtkPolyData* output0 = vtkPolyData::GetData(outputVector->GetInformationObject(0));

	vtkNew<vtkPolyData> previousResult;

	for (int i = 1; i < numConnections; ++i)
	{
		vtkPolyData* previousOutput = this->Boolean2DMesher->GetOutput();

		if (previousOutput != nullptr)
		{
			previousResult->DeepCopy(this->Boolean2DMesher->GetOutput());
		}
		
		vtkPolyData* input0 = i == 1 ? vtkPolyData::GetData(inputVector[0]->GetInformationObject(i - 1)) : previousResult; // Serial chaining
		vtkPolyData* input1 = vtkPolyData::GetData(inputVector[0]->GetInformationObject(i));
		this->Boolean2DMesher->SetInputData(0, input0);
		this->Boolean2DMesher->SetInputData(1, input1);
		this->Boolean2DMesher->Update();
	}

	output0->ShallowCopy(this->Boolean2DMesher->GetOutput());

	return 1;
}

//--------------------------------------------------------------------------
vtkMTimeType vtkCGALPolygonRecursiveSerialAppender::GetMTime()
{
	vtkMTimeType mTime = this->Superclass::GetMTime();
	vtkMTimeType mTime2;
	if (this->Boolean2DMesher != nullptr)
	{
		mTime2 = this->Boolean2DMesher->GetMTime();
		mTime = (mTime2 > mTime ? mTime2 : mTime);
	}
	return mTime;
}
