/**
* \class vtkCGALPHBooleanSetOperations
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
#include "vtkCGALPHBooleanSetOperations.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <vtkPolyData.h>

#include <CGAL/Boolean_set_operations_2.h>

//----------
// Declare the plugin
vtkStandardNewMacro(vtkCGALPHBooleanSetOperations);

// -----------------------------------------------------------------------------
// Constructor
// Fills the number of input and output objects.
// Initializes the members that need it.
vtkCGALPHBooleanSetOperations::vtkCGALPHBooleanSetOperations()
{ 
	this->OperationMode = vtkCGALPHBooleanSetOperations::OperationModes::JOIN;
	this->InputMode = InputModes::WITHOUT_HOLE;

	this->PwhA = nullptr;
	this->PA = nullptr;
	this->PwhB = nullptr;
	this->PB = nullptr;
	this->PwhListOut = nullptr;
}

//---------------------------------------------------
void vtkCGALPHBooleanSetOperations::SetPolygonWithHoleAInput(Polygon_with_holes_2* pwhA)
{
	this->PwhA = pwhA;
}

//---------------------------------------------------
void vtkCGALPHBooleanSetOperations::SetPolygonAInput(Polygon_2* pA)
{
	this->PA = pA;
}

//----------------------------------------------------
void vtkCGALPHBooleanSetOperations::SetPolygoneWithHoleBInput(Polygon_with_holes_2* pwhB)
{
	this->PwhB = pwhB;
}

//---------------------------------------------------
void vtkCGALPHBooleanSetOperations::SetPolygonBInput(Polygon_2* pB)
{
	this->PB = pB;
}

//---------------------------------------------------
void vtkCGALPHBooleanSetOperations::SetPolygonWithHoleListOutput(Pwh_list_2* pwhList)
{
	this->PwhListOut = pwhList;
}

// ----------------------------------------------------------------------------
int vtkCGALPHBooleanSetOperations::Update()
{
	int retValue = 1;
	if (this->PwhListOut == nullptr)
	{
		vtkErrorMacro("Output is not set.");
		return 0;
	}

	if (this->InputMode == InputModes::WITHOUT_HOLE)
	{
		if (!VerifyInput(this->PA, "Polygon A"))
			return 0;

		if (!VerifyInput(this->PB, "Polygon B"))
			return 0;

		retValue = Operate<Polygon_2, Polygon_2>(this->PA, this->PB);
	}
	else if (this->InputMode == InputModes::WITH_HOLE)
	{
		if (!VerifyInput(this->PwhA, "Polygon with hole A"))
			return 0;

		if (!VerifyInput(this->PwhB, "Polygon with hole B"))
			return 0;

		retValue = Operate<Polygon_with_holes_2, Polygon_with_holes_2>(this->PwhA, this->PwhB);
	}

	//try
	//{
	//	if (this->OperationMode == vtkCGALPHBooleanSetOperations::OperationModes::COMPLEMENT)
	//	{
	//		vtkErrorMacro("To fix.");
	//		//CGAL::complement(this->PwhA, std::back_inserter(*this->PwhListOut));
	//	}
	//	else if (this->OperationMode == vtkCGALPHBooleanSetOperations::OperationModes::INTERSECTION)
	//	{
	//		CGAL::intersection(this->PwhA, this->PwhB, std::back_inserter(*this->PwhListOut));
	//	}
	//	else if (this->OperationMode == vtkCGALPHBooleanSetOperations::OperationModes::JOIN)
	//	{
	//		CGAL::join(this->PwhA, this->PwhB, std::back_inserter(*this->PwhListOut));
	//	}
	//	else if (this->OperationMode == vtkCGALPHBooleanSetOperations::OperationModes::DIFFERENCE)
	//	{
	//		vtkErrorMacro("To fix.");
	//		//CGAL::difference(this->PwhA, this->PwhB, std::back_inserter(*this->PwhListOut));
	//	}
	//	else if (this->OperationMode == vtkCGALPHBooleanSetOperations::OperationModes::SYMMETRIC_DIFFERENCE)
	//	{
	//		CGAL::symmetric_difference(this->PwhA, this->PwhB, std::back_inserter(*this->PwhListOut));
	//	}
	//}
	//catch (const std::exception& e)
	//{
	//	vtkErrorMacro(<< "Error caught : " << e.what());
	//	return 0;
	//}

	return retValue;
}

// ----------------------------------------------------------------------------
template<typename InputAType, typename InputBType>
int vtkCGALPHBooleanSetOperations::Operate(InputAType* polygonA, InputBType* polygonB)
{
	try
	{
		if (this->OperationMode == vtkCGALPHBooleanSetOperations::OperationModes::COMPLEMENT)
		{
			vtkErrorMacro("To fix.");
			//CGAL::complement(this->PwhA, std::back_inserter(*this->PwhListOut));
		}
		else if (this->OperationMode == vtkCGALPHBooleanSetOperations::OperationModes::INTERSECTION)
		{
			CGAL::intersection(polygonA, polygonB, std::back_inserter(*this->PwhListOut));
		}
		else if (this->OperationMode == vtkCGALPHBooleanSetOperations::OperationModes::JOIN)
		{
			CGAL::join(polygonA, polygonB, std::back_inserter(*this->PwhListOut));
		}
		else if (this->OperationMode == vtkCGALPHBooleanSetOperations::OperationModes::DIFFERENCE)
		{
			vtkErrorMacro("To fix.");
			//CGAL::difference(this->PwhA, this->PwhB, std::back_inserter(*this->PwhListOut));
		}
		else if (this->OperationMode == vtkCGALPHBooleanSetOperations::OperationModes::SYMMETRIC_DIFFERENCE)
		{
			CGAL::symmetric_difference(polygonA, polygonB, std::back_inserter(*this->PwhListOut));
		}
	}
	catch (const std::exception& e)
	{
		vtkErrorMacro(<< "Error caught : " << e.what());
		return 0;
	}
}

// ----------------------------------------------------------------------------
template<typename InputType>
int vtkCGALPHBooleanSetOperations::VerifyInput(InputType* input, std::string inputName)
{
	if (input == nullptr)
	{
		vtkErrorMacro(<< inputName + " is empty.");
		return 0;
	}

	return 1;
}