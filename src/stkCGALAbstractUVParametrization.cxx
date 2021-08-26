#include "stkCGALAbstractUVParametrization.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkObjectFactory.h>
#include <vtkSmartPointer.h>

#include <vtkMassProperties.h>

#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

//---------------------------------------------------
stkCGALAbstractUVParametrization::stkCGALAbstractUVParametrization()
{
	this->SetNumberOfInputPorts(1);
	this->SetNumberOfOutputPorts(1);

    this->ScalingMode = 1;
    this->UVScaling = 1;
}

//---------------------------------------------------
stkCGALAbstractUVParametrization::~stkCGALAbstractUVParametrization()
{}

//---------------------------------------------------
int stkCGALAbstractUVParametrization::FillInputPortInformation(int port, vtkInformation *info)
{
    info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPolyData");
    return 1;
}

//---------------------------------------------------
int stkCGALAbstractUVParametrization::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector* outputVector) 
{
    // To override.
	return 1;
}

//---------------------------------------------------
vtkPolyData* stkCGALAbstractUVParametrization::GetInputMesh()
{
	if (this->GetNumberOfInputConnections(0) < 1) {
		return nullptr;
	}

	return vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
}

//---------------------------------------------------
bool stkCGALAbstractUVParametrization::ScaleUV(vtkPolyData* surface, vtkPolyData* UVSurface)
{
	vtkNew<vtkTransform> transform;

	vtkNew<vtkTransformPolyDataFilter> transformFilter;
	transformFilter->SetInputData(UVSurface);
	transformFilter->SetTransform(transform);

	// Scale the uv
	switch (this->ScalingMode)
	{
	case ScalingModes::MANUAL:
	{
		transform->Scale(this->UVScaling, this->UVScaling, this->UVScaling);
		transformFilter->Update();
		UVSurface->ShallowCopy(transformFilter->GetOutput());
		break;
	}
	case ScalingModes::AUTO:
	{
		// Calculate input surface area
		vtkNew<vtkMassProperties> massPropertiesFilter;
		massPropertiesFilter->SetInputData(surface);
		massPropertiesFilter->Update();

		double surfaceMeshArea = massPropertiesFilter->GetSurfaceArea();

		// Calculate result surface area
		massPropertiesFilter->SetInputData(UVSurface);
		massPropertiesFilter->Update();

		double unwrappedSurfaceMeshArea = massPropertiesFilter->GetSurfaceArea();
		double ratio = std::sqrt(surfaceMeshArea / unwrappedSurfaceMeshArea);

		transform->Scale(ratio, ratio, ratio);
		transformFilter->Update();
		UVSurface->ShallowCopy(transformFilter->GetOutput());
		break;
	}
	default:
	{
		vtkErrorMacro("Non-existent mode.");
		return false;
	}
	}

	return true;
}