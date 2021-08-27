/**
 * \class stkCGALUVParametrizationUtilities
 *
 * \brief Set of CGAL utility functions for UV parametrization methods.
 *
 */

#include "stkCGALUVParametrizationUtilities.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkObjectFactory.h>
#include <vtkSmartPointer.h>

#include <vtkDataArray.h>
#include <vtkPolyData.h>

#include <vtkMassProperties.h>

#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

vtkStandardNewMacro(stkCGALUVParametrizationUtilities);

//---------------------------------------------------
bool stkCGALUVParametrizationUtilities::ScaleUV(vtkPolyData* surface, vtkPolyData* UVSurface,
  int scalingMode, double scalingValue, vtkObject* caller /* = nullptr */)
{
  vtkNew<vtkTransform> transform;

  vtkNew<vtkTransformPolyDataFilter> transformFilter;
  transformFilter->SetInputData(UVSurface);
  transformFilter->SetTransform(transform);

  // Scale the uv
  switch (scalingMode)
  {
    case ScalingModes::MANUAL:
    {
      transform->Scale(scalingValue, scalingValue, scalingValue);
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
      if (caller)
      {
        vtkErrorWithObjectMacro(caller, "Non-existent mode.");
      }
      else
      {
        std::cout << "stkCGALUVParametrizationUtilities::ScaleUV(...) Error: Non-existent mode."
                  << std::endl;
      }

      return false;
    }
  }

  return true;
}
