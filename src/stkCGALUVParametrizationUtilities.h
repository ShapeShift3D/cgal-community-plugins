/**
 * @class stkCGALUVParametrizationUtilities
 * @brief Set of CGAL utility functions usable by UV Parametrization classes in this module.
 *
 * @sa
 * stkCGALUVParametrizationUtilities
 */
#pragma once

#include <stkCGALModule.h>
#include <vtkObject.h>

class vtkPolyData;

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALUVParametrizationUtilities : public vtkObject
{
public:
  static stkCGALUVParametrizationUtilities* New();
  vtkTypeMacro(stkCGALUVParametrizationUtilities, vtkObject);

  typedef enum
  {
    MANUAL = 1,
    AUTO
  } ScalingModes;

  static bool ScaleUV(vtkPolyData* surface, vtkPolyData* UVSurface, int scalingMode,
    double scalingValue, vtkObject* caller = nullptr);

protected:
  stkCGALUVParametrizationUtilities() = default;
  ~stkCGALUVParametrizationUtilities() = default;

private:
  stkCGALUVParametrizationUtilities(const stkCGALUVParametrizationUtilities&) = delete;
  void operator=(const stkCGALUVParametrizationUtilities&) = delete;
};
