/**
 * @class stkCGALIsotropicRemeshingFilter
 * @brief Remeshes a given vtkPolyData
 *
 * This filter performs isotropic remeshing. I takes a vtkPolyData as input, and returns a
 * vtkPolyData containing the remeshed input. An optional input array can be supplied to mask the
 * remeshing zone.
 *
 * Based on vtkIsotropicRemeshingFilter from https://github.com/CGAL/cgal-paraview-plugins
 *
 *
 * When masking is used, the following steps are executed:
 *  - Threshold the regions (inner and outer)
 *  - Remesh the inner region
 *  - Append remeshed and outer regions
 *
 * Note that in such scenario, we don't split border edges to preserve topology.
 * Otherwise the output mesh could be non-manifold.
 *
 *
 * TODO:
 *      - Explore alternative solution to extract and process the remeshing region.
 *        At the moment, a threshold is applied to the input, its result (inner region) is remeshed
 * then the output will be assigned to the remeshed region appended with the outer region (from the
 * threshold). There might be a better approach using CGAL.
 */

#pragma once

#include <stkCGALIsotropicRemeshingFilterInterface.h>
#include <stkCGALModule.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALIsotropicRemeshingFilter
  : public stkCGALIsotropicRemeshingFilterInterface
{
public:
  static stkCGALIsotropicRemeshingFilter* New();
  vtkTypeMacro(stkCGALIsotropicRemeshingFilter, stkCGALIsotropicRemeshingFilterInterface);
  void PrintSelf(ostream& os, vtkIndent indent) override;

protected:
  stkCGALIsotropicRemeshingFilter() = default;
  ~stkCGALIsotropicRemeshingFilter() = default;

  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  virtual int RequestInformation(
    vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALIsotropicRemeshingFilter(const stkCGALIsotropicRemeshingFilter&) = delete;
  void operator=(const stkCGALIsotropicRemeshingFilter&) = delete;
};
