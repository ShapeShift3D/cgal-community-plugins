/**
 * @class vtkCGALIsotropicRemeshingFilter
 * @brief Remeshes a given vtkPolyData
 *
 * This filter performs isotropic remeshing. I takes a vtkPolyData as input, and returns a vtkPolyData
 * containing the remeshed input. An optional input array can be supplied to mask the remeshing zone.
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
 *        At the moment, a threshold is applied to the input, its result (inner region) is remeshed then the
 *        output will be assigned to the remeshed region appended with the outer region (from the threshold).
 *        There might be a better approach using CGAL. 
*/

#ifndef vtkCGALIsotropicRemeshingFilter_h
#define vtkCGALIsotropicRemeshingFilter_h

#include <vtkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class VTKCGAL_EXPORT vtkCGALIsotropicRemeshingFilter : public vtkPolyDataAlgorithm
{
public:
  static vtkCGALIsotropicRemeshingFilter* New();
  vtkTypeMacro(vtkCGALIsotropicRemeshingFilter, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  //@{
  /**
  * Set/Get TargetEdgeLength. The target edge length for the remeshed surface patch.
  */
  vtkSetClampMacro(TargetEdgeLength, double, 0.0, VTK_DOUBLE_MAX);
  vtkGetMacro(TargetEdgeLength, double);
  //@}

  //@{
  /**
  * Set/Get TargetEdgeLengthInfo.
  * It serves as a default target edge length and is calculated in RequestInformation().
  */
  vtkGetMacro(TargetEdgeLengthInfo, double);
  //@}

  //@{
  /**
  * Set/Get NumberOfIterations.
  * The bigger this number, the smoother and closer to target edge length the mesh will be.
  */
  vtkSetClampMacro(NumberOfIterations, int, 1, VTK_INT_MAX);
  vtkGetMacro(NumberOfIterations, int);
  //@}

  //@{
  /**
  * Set/Get ProtectConstraints.
  * 
  * From https://doc.cgal.org/latest/Polygon_mesh_processing/group__PMP__meshing__grp.html
  * An additional option has been added to protect (i.e. not modify) some given polylines.
  * In some cases, those polylines are too long, and reaching the desired target edge length
  * while protecting them is not possible and leads to an infinite loop of edge splits in the incident
  * faces. To avoid that pitfall, the function CGAL::Polygon_mesh_processing::split_long_edges()
  * should be called on the list of constrained edges before remeshing.
  * 
  * Note: if masking array is used, the edge splitting will not be executed.
  */
  vtkSetMacro(ProtectConstraints, vtkTypeBool);
  vtkGetMacro(ProtectConstraints, vtkTypeBool);
  vtkBooleanMacro(ProtectConstraints, vtkTypeBool);
  //@}

  //@{
  /**
  * Set/Get MeshingMaskArrayName. A masking array used to limit the remeshing zone.
  */
  vtkSetStringMacro(MeshingMaskArrayName);
  vtkGetStringMacro(MeshingMaskArrayName);
  //@}

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

protected:
  vtkCGALIsotropicRemeshingFilter();
  ~vtkCGALIsotropicRemeshingFilter();

  int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*);

  double TargetEdgeLength;
  double TargetEdgeLengthInfo;
  int NumberOfIterations;
  vtkTypeBool ProtectConstraints;
  char* MeshingMaskArrayName;

private:
  vtkCGALIsotropicRemeshingFilter(const vtkCGALIsotropicRemeshingFilter&) = delete;
  void operator=(const vtkCGALIsotropicRemeshingFilter&) = delete;
};
#endif
