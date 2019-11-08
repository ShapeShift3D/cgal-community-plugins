#ifndef vtkCGAL3DPolyhedralMesher_h
#define vtkCGAL3DPolyhedralMesher_h

#include "vtkPolyDataAlgorithm.h"

class vtkPointSet;

class vtkCGAL3DPolyhedralMesher : public vtkPolyDataAlgorithm
{
public:
  static vtkCGAL3DPolyhedralMesher* New();
  vtkTypeMacro(vtkCGAL3DPolyhedralMesher, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;
  
  //@{
  /**
  * This constant is used as an upper bound for the distance 
  * between two protecting ball centers that are consecutive on a 1-feature.
  * This parameter has to be set to a positive value when 1-dimensional features protection is used.
  */
  vtkSetMacro(EdgeSize, double);
  vtkGetMacro(EdgeSize, double);
  //@}

  //@{
  /**
  * This parameter controls the shape of surface facets. Specifically, it is a lower bound for the angle (in degrees) 
  * of surface facets. When boundary surfaces are smooth, the termination of the meshing process is guaranteed 
  * if this angular bound is at most 30 degrees.
  */
  vtkSetMacro(FacetAngle, double);
  vtkGetMacro(FacetAngle, double);
  //@}

  //@{
  /**
  * This parameter controls the size of surface facets. 
  * Each surface facet has a surface Delaunay ball which is a ball circumscribing the surface facet 
  * and centered on the surface patch. The parameter facet_size is providing an upper bound 
  * for the radii of surface Delaunay balls.
  */
  vtkSetMacro(FacetSize, double);
  vtkGetMacro(FacetSize, double);
  //@}

  //@{
  /**
  * This parameter controls the approximation error of boundary and subdivision surfaces. 
  * It provides an upper bound for the distance between the circumcenter of a surface facet 
  * and the center of a surface Delaunay ball of this facet.
  */
  vtkSetMacro(FacetDistance, double);
  vtkGetMacro(FacetDistance, double);
  //@}

  //@{
  /**
  * This parameter controls the shape of mesh cells (but can't filter slivers, as we discussed earlier). 
  * It is an upper bound for the ratio between the circumradius of a mesh tetrahedron and its shortest edge. 
  * There is a theoretical bound for this parameter: the Delaunay refinement process is guaranteed to terminate 
  * for values of cell_radius_edge_ratio bigger than 2.
  */
  vtkSetMacro(CellRadiusEdgeRatio, double);
  vtkGetMacro(CellRadiusEdgeRatio, double);
  //@}

  //@{
  /**
  * This scalar parameter controls the size of mesh tetrahedra. 
  * It provides an upper bound on the circumradii of the mesh tetrahedra.
  */
  vtkSetMacro(CellSize, double);
  vtkGetMacro(CellSize, double);
  //@}

  //@{
  /**
  * Activates/deactivates the Lloyd smoother.
  */
  vtkSetMacro(Lloyd, bool);
  vtkGetMacro(Lloyd, bool);
  vtkBooleanMacro(Lloyd, bool);
  //@}

  //@{
  /**
  * Activates/deactivates the ODT smoother.
  */
  vtkSetMacro(Odt, bool);
  vtkGetMacro(Odt, bool);
  vtkBooleanMacro(Odt, bool);
  //@}

  //@{
  /**
  * Activates/deactivates the perturber.
  */
  vtkSetMacro(Perturb, bool);
  vtkGetMacro(Perturb, bool);
  vtkBooleanMacro(Perturb, bool);
  //@}

  //@{
  /**
  * Activates/deactivates the exuder.
  */
  vtkSetMacro(Exude, bool);
  vtkGetMacro(Exude, bool);
  vtkBooleanMacro(Exude, bool);
  //@}
  
  typedef enum {
    MANIFOLD = 1,
    MANIFOLD_WITH_BOUNDARY,
    NON_MANIFOLD
  } TopologicalStructures;

  //@{
  /**
  * Specifies the manifold of the interior surfaces
  */
  vtkGetMacro(TopologicalStructure, int);
  vtkSetMacro(TopologicalStructure, int);
  //@}

  virtual void SetTopologicalStructureToManifold(void) 
    { this->SetTopologicalStructure(vtkCGAL3DPolyhedralMesher::TopologicalStructures::MANIFOLD); }

  virtual void SetTopologicalStructureToManifoldWithBoundary(void) 
    { this->SetTopologicalStructure(vtkCGAL3DPolyhedralMesher::TopologicalStructures::MANIFOLD_WITH_BOUNDARY); }

  virtual void SetTopologicalStructureToNonManifold(void) 
    { this->SetTopologicalStructure(vtkCGAL3DPolyhedralMesher::TopologicalStructures::NON_MANIFOLD); }

  //@{
  /**
  * If true, every point of the interior surface will be part of the tetrahedral mesh output.
  * If false, the default behavior is applied.
  */
  vtkSetMacro(ConfineSurfacePoints, bool);
  vtkGetMacro(ConfineSurfacePoints, bool);
  vtkBooleanMacro(ConfineSurfacePoints, bool);
  //@}

  vtkPolyData* GetInteriorSurfaces();
  vtkPolyData* GetBoundingDomain();

protected:
  vtkCGAL3DPolyhedralMesher();
  ~vtkCGAL3DPolyhedralMesher(){}
  
  virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;
  virtual int FillOutputPortInformation(int, vtkInformation *info) override;

private:
  vtkCGAL3DPolyhedralMesher(const vtkCGAL3DPolyhedralMesher&) = delete;
  void operator=(const vtkCGAL3DPolyhedralMesher&) = delete;

  template <typename TM>
  bool vtkPolyDataToPolygonMesh(vtkPointSet* poly_data, TM& tmesh);

  // Mesh Criteria
  double EdgeSize;
  double FacetAngle;
  double FacetSize;
  double FacetDistance;
  double CellRadiusEdgeRatio;
  double CellSize;

  // Optimizer flags
  bool Lloyd;
  bool Odt;
  bool Perturb;
  bool Exude;

  int TopologicalStructure;
  bool ConfineSurfacePoints;
};
#endif
