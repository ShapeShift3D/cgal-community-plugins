#ifndef vtkCGALARAPUVParametrization_h
#define vtkCGALARAPUVParametrization_h

#include <vtkCGALModule.h>
#include <vtkCGALAbstractUVParametrization.h>

class VTKCGAL_EXPORT vtkCGALARAPUVParametrization : public vtkCGALAbstractUVParametrization
{
public:
    static vtkCGALARAPUVParametrization* New();
    vtkTypeMacro(vtkCGALARAPUVParametrization, vtkCGALAbstractUVParametrization);
 
    //@{
    /**
     * Parameter to give importance to shape or angle preservation.
     */
    vtkSetMacro(Lambda, double);
    vtkGetMacro(Lambda, double);
    //@}

    //@{
    /**
     * Minimal energy difference between two iterations for the minimization process to continue.
     */
    vtkSetMacro(Tolerance, double);
    vtkGetMacro(Tolerance, double);
    //@}

    //@{
    /**
     * Maximal number of iterations in the energy minimization process.
     */
    vtkSetMacro(MaximumNumberOfIterations, int);
    vtkGetMacro(MaximumNumberOfIterations, int);
    //@}

protected:
    vtkCGALARAPUVParametrization();
    ~vtkCGALARAPUVParametrization();

    virtual int RequestData(
        vtkInformation* request,
        vtkInformationVector** inputVector,
        vtkInformationVector* outputVector
    ) override;
  
private:
    // needed but not implemented
    vtkCGALARAPUVParametrization(const vtkCGALARAPUVParametrization&) = delete;
    void operator=(const vtkCGALARAPUVParametrization&) = delete;

    template <typename SurfaceMesh, typename VertexUVMap>
    bool UpdatePointCoordinates(SurfaceMesh& sm, VertexUVMap& uv_map, vtkPolyData* polyData);

    double Lambda;
    double Tolerance;
    int MaximumNumberOfIterations;
};
#endif
