#ifndef vtkCGALLSCMUVParametrization_h
#define vtkCGALLSCMUVParametrization_h

#include <vtkCGALModule.h>
#include <vtkCGALAbstractUVParametrization.h>

class VTKCGAL_EXPORT vtkCGALLSCMUVParametrization : public vtkCGALAbstractUVParametrization
{
public:
    static vtkCGALLSCMUVParametrization* New();
    vtkTypeMacro(vtkCGALLSCMUVParametrization, vtkCGALAbstractUVParametrization);
 
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
    vtkCGALLSCMUVParametrization();
    ~vtkCGALLSCMUVParametrization();

    virtual int RequestData(
        vtkInformation* request,
        vtkInformationVector** inputVector,
        vtkInformationVector* outputVector
    ) override;
  
private:
    // needed but not implemented
    vtkCGALLSCMUVParametrization(const vtkCGALLSCMUVParametrization&) = delete;
    void operator=(const vtkCGALLSCMUVParametrization&) = delete;

    template <typename SurfaceMesh, typename VertexUVMap>
    bool UpdatePointCoordinates(SurfaceMesh& sm, VertexUVMap& uv_map, vtkPolyData* polyData);

    double Lambda;
    double Tolerance;
    int MaximumNumberOfIterations;
};
#endif
