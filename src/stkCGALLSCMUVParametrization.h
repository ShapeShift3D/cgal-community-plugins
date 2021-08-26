#ifndef stkCGALLSCMUVParametrization_h
#define stkCGALLSCMUVParametrization_h

#include <stkCGALModule.h>
#include <stkCGALAbstractUVParametrization.h>

class STKCGAL_EXPORT stkCGALLSCMUVParametrization : public stkCGALAbstractUVParametrization
{
public:
    static stkCGALLSCMUVParametrization* New();
    vtkTypeMacro(stkCGALLSCMUVParametrization, stkCGALAbstractUVParametrization);
 
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
    stkCGALLSCMUVParametrization();
    ~stkCGALLSCMUVParametrization();

    virtual int RequestData(
        vtkInformation* request,
        vtkInformationVector** inputVector,
        vtkInformationVector* outputVector
    ) override;
  
private:
    // needed but not implemented
    stkCGALLSCMUVParametrization(const stkCGALLSCMUVParametrization&) = delete;
    void operator=(const stkCGALLSCMUVParametrization&) = delete;

    template <typename SurfaceMesh, typename VertexUVMap>
    bool UpdatePointCoordinates(SurfaceMesh& sm, VertexUVMap& uv_map, vtkPolyData* polyData);

    double Lambda;
    double Tolerance;
    int MaximumNumberOfIterations;
};
#endif
