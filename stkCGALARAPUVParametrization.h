#ifndef stkCGALARAPUVParametrization_h
#define stkCGALARAPUVParametrization_h

#include <stkCGALModule.h>
#include <stkCGALAbstractUVParametrization.h>

class STKCGAL_EXPORT stkCGALARAPUVParametrization : public stkCGALAbstractUVParametrization
{
public:
    static stkCGALARAPUVParametrization* New();
    vtkTypeMacro(stkCGALARAPUVParametrization, stkCGALAbstractUVParametrization);
 
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

    //@{
    /**
     * Minimal energy difference between two iterations for the minimization process to continue.
     */
    vtkSetMacro(SkipPostprocess, bool);
    vtkGetMacro(SkipPostprocess, bool);
    vtkBooleanMacro(SkipPostprocess, bool);
    //@}

protected:
    stkCGALARAPUVParametrization();
    ~stkCGALARAPUVParametrization();

    virtual int RequestData(
        vtkInformation* request,
        vtkInformationVector** inputVector,
        vtkInformationVector* outputVector
    ) override;
  
private:
    // needed but not implemented
    stkCGALARAPUVParametrization(const stkCGALARAPUVParametrization&) = delete;
    void operator=(const stkCGALARAPUVParametrization&) = delete;

    template <typename SurfaceMesh, typename VertexUVMap>
    bool UpdatePointCoordinates(SurfaceMesh& sm, VertexUVMap& uv_map, vtkPolyData* polyData);

    template<typename SurfaceMesh, typename VertexUVMap>
    bool UVMapToPolyData(SurfaceMesh& sm, VertexUVMap& uv_map, vtkPolyData* polyData);

    double Lambda;
    double Tolerance;
    int MaximumNumberOfIterations;
    bool SkipPostprocess;
};
#endif
