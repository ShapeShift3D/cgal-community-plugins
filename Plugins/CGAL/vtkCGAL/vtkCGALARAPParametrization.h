#ifndef vtkCGALARAPParametrization_h
#define vtkCGALARAPParametrization_h

#include "vtkObject.h" 
#include <vtkCGALModule.h>

class vtkPolyData;

class VTKCGAL_EXPORT vtkCGALARAPParametrization : public vtkObject
{
public:
    static vtkCGALARAPParametrization* New();
    vtkTypeMacro(vtkCGALARAPParametrization, vtkObject);
 
    void SetInputMesh(vtkPolyData* mesh);
    vtkPolyData* GetInputMesh();

    void SetOutput(vtkPolyData* mesh);
    vtkPolyData* GetOutput();

    int Update();

protected:
    vtkCGALARAPParametrization();
    ~vtkCGALARAPParametrization();
  
private:
    // needed but not implemented
    vtkCGALARAPParametrization(const vtkCGALARAPParametrization&) = delete;
    void operator=(const vtkCGALARAPParametrization&) = delete;

    vtkPolyData* Input;
    vtkPolyData* Output;

    template <typename SurfaceMesh, typename HalfedgeDescriptor, typename VertexUVMap>
    bool OutputToPolyData(SurfaceMesh& sm, HalfedgeDescriptor bhd, VertexUVMap uv_map, vtkPolyData* polyDataOut);

    double Lambda;
    unsigned int NumberOfIterations;
    double Tolerance;
};
#endif
