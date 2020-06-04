#ifndef vtkCGALSelfIntersectionMeasurer_h
#define vtkCGALSelfIntersectionMeasurer_h

#include "vtkPolyDataAlgorithm.h" 
#include <vtkCGALModule.h>

#include <string>

class vtkPolyData;
class vtkUnstructuredGrid;

class VTKCGAL_EXPORT vtkCGALSelfIntersectionMeasurer : public vtkPolyDataAlgorithm
{
public:
    static vtkCGALSelfIntersectionMeasurer* New();
    vtkTypeMacro(vtkCGALSelfIntersectionMeasurer, vtkPolyDataAlgorithm);
 
    vtkPolyData* GetInputMesh();

    //@{
    /**
    * Name of the Self-Intersections array.
    */
    vtkSetMacro(SelfIntersectionsArrayName, std::string);
    vtkGetMacro(SelfIntersectionsArrayName, std::string);
    //@}

    //@{
    /**
     * Print pairs of triangles self-intersecting themselves to the console output.
     */
    vtkSetMacro(PrintSelfIntersectingPairs, bool);
    vtkGetMacro(PrintSelfIntersectingPairs, bool);
    vtkBooleanMacro(PrintSelfIntersectingPairs, bool);
    //@}

protected:
    vtkCGALSelfIntersectionMeasurer();
    ~vtkCGALSelfIntersectionMeasurer();

    int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;
  
private:
    // needed but not implemented
    vtkCGALSelfIntersectionMeasurer(const vtkCGALSelfIntersectionMeasurer&) = delete;
    void operator=(const vtkCGALSelfIntersectionMeasurer&) = delete;

    std::string SelfIntersectionsArrayName;
    bool PrintSelfIntersectingPairs;
};
#endif
