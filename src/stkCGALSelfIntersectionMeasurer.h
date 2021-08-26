#ifndef stkCGALSelfIntersectionMeasurer_h
#define stkCGALSelfIntersectionMeasurer_h

#include "vtkPolyDataAlgorithm.h" 
#include <stkCGALModule.h>

#include <string>

class vtkPolyData;
class vtkUnstructuredGrid;

class STKCGAL_EXPORT stkCGALSelfIntersectionMeasurer : public vtkPolyDataAlgorithm
{
public:
    static stkCGALSelfIntersectionMeasurer* New();
    vtkTypeMacro(stkCGALSelfIntersectionMeasurer, vtkPolyDataAlgorithm);
 
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
     * Checks self-intersections independently for every geometrically non-connected mesh of the polydata.
     * Fixes some issues where you have multiple disjoint zones.
     */
    vtkSetMacro(IterateByConnectivity, bool);
    vtkGetMacro(IterateByConnectivity, bool);
    vtkBooleanMacro(IterateByConnectivity, bool);
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
    stkCGALSelfIntersectionMeasurer();
    ~stkCGALSelfIntersectionMeasurer();

    int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;
  
private:
    // needed but not implemented
    stkCGALSelfIntersectionMeasurer(const stkCGALSelfIntersectionMeasurer&) = delete;
    void operator=(const stkCGALSelfIntersectionMeasurer&) = delete;

    int ExecuteSelfIntersect(vtkPolyData* polyDataIn, vtkPolyData* polyDataOut);

    std::string SelfIntersectionsArrayName;
    bool IterateByConnectivity;
    bool PrintSelfIntersectingPairs;
};
#endif
