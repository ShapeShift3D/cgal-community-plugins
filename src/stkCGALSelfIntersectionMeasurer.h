#pragma once

#include <stkCGALSelfIntersectionMeasurerInterface.h>
#include <stkCGALModule.h>

class STKCGAL_EXPORT stkCGALSelfIntersectionMeasurer : public stkCGALSelfIntersectionMeasurerInterface
{
public:
    static stkCGALSelfIntersectionMeasurer* New();
    vtkTypeMacro(stkCGALSelfIntersectionMeasurer, stkCGALSelfIntersectionMeasurerInterface);

protected:
    stkCGALSelfIntersectionMeasurer() = default;
    ~stkCGALSelfIntersectionMeasurer() = default;

    virtual int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;
  
private:
    stkCGALSelfIntersectionMeasurer(const stkCGALSelfIntersectionMeasurer&) = delete;
    void operator=(const stkCGALSelfIntersectionMeasurer&) = delete;

    int ExecuteSelfIntersect(vtkPolyData* polyDataIn, vtkPolyData* polyDataOut);
};
