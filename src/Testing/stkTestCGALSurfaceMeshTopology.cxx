#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include "vtkArcSource.h"
#include "vtkCellArray.h"
#include "vtkFloatArray.h"
#include "vtkPolyData.h"
#include "vtkPointData.h"
#include "vtkTriangleFilter.h"
#include "vtkTubeFilter.h"
#include "vtkTestUtilities.h"

#include "stkCGALSurfaceMeshTopology.h"

// Use the following command in Windows Powershell from SpecifX's Build folder to run this test:
// .\bin\stkCGALModuleTests.exe "stkTestCGALSurfaceMeshTopology" "-D" "C:/dev/Build/SpecifxSoftware-RelWithDebInfo/ExternalData" 
// CMake variable PARAVIEW_BUILD_TESTING must be set to ON to be able to run this test.

int stkTestCGALSurfaceMeshTopology(int argc, char **const argv)
{
    doctest::Context context;

    // https://github.com/onqtam/doctest/blob/4d8716f1efc1d14aa736ef52ee727bd4204f4c40/doc/markdown/commandline.md
    context.setOption("force-colors", true);
    context.setOption("duration", true);

    // Command line can be used to override above parameters.
    context.applyCommandLine(argc, argv);

    return context.run();
}

TEST_CASE("filter should return the shortest loop") {
    vtkNew<vtkArcSource> arc;
    vtkNew<vtkTubeFilter> tubeFilter;
    vtkNew<vtkTriangleFilter> triFilter;

    // Donut on XY plane
    arc->UseNormalAndAngleOn();
    arc->SetPolarVector(-1, 0, 0);
    arc->SetNormal(0, 0, 1);
    arc->SetAngle(360);
    arc->SetResolution(32);

    arc->Update();
    vtkPolyData *arcP = arc->GetOutput();

    vtkNew<vtkFloatArray> radii;
    radii->SetNumberOfValues(arcP->GetNumberOfPoints());
    radii->SetName("radius");

    for(auto i=0; i < radii->GetNumberOfValues(); i++) {
        const float s = (float)i / (radii->GetNumberOfValues()-1) * 2 * M_PI;
        const float value = 0.05 * cos(s) + .3 ;
        radii->SetTuple(i, &value);
    }

    arcP->GetPointData()->AddArray(radii);
    arcP->GetPointData()->SetActiveScalars("radius");

    tubeFilter->SetInputConnection(arc->GetOutputPort());
    tubeFilter->SetNumberOfSides(4);
    tubeFilter->SetVaryRadiusToVaryRadiusByAbsoluteScalar();

    triFilter->SetInputConnection(tubeFilter->GetOutputPort());

    vtkNew<stkCGALSurfaceMeshTopology> cycler;
    cycler->SetInputConnection(triFilter->GetOutputPort());
    cycler->Update();

    vtkPolyData *mesh = triFilter->GetOutput();
    vtkPolyData *output = cycler->GetOutput();
    vtkCellArray *lines = output->GetLines();

    lines->InitTraversal();
    vtkIdType npts;
    const vtkIdType *pts = 0;
    lines->GetNextCell(npts, pts);

    for(auto i=0; i < npts; i++) {
        double *coords = mesh->GetPoint(pts[i]);
        double x = coords[0] - 1;
        double sq_radius = x*x + coords[2]*coords[2];

        REQUIRE_MESSAGE(coords[1] == doctest::Approx(0), "shortest cycle must be on y = 0 plane");
        REQUIRE_MESSAGE(sq_radius == doctest::Approx(0.25*0.25), "the shortest cycle must be a circle");
    }
}
