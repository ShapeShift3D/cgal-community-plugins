#ifndef vtkCGALNefPolyhedronTest_h
#define vtkCGALNefPolyhedronTest_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <vtkCGALModule.h>
#include <CGAL/Exact_integer.h>
#include <CGAL/Filtered_extended_homogeneous.h>
#include <CGAL/Nef_polyhedron_2.h>

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALNefPolyhedronTest : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGALNefPolyhedronTest* New();
  vtkTypeMacro(vtkCGALNefPolyhedronTest, vtkPolyDataAlgorithm);
  
  vtkPolyData* GetInputMeshA();
  vtkPolyData* GetInputMeshB();

protected:
  vtkCGALNefPolyhedronTest();
  ~vtkCGALNefPolyhedronTest(){}

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  //typedef CGAL::Exact_integer RT;
  //typedef CGAL::Filtered_extended_homogeneous<RT> Extended_kernel;
  //typedef CGAL::Nef_polyhedron_2<Extended_kernel> Nef_polyhedron;
  //void explore(std::string s, const Nef_polyhedron& poly);

private:
  // needed but not implemented
  vtkCGALNefPolyhedronTest(const vtkCGALNefPolyhedronTest&) = delete;
  void operator=(const vtkCGALNefPolyhedronTest&) = delete;
};
#endif
