#ifndef vtkCGALPolygonRecursiveSerialAppender_h
#define vtkCGALPolygonRecursiveSerialAppender_h
// Gives access to macros for communication with the UI
#include "vtkPolyDataAlgorithm.h" 
#include <vtkCGALModule.h>

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALPolygonRecursiveSerialAppender : public vtkPolyDataAlgorithm
{
public:
  // VTK requirements
  static vtkCGALPolygonRecursiveSerialAppender* New();
  vtkTypeMacro(vtkCGALPolygonRecursiveSerialAppender, vtkPolyDataAlgorithm);

  //@{
  /**
  * Specify a boolean mesher.
  */
  void SetBoolean2DMesher(vtkPolyDataAlgorithm* booleanMesher);
  vtkGetObjectMacro(Boolean2DMesher, vtkPolyDataAlgorithm);
  //@}

  virtual int FillInputPortInformation(int, vtkInformation*) override;
  vtkMTimeType GetMTime() override;

protected:
  vtkCGALPolygonRecursiveSerialAppender();
  ~vtkCGALPolygonRecursiveSerialAppender() override;

  virtual int RequestData(vtkInformation*, 
                            vtkInformationVector**, 
                            vtkInformationVector*) override;

  vtkPolyDataAlgorithm* Boolean2DMesher;

private:
  // needed but not implemented
  vtkCGALPolygonRecursiveSerialAppender(const vtkCGALPolygonRecursiveSerialAppender&) = delete;
  void operator=(const vtkCGALPolygonRecursiveSerialAppender&) = delete;
};
#endif
