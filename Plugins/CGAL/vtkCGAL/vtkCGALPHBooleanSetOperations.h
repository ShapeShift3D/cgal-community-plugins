#ifndef vtkCGALPHBooleanSetOperations_h
#define vtkCGALPHBooleanSetOperations_h
// Gives access to macros for communication with the UI
#include "vtkObject.h" 
#include <vtkCGALModule.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>

#include <string>

typedef CGAL::Exact_predicates_exact_constructions_kernel	K;
typedef K::Point_2											Point_2;
typedef CGAL::Polygon_2<K>									Polygon_2;
typedef CGAL::Polygon_with_holes_2<K>						Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>						Pwh_list_2;

// Inherit from the desired filter
class VTKCGAL_EXPORT vtkCGALPHBooleanSetOperations : public vtkObject
{
public:
  // VTK requirements
  static vtkCGALPHBooleanSetOperations* New();
  vtkTypeMacro(vtkCGALPHBooleanSetOperations, vtkObject);

  enum InputModes {
      WITHOUT_HOLE = 1,
      WITH_HOLE
  };

  void SetInputModeToWithoutHole() { InputMode = InputModes::WITHOUT_HOLE; }
  void SetInputModeToWithHole() { InputMode = InputModes::WITH_HOLE; }
  
  void SetPolygonWithHoleAInput(Polygon_with_holes_2* polygonA);
  void SetPolygonAInput(Polygon_2* pA);
  void SetPolygoneWithHoleBInput(Polygon_with_holes_2* polygonB);
  void SetPolygonBInput(Polygon_2* pB);
  void SetPolygonWithHoleListOutput(Pwh_list_2* pwhList);

  enum OperationModes {
      COMPLEMENT = 1,
      INTERSECTION,
      JOIN,
      DIFFERENCE,
      SYMMETRIC_DIFFERENCE
  };

  vtkGetMacro(OperationMode, int);
  vtkSetMacro(OperationMode, int);

  void SetOperationModeToIntersection() { OperationMode = OperationModes::INTERSECTION; }
  void SetOperationModeToJoin() { OperationMode = OperationModes::JOIN; }
  void SetOperationModeToSymmetricDifference() { OperationMode = OperationModes::SYMMETRIC_DIFFERENCE; }

  int Update();
  template<typename InputAType, typename InputBType>
  int Operate(InputAType* polygonA, InputBType* polygonB);

protected:
  vtkCGALPHBooleanSetOperations();
  ~vtkCGALPHBooleanSetOperations(){}

  int OperationMode;
  int InputMode;

private:
  // needed but not implemented
  vtkCGALPHBooleanSetOperations(const vtkCGALPHBooleanSetOperations&) = delete;
  void operator=(const vtkCGALPHBooleanSetOperations&) = delete;

  template<typename InputType>
  int VerifyInput(InputType* input, std::string inputName);

  Polygon_with_holes_2* PwhA;
  Polygon_2* PA;
  Polygon_with_holes_2* PwhB;
  Polygon_2* PB;
  Pwh_list_2* PwhListOut;
};
#endif
