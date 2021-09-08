/**
 * @class stkPointCloudScalarSizingField
 * @brief Sizing field based on a point cloud
 *
 * @sa
 * stkPointCloudScalarSizingField
 */

// VTK
#include <vtkPointData.h>
#include <vtkPointSet.h>
#include <vtkPolyData.h>
#include <vtkProbeFilter.h>
#include <vtkSmartPointer.h>
#include <vtkStaticPointLocator.h>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Labeled_mesh_domain_3.h>

#include <string>

/**
 * @ingroup stkCGAL
 *
 */
namespace stk
{
struct PointCloudScalarSizingField
{

  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::FT FT;
  typedef CGAL::Labeled_mesh_domain_3<K> Mesh_domain;
  typedef Mesh_domain::Index Index;

public:
  PointCloudScalarSizingField()
  {
    this->PointLocator = vtkSmartPointer<vtkStaticPointLocator>::New();

    this->UseDefaultCellSize = false;
    this->DefaultCellSize = 1;
  }

  ~PointCloudScalarSizingField() = default;

  void SetSizingFieldArray(vtkDataArray* array) { this->SizingFieldArray = array; }

  vtkDataArray* GetSizingFieldArray() { return this->SizingFieldArray; }

  void SetPointCloud(vtkPointSet* pointCloud)
  {
    this->PointCloud = pointCloud;
    this->PointLocator->SetDataSet(pointCloud);
    this->PointLocator->BuildLocator();
  }

  vtkPointSet* GetPointCloud() { return this->PointCloud; }

  void SetDefaultCellSize(double cellSize) { this->DefaultCellSize = cellSize; }

  void SetUseDefaultCellSize(bool useDefaultCellSize)
  {
    this->UseDefaultCellSize = useDefaultCellSize;
  }

  FT operator()(const K::Point_3& p, const int, const Index&) const
  {
    if (this->UseDefaultCellSize)
    {
      return DefaultCellSize;
    }

    vtkIdType Id = this->PointLocator->FindClosestPoint(p[0], p[1], p[2]);

    return SizingFieldArray->GetTuple1(Id);
  }

private:
  vtkDataArray* SizingFieldArray;
  vtkPointSet* PointCloud;
  vtkSmartPointer<vtkStaticPointLocator> PointLocator;
  bool UseDefaultCellSize;
  double DefaultCellSize;
};
};
