/**
 * @class stkCGALRegionFairingOperator
 * @brief Fairs a region on a triangle mesh.
 * 
 * The points of the selected vertices are relocated to yield an as-smooth-as-possible surface patch. The parameter fairing_continuity gives the ability to control the tangential continuity Cn of the output mesh.
 * The region described by vertices might contain multiple disconnected components. Note that the mesh connectivity is not altered in any way, only vertex locations get updated.
 * Fairing might fail if fixed vertices, which are used as boundary conditions, do not suffice to solve constructed linear system.
 * Note that if the vertex range to which fairing is applied contains all the vertices of the triangle mesh, fairing does not fail, but the mesh gets shrinked to origin.
 *
 * @sa
 * stkCGALRegionFairingOperatorInterface
 */
#pragma once

#include <stkCGALModule.h>
#include <stkCGALRegionFairingOperatorInterface.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALRegionFairingOperator : public stkCGALRegionFairingOperatorInterface
{
public:
  static stkCGALRegionFairingOperator* New();
  vtkTypeMacro(stkCGALRegionFairingOperator, stkCGALRegionFairingOperatorInterface);

protected:
  stkCGALRegionFairingOperator() = default;
  ~stkCGALRegionFairingOperator() = default;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALRegionFairingOperator(const stkCGALRegionFairingOperator&) = delete;
  void operator=(const stkCGALRegionFairingOperator&) = delete;
};
