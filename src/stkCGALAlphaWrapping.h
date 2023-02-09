/**
 * @class stkCGALAlphaWrapping
 * @brief Generate a valid triangulated surface mesh that strictly contains the input (watertight,
 * intersection-free and 2-manifold).
 *
 * Two user-defined parameters, alpha and offset, offer control over the maximum size of cavities
 * where the shrink-wrapping process can enter, and the tightness of the final surface mesh to the
 * input, respectively. Once combined, these parameters provide a means to trade fidelity to the
 * input for complexity of the output.
 *
 *
 * Inputs: inputMesh (port 0, vtkPointSet)
 * Output: output (port 0, vtkPolyData)
 *
 * @sa
 * stkCGALAlphaWrapping
 */
#pragma once

#include <stkCGALAlphaWrappingInterface.h>
#include <stkCGALModule.h>

/**
 * @ingroup stkCGAL
 *
 */
class STKCGAL_EXPORT stkCGALAlphaWrapping : public stkCGALAlphaWrappingInterface
{
public:
  static stkCGALAlphaWrapping* New();
  vtkTypeMacro(stkCGALAlphaWrapping, stkCGALAlphaWrappingInterface);

protected:
  stkCGALAlphaWrapping() = default;
  ~stkCGALAlphaWrapping() = default;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

private:
  stkCGALAlphaWrapping(const stkCGALAlphaWrapping&) = delete;
  void operator=(const stkCGALAlphaWrapping&) = delete;
};
