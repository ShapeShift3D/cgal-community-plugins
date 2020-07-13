#ifndef __vtkCGALAbstractUVParametrization_h
#define __vtkCGALAbstractUVParametrization_h

#include <vtkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class VTKCGAL_EXPORT vtkCGALAbstractUVParametrization : public vtkPolyDataAlgorithm
{
public:
	vtkAbstractTypeMacro(vtkCGALAbstractUVParametrization, vtkPolyDataAlgorithm);
	
	vtkPolyData* GetInputMesh();

	typedef enum {
		MANUAL = 1,
		AUTO
	} ScalingModes;

	//@{
	/**
	* Specify Range of acceptable angles
	*/
	vtkGetMacro(ScalingMode, int);
	vtkSetMacro(ScalingMode, int);
	//@}
	virtual void SetScalingModeToManual(void) { this->SetScalingMode(ScalingModes::MANUAL); }
	virtual void SetScalingModeToAuto(void) { this->SetScalingMode(ScalingModes::AUTO); }

	//@{
	/**
	 * Scaling of UV map
	 */
	vtkSetMacro(UVScaling, double);
	vtkGetMacro(UVScaling, double);
	//@}

protected:
	vtkCGALAbstractUVParametrization();
	~vtkCGALAbstractUVParametrization() override;

	virtual int RequestData(
	  vtkInformation* request,
	  vtkInformationVector** inputVector,
	  vtkInformationVector* outputVector
	) override;
	
	virtual int FillInputPortInformation(int port, vtkInformation* info) override;

	bool ScaleUV(vtkPolyData* surface, vtkPolyData* UVSurface);

	int ScalingMode;
	double UVScaling;

private:
	vtkCGALAbstractUVParametrization(const vtkCGALAbstractUVParametrization&) = delete;
	void operator = (const vtkCGALAbstractUVParametrization&) = delete;
};

#endif // __vtkCGALAbstractUVParametrization_h
