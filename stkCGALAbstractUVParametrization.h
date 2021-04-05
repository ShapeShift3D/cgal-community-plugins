#ifndef __stkCGALAbstractUVParametrization_h
#define __stkCGALAbstractUVParametrization_h

#include <stkCGALModule.h>
#include <vtkPolyDataAlgorithm.h>

class STKCGAL_EXPORT stkCGALAbstractUVParametrization : public vtkPolyDataAlgorithm
{
public:
	vtkAbstractTypeMacro(stkCGALAbstractUVParametrization, vtkPolyDataAlgorithm);
	
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
	stkCGALAbstractUVParametrization();
	~stkCGALAbstractUVParametrization() override;

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
	stkCGALAbstractUVParametrization(const stkCGALAbstractUVParametrization&) = delete;
	void operator = (const stkCGALAbstractUVParametrization&) = delete;
};

#endif // __stkCGALAbstractUVParametrization_h
