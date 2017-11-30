#ifndef __MPPID_H_INCLUDED
#define __MPPID_H_INCLUDED

#include "matrix.h"

namespace ControlHandler{
	template <typename Type>
    class MP_PID
    {
	public:
		MP_PID();

    	Type OutputControl(Type Reference, Type SignalInput);
    	std::string setRestrictions(std::string restrictions);
    	void setControllerParameters(std::string controllers);

    private:
    	LinAlg::Matrix<Type> *Restrictions;
    	LinAlg::Matrix<Type> *controllerParameters;
    	Type step, error, pastError, integralError, derivativeError, PIDout, kp, ki, kd, bias, inWitchRegion;
    	uint16_t quantityOfRegions;

    	void intError(void);
        void difError(void);
        bool isInside(void);
	};
}


#endif