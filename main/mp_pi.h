#ifndef __MPPI_H_INCLUDED
#define __MPPI_H_INCLUDED

#include "matrix.h"
#include "mp_general_controller.h"

namespace ControlHandler{
	template <typename Type>
    class MP_PI : public MP_General_Controller<Type>
    {
    public:
        MP_PI(){this->step = 0.1; error = 1.0; integralError = 1.0;}

        Type OutputControl(Type Reference, Type SignalInput);

    protected:
    	Type step, error, integralError, PIout, kp, ki, kd, bias;
    	
    	void intError(void);
	};
}

#include "mp_pi.hpp"
#endif
