#ifndef MPGENERALCONTROLLER_H_INCLUDED
#define MPGENERALCONTROLLER_H_INCLUDED

#include "matrix.h"

namespace ControlHandler{
	template <typename Type>
    class MP_General_Controller
    {
    public:
    	MP_General_Controller(){}

    	std::string setRestrictions(std::string restrictions);
        std::string setControllerParameters(std::string controllers);
    	virtual Type OutputControl(Type Reference, Type SignalInput) = 0;

    protected:
    	LinAlg::Matrix<Type> *Restrictions;
    	LinAlg::Matrix<Type> *controllerParameters;
    	uint16_t quantityOfRegions;
    	int inWitchRegion;

    	bool isInside(const LinAlg::Matrix<Type> &_point);
        bool any(const LinAlg::Matrix<Type> &H);
	};
}

#include "mp_general_controller.hpp"
#endif