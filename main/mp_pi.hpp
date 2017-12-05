#include "mp_pi.h"

template<typename Type>
void ControlHandler::MP_PI<Type>::intError()
{
    this->integralError += this->error*this->step;
}

template<typename Type>
Type ControlHandler::MP_PI<Type>::OutputControl(Type Reference, Type SignalInput)
{
    this->error = Reference - SignalInput;

    this->intError();

    LinAlg::Matrix<Type> state(2,1); state(0,0) = this->integralError;
    state(1,0) = this->error;
//    std::cout << "Estado interno: \n" << state(0,0) << ';' << state(1,0) << std::endl;
    if(this->isInside(state))
    {
    	this->ki   = this->controllerParameters[this->inWitchRegion](0,0);
    	this->kp   = this->controllerParameters[this->inWitchRegion](0,1);
        this->bias = this->controllerParameters[this->inWitchRegion](0,2);

    	this->PIout = (this->kp*this->error + this->ki*this->integralError + this->bias);
    }
    else
    	std::cout << "Valor enviado nao pertence ao conjunto de restricoes!" << std::endl;

    return this->PIout;
}

