#include "mp_general_controller.h"

template<typename Type>
bool ControlHandler::MP_General_Controller<Type>::isInside(const LinAlg::Matrix<Type> &_point)
{
	this->inWitchRegion = -1;
	LinAlg::Matrix<Type> point(_point.getNumberOfRows()+1,1);

	for(uint16_t i = 0; i < _point.getNumberOfRows(); ++i)
		point(i,0) = _point(i,0);

	point(_point.getNumberOfRows(),0) = (Type)(-1);

    for (uint16_t i = 0; i < quantityOfRegions; ++i)
    {
        // std::cout << "A matriz de restricoes: \n"<< this->Restrictions[i] << std::endl;
        // std::cout << "A matriz de pontos: \n"<< point << std::endl;

    	LinAlg::Matrix<Type> H = this->Restrictions[i]*point;

//        std::cout << "Resultado de H: \n"<< point << std::endl;

        if(!any(H))
        {
            this->inWitchRegion = i;
//            std::cout << "Controle na regiao: "<< i+1 << std::endl;
            return true;
        }
    }	
    return false;
}

template<typename Type>
bool ControlHandler::MP_General_Controller<Type>::any(const LinAlg::Matrix<Type> &H){
    for(uint16_t j = 0; j < H.getNumberOfRows(); ++j)
        if(H(j,0) > (Type)(1e-8))
            return true;
    return false;
}

template<typename Type>
std::string ControlHandler::MP_General_Controller<Type>::setRestrictions(std::string restrictions)
{
	uint16_t quantityOfRegions = 0, posOfNRestriction;

	for(uint16_t i = 0; i < restrictions.length(); ++i)
		if(restrictions[i] == 'R')
			quantityOfRegions++;

	this->Restrictions = new LinAlg::Matrix<Type>[quantityOfRegions];

	for(uint16_t i = 0; i < quantityOfRegions; ++i)
    {
        posOfNRestriction = restrictions.find("R");
		this->Restrictions[i] = restrictions.substr(0, posOfNRestriction - 1).c_str();
        restrictions.erase(0, posOfNRestriction + 1);

//         std::cout << "Parametros das Restricoes, Regiao [" << i+1 << "] = \n"<< this->Restrictions[i] << std::endl;
	}

	this->quantityOfRegions = quantityOfRegions;

	return restrictions;
}

template<typename Type>
std::string ControlHandler::MP_General_Controller<Type>::setControllerParameters(std::string controllers)
{
	uint16_t quantityOfRegions = 0, posOfNController;

	for(uint16_t i = 0; i < controllers.length(); ++i)
		if(controllers[i] == 'C')
			quantityOfRegions++;

	this->controllerParameters = new LinAlg::Matrix<Type>[quantityOfRegions];

	for(uint16_t i = 0; i < quantityOfRegions; ++i)
    {
        posOfNController = controllers.find("C");
		this->controllerParameters[i] = controllers.substr(0, posOfNController - 1).c_str();
        controllers.erase(0, posOfNController + 1);


//         std::cout << "Parametros do Controlador, Regiao [" << i+1 << "] = \n" << this->controllerParameters[i] << std::endl;
	}

	this->quantityOfRegions = quantityOfRegions;
    return controllers;
}
