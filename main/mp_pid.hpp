template<typename Type>
void ControlHandler::PID<Type>::intError()
{
    this->integralError += this->Error*this->step;
}

template<typename Type>
void ControlHandler::PID<Type>::difError()
{
    this->derivativeError = ((this->Error - this->pastError)/this->step);
    this->pastError = this->Error;
}

template<typename Type>
bool ControlHandler::PID<Type>::isInside()
{
	this->inWitch = -1;
	LinAlg::Matrix<Type> point(4,1);
	point(0,0) = this->integralError;   point(1,0) = this->error; 
	point(2,0) = this->derivativeError; point(3,0) = -1.0;

    for (uint16_t i = 0; i < quantityOfRegions; ++i)
    {
    	uint16_t sum = 0;
    	LinAlg::Matrix<Type> H = Restrictions*point;
    	for(uint16_t j = 0; j < H.getNumberOfRows(); ++j)
        	if(H(j,0) > 1e-8) %o resultado da multiplicação de Pn pelo ponto deve ser menor que uma tolerância, caso todas as linhas do resultado da multiplicação sejam menores que a tolerância, o programa identifica como o ponto pertencente ao polihedro
            	sum++;
        if(H.getNumberOfRows() != sum)
        	continue;
        else
        {
            this->inWitch = i;
            return true;
        }
    }	
}

template<typename Type>
Type ControlHandler::MP_PID<Type>::OutputControl(Type Reference, Type SignalInput)
{
    this->error = Reference - SignalInput;

    this->difError();
    this->intError();

    if(this->isInside())
    {
    	this->ki   = this->controllerParameters[this->inWitch](0,0);
    	this->kp   = this->controllerParameters[this->inWitch](0,1);
    	this->kd   = this->controllerParameters[this->inWitch](0,2);
    	this->bias = this->controllerParameters[this->inWitch](0,3);

    	this->PIDout = (this->kp*this->error + this->ki*this->integralError + this->kd*this->derivativeError + this->bias);
    }
    else
    	std::cout << "Valor enviado nao pertence ao conjunto de restricoes!" << std::endl;

    return this->PIDout;
}

template<typename Type>
std::string ControlHandler::MP_PID<Type>::setRestrictions(std::string restrictions)
{
	uint16_t quantityOfRegions = 0, posOfNRestriction;

	for(uint16_t i = 0; i < restrictions.length(); ++i)
		if(restrictions[i] == 'R')
			quantityOfRegions++;

	this->Restrictions = new LinAlg::Matrix<Type>()[quantityOfRegions];

	for(uint16_t i = 0; i < quantityOfRegions; ++i)
    {
        posOfNRestriction = restrictions.find("R");
		this->Restrictions[i] = restrictions.substr(0, posOfNRestriction - 1).c_str();
        restrictions.erase(0, posOfNRestriction + 1);

        std::cout << this->Restrictions[i] << std::endl;
	}

	this->quantityOfRegions = quantityOfRegions;
	
	return restrictions;
}

template<typename Type>
void ControlHandler::MP_PID<Type>::setControllerParameters(std::string controllers)
{
	uint16_t quantityOfRegions = 0, posOfNController;

	for(uint16_t i = 0; i < controllers.length(); ++i)
		if(controllers[i] == 'C')
			quantityOfRegions++;

	this->controllerParameters = new LinAlg::Matrix<Type>()[quantityOfRegions];

	for(uint16_t i = 0; i < quantityOfRegions; ++i)
    {
        posOfNController = controllers.find("R");
		this->controllerParameters[i] = controllers.substr(0, posOfNController - 1).c_str();
        controllers.erase(0, posOfNController + 1);

        std::cout << this->controllerParameters[i] << std::endl;
	}

	this->quantityOfRegions = quantityOfRegions;
}