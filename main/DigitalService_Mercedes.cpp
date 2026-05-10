#include "DigitalService_Mercedes.h"

using namespace Esp32;

#ifdef DIGITALSERVICE_MERCEDES_H
namespace EmbeddedIOServices
{
    DigitalService_Mercedes::DigitalService_Mercedes(Esp32IdfDigitalService *esp32DigitalService, DigitalService_ATTiny427Expander *attinyDigitalService, ATTiny427_PassthroughService *attinyPassthroughService) :
		_esp32DigitalService(esp32DigitalService),
		_attinyDigitalService(attinyDigitalService),
		_attinyPassthroughService(attinyPassthroughService)
    {
    }
	
	void DigitalService_Mercedes::InitPin(digitalpin_t pin, PinDirection direction)
	{
        switch(pin)
        {
            case 11:
				_attinyDigitalService->InitPin(13, direction);
				break;
            case 22:
				_attinyDigitalService->InitPin(16, direction);
				break;
            case 23:
				_attinyDigitalService->InitPin(12, direction);
				break;
            case 26:
				_esp32DigitalService->InitPin(15, direction);
				if((In26 = (direction == In)))
					_attinyPassthroughService->InitPassthrough(17, 20, false);
				else
					_attinyPassthroughService->InitPassthrough(20, 17, true);
				break;
            case 29:
				_attinyDigitalService->InitPin(1, direction);
				break;
            case 32:
				_attinyDigitalService->InitPin(8, direction);
				break;
            case 44:
				_attinyDigitalService->InitPin(7, direction);
				break;
            case 45:
				_esp32DigitalService->InitPin(0, direction);
				break;
            case 46:
				_attinyDigitalService->InitPin(21, direction);
				break;
            case 48:
				_attinyDigitalService->InitPin(3, direction);
				break;
            case 50:
				_attinyDigitalService->InitPin(9, direction);
				break;
        }
	}
	bool DigitalService_Mercedes::ReadPin(digitalpin_t pin)
	{
        switch(pin)
        {
			case 11:
				return _attinyDigitalService->ReadPin(13);
			case 22:
				return _attinyDigitalService->ReadPin(16);
			case 23:
				return _attinyDigitalService->ReadPin(12);
            case 26:
				if(In26)
					return _esp32DigitalService->ReadPin(20);
				return _attinyDigitalService->ReadPin(17);
			case 29:
				return _attinyDigitalService->ReadPin(1);
			case 32:
				return _attinyDigitalService->ReadPin(8);
			case 44:
				return _attinyDigitalService->ReadPin(7);
			case 45:
				return _esp32DigitalService->ReadPin(0);
			case 46:
				return _attinyDigitalService->ReadPin(21);
			case 48:
				return _attinyDigitalService->ReadPin(3);
			case 50:
				return _attinyDigitalService->ReadPin(9);
        }
		return false;
	}
	void DigitalService_Mercedes::WritePin(digitalpin_t pin, bool value)
	{
        switch(pin)
        {
			case 11:
				return _attinyDigitalService->WritePin(13, value);
			case 22:
				return _attinyDigitalService->WritePin(16, value);
			case 23:
				return _attinyDigitalService->WritePin(12, value);
            case 26:
				return _esp32DigitalService->WritePin(15, !value);
			case 29:
				return _attinyDigitalService->WritePin(1, value);
			case 32:
				return _attinyDigitalService->WritePin(8, value);
			case 44:				
				return _attinyDigitalService->WritePin(7, value);
			case 45:
				return _esp32DigitalService->WritePin(0, value);
			case 46:
				return _attinyDigitalService->WritePin(21, value);
			case 48:
				return _attinyDigitalService->WritePin(3, value);
			case 50:
				return _attinyDigitalService->WritePin(9, value);
        }
	}
	void DigitalService_Mercedes::AttachInterrupt(digitalpin_t pin, callback_t callBack)
	{
        switch(pin)
        {
            case 26:
				if(In26)
					return _esp32DigitalService->AttachInterrupt(15, callBack);
				return _attinyDigitalService->AttachInterrupt(20, callBack);
            case 29:
				return _attinyDigitalService->AttachInterrupt(1, callBack);
            case 32:
				return _attinyDigitalService->AttachInterrupt(8, callBack);
        }
	}
	void DigitalService_Mercedes::DetachInterrupt(digitalpin_t pin)
	{
        switch(pin)
        {
            case 26:
				_esp32DigitalService->DetachInterrupt(15);
				return _attinyDigitalService->DetachInterrupt(20);
            case 29:
				return _attinyDigitalService->DetachInterrupt(1);
			case 32:
				return _attinyDigitalService->DetachInterrupt(8);
        }
	}
}
#endif
