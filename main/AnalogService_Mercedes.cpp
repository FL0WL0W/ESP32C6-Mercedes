#include "AnalogService_Mercedes.h"

using namespace Esp32;

#ifdef ANALOGSERVICE_MERCEDES_H
namespace EmbeddedIOServices
{
    AnalogService_Mercedes::AnalogService_Mercedes(Esp32IdfAnalogService *esp32AnalogService, AnalogService_ATTiny427Expander *attinyAnalogService) :
		_esp32AnalogService(esp32AnalogService),
		_attinyAnalogService(attinyAnalogService)
    {
    }
	
	void AnalogService_Mercedes::InitPin(analogpin_t pin)
	{
        switch(pin)
        {
            case 15:
            case 25:
			case 203:
				_attinyAnalogService->InitPin(19);
				break;
            case 26:
				_attinyAnalogService->InitPin(20);
				break;
            case 29:
            case 213:
				_attinyAnalogService->InitPin(1);
				break;
            case 32:
            case 217:
				_attinyAnalogService->InitPin(8);
				break;

            case 103:
				_attinyAnalogService->InitPin(4);
				break;
            case 104:
				_attinyAnalogService->InitPin(2);
				break;
            case 105:
				_attinyAnalogService->InitPin(18);
				break;

            case 45:
			case 204:
				_esp32AnalogService->InitPin(0);
				break;
            case 57:
				_esp32AnalogService->InitPin(21);
				break;
            case 58:
			case 202:
				_esp32AnalogService->InitPin(1);
				break;
		}
	}

	float AnalogService_Mercedes::ReadPin(analogpin_t pin)
	{
        switch(pin)
        {
            case 15:
            case 25:
				return _attinyAnalogService->ReadPin(19);
            case 26:
				return _attinyAnalogService->ReadPin(20);
			case 29:
			case 213:
				return _attinyAnalogService->ReadPin(1);
            case 32:
			case 217:
				return _attinyAnalogService->ReadPin(8);

            case 103:
				return _attinyAnalogService->ReadPin(4);
            case 104:
				return _attinyAnalogService->ReadPin(2);
            case 105:
				return _attinyAnalogService->ReadPin(18);

			case 45:
			case 204:
				return _esp32AnalogService->ReadPin(0);
            case 57:
				return _esp32AnalogService->ReadPin(21);
            case 58:
			case 202:
				return _esp32AnalogService->ReadPin(1);
		}
		return 0;
	}
}
#endif
