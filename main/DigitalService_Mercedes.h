#include "Esp32IdfDigitalService.h"
#include "DigitalService_ATTiny427Expander.h"
#include "ATTiny427_PassthroughService.h"

#ifndef DIGITALSERVICE_MERCEDES_H
#define DIGITALSERVICE_MERCEDES_H
namespace EmbeddedIOServices
{
	class DigitalService_Mercedes : public IDigitalService
	{
	protected:
		Esp32::Esp32IdfDigitalService * const _esp32DigitalService;
		DigitalService_ATTiny427Expander * const _attinyDigitalService;
		ATTiny427_PassthroughService * const _attinyPassthroughService;
		bool In26 : 1;
		bool In6 : 1;
		bool In7 : 1;
		bool In16 : 1;
	public:
		DigitalService_Mercedes(Esp32::Esp32IdfDigitalService *esp32DigitalService, DigitalService_ATTiny427Expander *attinyDigitalService, ATTiny427_PassthroughService *attinyPassthroughService);
		void InitPin(digitalpin_t pin, PinDirection direction);
		bool ReadPin(digitalpin_t pin);
		void WritePin(digitalpin_t pin, bool value);
		void AttachInterrupt(digitalpin_t pin, callback_t callBack);
		void DetachInterrupt(digitalpin_t pin);
	};
}
#endif
