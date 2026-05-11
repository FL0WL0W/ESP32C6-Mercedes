#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "rom/crc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include "string.h"
#include "esp_task_wdt.h"
#include "esp_task_wdt.h"
#include <time.h>
#include <sys/time.h>
#include "esp_random.h"
#include "EmbeddedIOServiceCollection.h"
#include <string.h>

using namespace EmbeddedIOServices;
using namespace EmbeddedIOOperations;

uint32_t MercedesCANIDs[] = {
    0x001,
    0x002,
    0x003,
    0x004,
    0x005,
    0x00E,
    0x00F,
    0x015,
    0x029,
    0x045,
    0x05F,
    0x069,
    0x06D,
    0x073,
    0x0D3,
    0x0D5,
    0x0Dd,
    0x0DE,
    0x0F1,
    0x0F3,
    0x0F4,
    0x0F9,
    0x0FE,
    0x0FF,
    0x101,
    0x104,
    0x105,
    0x12D,
    0x14B,
    0x175,
    0x17B,
    0x17d,
    0x18A,
    0x19F,
    0x1B9,
    0x1Bd,
    0x1C1,
    0x1CD,
    0x200,
    0x201,
    0x202,
    0x203,
    0x204,
    0x205,
    0x206,
    0x207,
    0x208,
    0x21A,
    0x245,
    0x247,
    0x283,
    0x285,
    0x2E2,
    0x2E5,
    0x2E9,
    0x2EB,
    0x2F0,
    0x2F1,
    0x2F3,
    0x2F5,
    0x2F6,
    0x2F7,
    0x2F8,
    0x2FE,
    0x2FF,
    0x302,
    0x305,
    0x306,
    0x30A,
    0x30D,
    0x30E,
    0x319,
    0x321,
    0x32F,
    0x336,
    0x33D,
    0x349,
    0x34E,
    0x353,
    0x357,
    0x375,
    0x378,
    0x379,
    0x37A,
    0x37D,
    0x381,
    0x389,
    0x38D,
    0x39D,
    0x39F,
    0x3A0,
    0x3A6,
    0x3A8,
    0x3AB,
    0x3AF,
    0x3B5,
    0x3C5,
    0x3D0,
    0x3D4,
    0x3D9,
    0x3E1,
    0x3E5,
    0x3E7,
    0x3F1,
    0x400,
    0x401,
    0x402,
    0x403,
    0x404,
    0x405,
    0x406,
    0x407,
    0x408,
    0x409,
    0x41C,
    0x41E,
    0x41D,
    0x41F,
    0x429,
    0x42F,
    0x435,
    0x49D,
    0x49E,
    0x49F,
    0x4AF,
    0x51D,
    0x51F,
    0x52F,
    0x584,
    0x586,
    0x588,
    0x59D,
    0x59E,
    0x59F,
    0x5AF,
    0x600,
    0x6EA,
    0x6EC,
    0x6F2,
    0x6FA,
    0x6FC,
    0x77A,
    0x77C
};

bool MercedesID(uint32_t id)
{
    for(int i = 0; i < sizeof(MercedesCANIDs); i++)
        if(id == MercedesCANIDs[i]) return true;
    return false;
}

typedef enum enum_TorqueInterventionType {
    TorqueInterventionType_None = 0,
    TorqueInterventionType_Reduce = 1,
    TorqueInterventionType_Increase = 2,
    TorqueInterventionType_NA = 4
} enum_TorqueInterventionType;

typedef enum {
    HeatingCutoffValve_Close = 0,
    HeatingCutoffValve_Open = 1,
    HeatingCutoffValve_Lock = 2,
    HeatingCutoffValve_NA = 3 
} HeatingCutoffValve_t;

typedef enum {
    EngineRunStatus_Stop = 0,
    EngineRunStatus_Start = 1,
    EngineRunStatus_Idle_Unstable = 2,
    EngineRunStatus_Idle_Stable = 3,
    EngineRunStatus_Run_NoRPMLimit = 4,
    EngineRunStatus_Run_RPMLimited = 5,
    EngineRunStatus_NotDefined = 6,
    EngineRunStatus_NA = 7
} EngineRunStatus_t;

typedef enum TCCStatus {
    TCCStatus_NotDefined0 = 0,
    TCCStatus_Engaged = 1,
    TCCStatus_Disengaged = 2,
    TCCStatus_NotDefined1 = 3,
    TCCStatus_Slipping = 4,
    TCCStatus_Engaged_Slipping = 5,
    TCCStatus_Disengaged_Slipping = 6,
    TCCStatus_NotDefined7 = 7
} TCCStatus_t;

typedef enum {
    TransmissionSelectorPosition_P = 0,
    TransmissionSelectorPosition_R = 1,
    TransmissionSelectorPosition_N = 2,
    TransmissionSelectorPosition_D = 4,
    TransmissionSelectorPosition_NA = 7
} TransmissionSelectorPosition_t;

typedef enum enum_VehicleDrivingProgram {
    VehicleDrivingProgram_Sport = 0,
    VehicleDrivingProgram_Comfort = 1,
    VehicleDrivingProgram_NotDefined2 = 2,
    VehicleDrivingProgram_NotDefined3 = 3,
    VehicleDrivingProgram_NotDefined4 = 4,
    VehicleDrivingProgram_NotDefined5 = 5,
    VehicleDrivingProgram_Offroad = 6,
    VehicleDrivingProgram_NA = 7,
} VehicleDrivingProgram_t;

typedef enum {
    TransmissionDrivingPosition_M1 = 1,
    TransmissionDrivingPosition_M2 = 2,
    TransmissionDrivingPosition_M3 = 3,
    TransmissionDrivingPosition_M4 = 4,
    TransmissionDrivingPosition_M5 = 5,
    TransmissionDrivingPosition_M6 = 6,
    TransmissionDrivingPosition_M7 = 7,
    TransmissionDrivingPosition_Blank = 32,
    TransmissionDrivingPosition_D1 = 49,
    TransmissionDrivingPosition_D2 = 50,
    TransmissionDrivingPosition_D3 = 51,
    TransmissionDrivingPosition_D4 = 52,
    TransmissionDrivingPosition_D5 = 53,
    TransmissionDrivingPosition_D6 = 54,
    TransmissionDrivingPosition_D7 = 55,
    TransmissionDrivingPosition_A = 65,
    TransmissionDrivingPosition_D = 68,
    TransmissionDrivingPosition_F = 70,
    TransmissionDrivingPosition_N = 78,
    TransmissionDrivingPosition_P = 80,
    TransmissionDrivingPosition_R = 82,
    TransmissionDrivingPosition_NA = 255,
} TransmissionDrivingPosition_t;

typedef enum {
    ShiftRecommendation_Idle = 0,
    ShiftRecommendation_Up = 1,
    ShiftRecommendation_Down = 2,
    ShiftRecommendation_NA = 3
} ShiftRecommendation_t;

typedef enum {
    GearTargetDisplay_Blank = 32,
    GearTargetDisplay_G1 = 49,
    GearTargetDisplay_G2 = 50,
    GearTargetDisplay_G3 = 51,
    GearTargetDisplay_G4 = 52,
    GearTargetDisplay_G5 = 53,
    GearTargetDisplay_G6 = 54,
    GearTargetDisplay_G7 = 55,
    GearTargetDisplay_G8 = 56,
    GearTargetDisplay_F = 70,
    GearTargetDisplay_NA = 255
} GearTargetDisplay_t;

typedef enum enum_Gear {
    Gear_N = 0,
    Gear_D1 = 1,
    Gear_D2 = 2,
    Gear_D3 = 3,
    Gear_D4 = 4,
    Gear_D5 = 5,
    Gear_D6 = 6,
    Gear_D7 = 7,
    Gear_D_CVT = 8,
    Gear_R_CVT = 9,
    Gear_R_3 = 10,
    Gear_R = 11,
    Gear_R_2 = 12,
    Gear_P = 13,
    Gear_Abort = 14,
    Gear_NA = 15
} Gear_t;

typedef enum {
    IgnitionSwitchState_Lock = 0,
    IgnitionSwitchState_Off = 1,
    IgnitionSwitchState_Acc = 2,
    IgnitionSwitchState_On = 4,
    IgnitionSwitchState_Start = 5,
    IgnitionSwitchState_NA = 7,
} IgnitionSwitchState_t;

typedef enum {
    TransmissionDrivingProgram_Economy = 69,
    TransmissionDrivingProgram_Sport = 83,
} TransmissionDrivingProgram_t;

float TorqueLoss = 0;
float DrivelineRatio = 0;
float TransmissionOilTemp = 0; //C
float EngineRPM = 0; // RPM
float EngineTorqueMaxCorrectionFactor = 0.421875;
float AcceleratorPedalPosition = 0; // %
float AcceleratorPedalPositionRaw = 0; // %
float AlternatorLoad = 0; // %
float StaticEngineTorque = 0; // N m
float MaximumEngineTorque = 0; // N m
float MinimumEngineTorque = 0; // N m
float DriverSelectedTorque = 0; // N m
float AssistanceSystemSelectedTorque = 0; // N m
float SystemBaseChipSelectedTorque = 0; // N m
float TransmissionDriveshaftTorque = 0; // N m
EngineRunStatus_t EngineRunStatus = EngineRunStatus_Stop;
TCCStatus_t TCCStatus = TCCStatus_Disengaged;
TransmissionSelectorPosition_t TransmissionSelectorPosition = TransmissionSelectorPosition_P;
VehicleDrivingProgram_t VehicleDrivingProgram = VehicleDrivingProgram_Comfort;
TransmissionDrivingPosition_t TransmissionDrivingPosition = TransmissionDrivingPosition_P;
TransmissionDrivingProgram_t TransmissionDrivingProgram = TransmissionDrivingProgram_Economy;
GearTargetDisplay_t GearTargetDisplay = GearTargetDisplay_Blank;
ShiftRecommendation_t ShiftRecommendation = ShiftRecommendation_Idle;
Gear_t Gear = Gear_P;
Gear_t GearTarget = Gear_P;
bool TransmissionBeep = 0;
bool TCMLimp = 0;
bool TCCNoLoad = 0;
bool BasicShiftingProgramOk = 1;
bool HighDriveResistance = 0;
bool ExcessiveTransmissionTemperature = 0;
bool TransmissionOffRoadActive = 0;
bool TransmissionDrivingProgramManualActive = 0;
bool StartBrakeRequest = 0;
bool KickDownSwitchPressed = 0;
bool PreHeat = 1;
bool AlternatorCharging = 0;
bool AdditionalPowerConsumersOnRequest = 0;
bool AcceleratorPedalPositionFault = 0;
bool FullFuelCut = 0;
bool PartFuelCut = 0;
    
bool AuxWaterPumpRequest = 1;
HeatingCutoffValve_t HeatingCutoffValveState = HeatingCutoffValve_Open;
bool ClutchDisengaged = 0;
bool CEL = true;
float CLT = 0; // *C
float IAT = 30; // *C
float OilTemp = 0; // *C
float OilLvl = 80; // mm
float OilQuality = 100; // %
float FuelConsumption = 0; // µl/250ms
float Baro = 100; //kPa
bool ACCompressorOn = false;
bool ACCompressorOnRequest = false;
float ACCompressorMaxTorque = 0; //nm
bool FuelPumpOnRequest = 0;
float DesiredEngineIdleRpm = 0; // RPM
float EngineEfficiency = 100; // %
bool FSCMAlive = 1;
float FuelPressureRequested = 6; // bar
float EngineMinTorque = 0; //nm
float EngineStaticTorque = 0; //nm
float EngineMaxTorque = 0; //nm

float TransmissionTorqueRatio = 0;
float FuelPressure = 0; //bar
float ACCompressorTorque = 0; //nm
bool BrakeApplied = false;
bool ParkBrakeApplied = false;
bool IgnitionOnStartInactive = 0;
float FinalDriveRatio = 3.07;
IgnitionSwitchState_t IgnitionSwitchState = IgnitionSwitchState_Lock;
bool TransmissionTorqueRatioValid = false;
bool FinalDriveRatioValid = false;
bool CruiseCancel = false;
bool CruiseResume = false;
bool CruiseAccelHigh = false;
bool CruiseDeccelHigh = false;
bool CruiseAccelLow = false;
bool CruiseDeccelLow = false;
bool CruiseEnabled = false;
float CruiseSetSpeed = 0;
bool SBWUp = false;
bool SBWDown = false;
bool STWUp = false;
bool STWDown = false;
bool SportSwitch = false;
bool TapShiftEnable = false;
bool ESPEnable = true;
float SteeringWheelAngle = 0;
float SteeringWheelAngleSpeed = 0;
float LateralAcceleration = 0;
float YawRate = 0;
bool ESPDriverIntent = true;
float CurrentSpeed = 0; // kmh
float GovernSpeed = 510; // kmh
enum_TorqueInterventionType TorqueInterventionType = TorqueInterventionType_None;
float TorqueRequestValue = 0;
float TractionControlMaximumTorqueIncreaseRate = 1008;
bool TractionControlSystemActive = false;
bool AntiLockBrakeSystemActive = false;
bool VehicleStabilityEnhancementSystemActive = false;

extern EmbeddedIOServiceCollection _embeddedIOServiceCollection;

void SendCan1(twai_message_t *message) {
    CANIdentifier_t identifier{.CANIdentifier = message->identifier, .CANBusNumber = 0};
    CANData_t data;
    memcpy(data.Data, message->data, message->data_length_code);
    _embeddedIOServiceCollection.CANService->Send(identifier, data, message->data_length_code);
}
void SendCan2(twai_message_t *message) {
    CANIdentifier_t identifier{.CANIdentifier = message->identifier, .CANBusNumber = 1};
    CANData_t data;
    memcpy(data.Data, message->data, message->data_length_code);
    _embeddedIOServiceCollection.CANService->Send(identifier, data, message->data_length_code);
}

uint8_t crcJ1850Table[256];
void CRCJ1850Init(void) {
        uint8_t _crc;
        for (int i = 0; i < 0x100; i++) {
                _crc = i;

                for (uint8_t bit = 0; bit < 8; bit++) _crc = (_crc & 0x80) ? ((_crc << 1) ^ 0x1D) : (_crc << 1);

                crcJ1850Table[i] = _crc;
        }
}
uint8_t CalcJ1850CRC(uint8_t * buf, uint8_t len) {
        const uint8_t * ptr = buf;
        uint8_t _crc = 0xFF;

        while(len--) _crc = crcJ1850Table[_crc ^ *ptr++];

        return ~_crc;
}

bool tutdEnabled = false;
uint8_t gearCommanded = 0;
uint32_t lastCommandGearTS = 0;
uint16_t minUpshiftKph[7] = {
    0,
    5,
    14,
    19,
    24,
    28,
    35
};

uint32_t usec() {
    struct timeval tv;
    gettimeofday(&tv, NULL); 
    return (tv.tv_sec * 1000000) + tv.tv_usec;
}

static void CommandGearUp() {
    if(tutdEnabled) return;
    const uint32_t ts = usec();
    if(lastCommandGearTS - ts < 50000) {
        lastCommandGearTS = ts;
        return;
    }
    lastCommandGearTS = ts;
    if(gearCommanded == 0) {
        gearCommanded = GearTarget;
    }
    if(gearCommanded > 7) {
        gearCommanded = 0;
        return;
    }
    if(CurrentSpeed < minUpshiftKph[gearCommanded-1]) {
        gearCommanded = 0;
        return;
    }
    ++gearCommanded;
}
uint16_t maxDownshiftKph[7] = {
    37,
    60,
    83,
    102,
    136,
    173,
    204
};
static void CommandGearDown() {
    if(tutdEnabled) return;
    const uint32_t ts = usec();
    if(lastCommandGearTS - ts < 50000) {
        lastCommandGearTS = ts;
        return;
    }
    lastCommandGearTS = ts;
    if(gearCommanded == 0) {
        gearCommanded = GearTarget;
    }
    if(gearCommanded == 1) {
        gearCommanded = 1;
        return;
    }
    if(gearCommanded > 8) {
        gearCommanded = 8;
        return;
    }
    if(CurrentSpeed > maxDownshiftKph[gearCommanded-2]) {
        return;
    }
    --gearCommanded;
}

uint8_t CAN105Cnt = 0;
uint8_t CAN1E5Cnt = 0;
uint8_t CAN1CDCnt = 0;
uint8_t CAN14BCnt = 0;
uint8_t CANF3Cnt = 0;
uint8_t CANF1Cnt = 0;
uint8_t WeirdIgnitionThingOnCount = 0;
static void CANTask100Hz(void *pvParameters) 
{
    twai_message_t message;
    message.ss = 0;
    message.self = 0;
    message.dlc_non_comp = 0;

    while(1)
    {
        //Block power steering message
        // message.identifier = 0x340;
        // message.extd = 0;
        // message.rtr = 0;
        // message.data_length_code = 1;
        // message.data[0] = 0x00;
        // message.data[1] = 0x00;
        // message.data[2] = 0x00;
        // message.data[3] = 0x00;
        // message.data[4] = 0x00;
        // message.data[5] = 0x00;
        // message.data[6] = 0x00;
        // message.data[7] = 0x00;
        // SendCan2(&message);
        
        vTaskDelay(pdMS_TO_TICKS(10));

        if(IgnitionSwitchState != IgnitionSwitchState_Off && IgnitionSwitchState != IgnitionSwitchState_Lock) {
            //ENG_RS1_PT
            message.identifier = 0x1CD;
            message.extd = 0;
            message.rtr = 0;
            message.data_length_code = 8;
            const uint16_t driverSelectedTorque = (DriverSelectedTorque - 500) * 4;
            message.data[0] = 0x60 | ((driverSelectedTorque >> 8) & 0x1F);
            message.data[1] = driverSelectedTorque;
            const uint16_t assistanceSystemSelectedTorque = (AssistanceSystemSelectedTorque - 500) * 4;
            message.data[2] = 0x60 | ((assistanceSystemSelectedTorque >> 8) & 0x1F);
            message.data[3] = assistanceSystemSelectedTorque;
            const uint16_t systemBaseChipSelectedTorque = (SystemBaseChipSelectedTorque - 500) * 4;
            message.data[4] = 0x60 | ((systemBaseChipSelectedTorque >> 8) & 0x1F);
            message.data[5] = systemBaseChipSelectedTorque;
            message.data[6] = (CAN1CDCnt++ & 0xF) << 4;
            message.data[7] = CalcJ1850CRC(message.data, 7);
            SendCan2(&message);
            //ENG_RS2_PT
            message.identifier = 0x14B;
            message.extd = 0;
            message.rtr = 0;
            message.data_length_code = 8;
            const uint16_t staticTorque = (StaticEngineTorque - 500) * 4;
            message.data[0] = ((staticTorque >> 8) & 0x1F);
            message.data[1] = staticTorque;
            const uint16_t maxTorque = (MaximumEngineTorque - 500) * 4;
            message.data[2] = ((maxTorque >> 8) & 0x1F);
            message.data[3] = maxTorque;
            const uint16_t minTorque = (MinimumEngineTorque - 500) * 4;
            message.data[4] = ((FullFuelCut & 0x1) << 7) | ((PartFuelCut & 0x1) << 6) | ((minTorque >> 8) & 0x1F);
            message.data[5] = minTorque;
            message.data[6] = (CAN14BCnt++ & 0xF) << 4;
            message.data[7] = CalcJ1850CRC(message.data, 7);
            SendCan2(&message);
            //ENG_RS3_PT
            message.identifier = 0x105;
            message.extd = 0;
            message.rtr = 0;
            message.data_length_code = 8;
            const uint16_t engineRPM = EngineRPM;
            message.data[0] = ((KickDownSwitchPressed & 0x1) << 7) | ((PreHeat & 0x1) << 6) | ((engineRPM >> 8) & 0x3F);
            message.data[1] = engineRPM;
            message.data[2] = EngineTorqueMaxCorrectionFactor * 128;
            message.data[3] = AcceleratorPedalPosition *2.5;
            message.data[4] = AcceleratorPedalPositionRaw * 2.5;
            const uint8_t alternatorLoad = AlternatorLoad * 0.309981401f;
            message.data[5] = ((AlternatorCharging & 0x1) << 7) | ((AdditionalPowerConsumersOnRequest & 0x1) << 6) | (alternatorLoad & 0x3F);
            message.data[6] = ((CAN105Cnt++ & 0xF) << 4) | ((AcceleratorPedalPositionFault & 0x1) << 3) | (EngineRunStatus & 0x7);
            message.data[7] = CalcJ1850CRC(message.data, 7);
            SendCan2(&message);

            //TCM_A1
            message.identifier = 0x2F1;
            message.extd = 0;
            message.rtr = 0;
            message.data_length_code = 8;
            message.data[0] = TransmissionOilTemp + 50;
            message.data[1] =   ((TCCNoLoad & 0x1) << 7) | (TCCStatus << 4) | 
                                ((TCMLimp & 0x1) << 3) | ((BasicShiftingProgramOk & 0x1) << 2) | 
                                ((HighDriveResistance & 0x1) << 1) | (ExcessiveTransmissionTemperature & 0x1);
            message.data[2] =   ((TransmissionOffRoadActive & 0x1) << 7) | (TransmissionSelectorPosition << 4) | 
                                ((TransmissionDrivingProgramManualActive & 0x1) << 3) | VehicleDrivingProgram;
            message.data[3] = (StartBrakeRequest & 0x1) << 7;
            const uint16_t transmissionDriveshaftTorque = TransmissionDriveshaftTorque;
            message.data[4] = transmissionDriveshaftTorque >> 8;
            message.data[5] = transmissionDriveshaftTorque;
            message.data[6] =
            message.data[7] = 0xFF;
            SendCan2(&message);

            //TCM_DISP_RQ
            message.identifier = 0x2F3;
            message.extd = 0;
            message.rtr = 0;
            message.data_length_code = 8;
            message.data[0] = TransmissionDrivingPosition;
            message.data[1] = TransmissionDrivingProgram;
            message.data[2] = ((TransmissionBeep & 0x1) << 7) | (ShiftRecommendation << 5);
            message.data[3] = 0;
            message.data[4] = 0;
            message.data[5] = GearTargetDisplay;
            message.data[6] = 0;
            message.data[7] = 0xFF;
            SendCan2(&message);

            //ENG_RQ2_TCM
            message.identifier = 0xf3;
            message.extd = 0;
            message.rtr = 0;
            message.data_length_code = 8;
            message.data[0] = (Gear & 0xF) | ((GearTarget & 0xF) << 4);
            message.data[1] = 0xFF;
            uint16_t drivelineRatio = DrivelineRatio * 100;
            message.data[2] = drivelineRatio << 8;
            message.data[3] = drivelineRatio;
            uint8_t torqueLoss = TorqueLoss * 4;
            message.data[4] = torqueLoss;
            message.data[5] = 0x0A;
            message.data[6] = (CANF3Cnt++ & 0xF) << 4;
            message.data[7] = CalcJ1850CRC(message.data, 7);
            SendCan2(&message);
        }
        if(IgnitionSwitchState == IgnitionSwitchState_Start) {
            message.identifier = 0x1B9;
            message.extd = 0;
            message.rtr = 0;
            message.data_length_code = 8;
            message.data[0] = 0x41;
            if(WeirdIgnitionThingOnCount > 2)
                message.data[1] = 0x48;
            else {
                WeirdIgnitionThingOnCount++;
                message.data[1] = 0x44;
            }
            message.data[2] = 0;
            message.data[3] = 0;
            message.data[4] = 0;
            message.data[5] = 0;
            message.data[6] = 0;
            message.data[7] = 0;
            SendCan2(&message);
            message.identifier = 0x2F0;
            SendCan2(&message);
        } else {
            WeirdIgnitionThingOnCount = 0;
        }

        //PPEI Fuel System Status 
        message.identifier = 0x1EB;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 2;
        uint16_t fuelPressure = FuelPressure * 100;
        message.data[0] = (fuelPressure >> 8) & 0x3;
        message.data[1] = fuelPressure;
        SendCan1(&message);

        //PPEI Brake Apply Status
        message.identifier = 0xf1;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 4;
        message.data[0] = (CANF1Cnt++ & 0x3) << 4 | (BrakeApplied & 0x1) << 1;
        message.data[0] |= (((~((message.data[0] & 0x3) + ((message.data[0] & 0x30) >> 4)) + 1)) << 2) & 0xC;
        message.data[1] = BrakeApplied? 255 : 0;
        message.data[2] = 0;
        message.data[3] = 0x40 | ((BrakeApplied & 0x1) << 7);
        SendCan1(&message);

        //PPEI Steering Wheel Angle
        message.identifier = 0x1E5;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 8;
        message.data[0] = 0x44;
        uint16_t steeringWheelAngle = SteeringWheelAngle * 16;
        message.data[1] = steeringWheelAngle >> 8;
        message.data[2] = steeringWheelAngle;
        uint16_t steeringWheelAngleSpeed = SteeringWheelAngleSpeed;
        message.data[3] = 0x90 | ((CAN1E5Cnt * 0x3) << 5) | ((steeringWheelAngleSpeed << 8) & 0xF);
        message.data[4] = steeringWheelAngleSpeed;
        uint16_t steeringWheelAngleProtection = (~(steeringWheelAngle + CAN1E5Cnt)) + 1;
        message.data[5] = steeringWheelAngleProtection >> 8;
        message.data[6] = steeringWheelAngleProtection;
        message.data[7] = CAN1E5Cnt;
        SendCan1(&message);
        CAN1E5Cnt++;
    }
}

float prevCruiseSetSpeed = 0;
uint8_t CAN1C7Cnt = 0;
static void period50Hz(void *pvParameters) 
{
    twai_message_t message;
    message.ss = 0;
    message.self = 0;
    message.dlc_non_comp = 0;
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(20));

        bool govern = GovernSpeed < 510 && CurrentSpeed > GovernSpeed;

        //PPEI Chassis Engine Torque Request 1
        message.identifier = 0x1C7;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 8;
        float trv = TorqueRequestValue;
        enum_TorqueInterventionType tit = TorqueInterventionType;
        if(govern) {
            trv = -10;
            if(TorqueRequestValue < trv && tit == TorqueInterventionType_Reduce)
                trv = TorqueRequestValue;
            tit = TorqueInterventionType_Reduce;
        }
        if(trv < -848)
            trv = -848;
        if(trv > 1199)
            trv = 1199;
        uint16_t torqueRequestValue = trv * 2 + 848;
        message.data[0] = tit << 4 | ((torqueRequestValue >> 8) & 0xF);
        message.data[1] = torqueRequestValue;
        const uint16_t torqueRequestProtection = (~(((message.data[0] << 8) | message.data[1]) + (CAN1C7Cnt & 0x3))) + 1;
        message.data[2] = ((CAN1C7Cnt & 0x3) << 6) | ((torqueRequestProtection >> 8) & 0x3F);
        message.data[3] = torqueRequestProtection;
        message.data[4] = 0;
        message.data[5] = 0;
        uint8_t tractionControlMaximumTorqueIncreaseRate = TractionControlMaximumTorqueIncreaseRate;
        message.data[6] = (tractionControlMaximumTorqueIncreaseRate / 16) & 0x3F;
        message.data[7] = 0;
        SendCan1(&message);
        CAN1C7Cnt++;

        //PPEI Chassis General Status 1
        message.identifier = 0x1E9;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 8;
        int16_t lateralAcceleration = LateralAcceleration * 64;
        message.data[0] = ((BrakeApplied & 0x1) << 6) | ((lateralAcceleration >> 8) & 0xF);
        message.data[1] = lateralAcceleration;
        message.data[2] = (gearCommanded > 0? 0x30 : 0x00)| (gearCommanded & 0xF);
        message.data[3] = (((TractionControlSystemActive || govern) & 0x1) << 4) | ((ESPEnable & 0x1) << 3) | ((ESPEnable & 0x1) << 2) | (VehicleStabilityEnhancementSystemActive & 0x1);
        int16_t yawRate = YawRate * 16;
        message.data[4] = ((yawRate >> 8) & 0xF);
        message.data[5] = yawRate;
        message.data[6] = 0;
        message.data[7] = 0;
        SendCan1(&message);

        message.identifier = 0x378;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 8;
        message.data[0] = 0;
        message.data[1] = CruiseSetSpeed;
        message.data[2] = 0;
        message.data[3] = 0;
        message.data[4] = (CruiseEnabled & 0x1) << 5;
        message.data[5] = 0;
        message.data[6] = (CruiseSetSpeed != prevCruiseSetSpeed);
        message.data[7] = 0;
        // SendCan2(&message);
        prevCruiseSetSpeed = CruiseSetSpeed;
    }
}

uint8_t CAN1E1Cnt = 0;
uint8_t CAN1F3Cnt = 0;
bool CruiseOn = false;
static void period33Hz(void *pvParameters) 
{
    twai_message_t message;
    message.ss = 0;
    message.self = 0;
    message.dlc_non_comp = 0;
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(30));

        if(CruiseResume || CruiseAccelLow || CruiseAccelHigh || CruiseDeccelLow || CruiseDeccelHigh)
            CruiseOn = true;

        //PPEI Cruise Control Switch Status
        message.identifier = 0x1E1;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 7;
        // message.data[0] = ((CruiseCancel & 0x1) << 7) | ((CruiseOn & 0x1) << 6) | (((CruiseAccelLow | CruiseAccelHigh) & 0x1) << 5) | (((CruiseDeccelLow | CruiseDeccelHigh) & 0x1) << 4) | (((CruiseAccelLow | CruiseAccelHigh) & 0x1) << 3) | (((CruiseDeccelLow | CruiseDeccelHigh) & 0x1) << 2);
        // message.data[1] = (~(message.data[0] + (CAN1E1Cnt & 0x3))) + 1;
        // message.data[2] = (CAN1E1Cnt & 0x3);
        // message.data[3] = 0;
        // message.data[4] = (SportSwitch & 0x1) << 7 | (CAN1E1Cnt & 0x3) << 5;
        message.data[0] = 0;
        message.data[1] = 0;
        message.data[2] = EngineRPM < 200? 0x04 : 0x00;
        message.data[3] = 0;
        message.data[4] = (CAN1E1Cnt & 0x3);
        uint8_t switchVal = 0;
        if(message.data[2] != 0x04) {
            switchVal = 1;
            if(CruiseCancel) {
                switchVal = 6;
            } else if(CruiseDeccelHigh || CruiseDeccelLow) {
                switchVal = 3;
            } else if(CruiseAccelHigh || CruiseAccelLow || CruiseResume) {
                switchVal = 2;
            }
            if(!CruiseEnabled && usec() % 1000000 > 500000) {
                switchVal = 5;
                // ESP_LOGI("CRUISE", "on press");
            } 
            // else
            //     ESP_LOGI("CRUISE", "btn: %d", switchVal);
        }
        message.data[5] = (switchVal << 4) | ((CAN1E1Cnt & 0x3) << 2);
        switchVal += (CAN1E1Cnt & 0x3);
        switchVal = -switchVal;
        message.data[6] = switchVal << 4;
        SendCan1(&message);
        CAN1E1Cnt++;
        
        //PPEI Platform TransmissionRequests
        message.identifier = 0x1F3;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 3;
        message.data[0] = ((CAN1F3Cnt & 0x3) << 6);// | (SportSwitch & 0x1) << 4;
        message.data[1] = ((CAN1F3Cnt & 0x3) << 6) | ((TapShiftEnable & 0x1) << 5) | ((SBWUp & 0x1) << 2) | ((SBWDown & 0x1) << 3) | (STWUp & 0x1) | ((STWDown & 0x1) << 1);
        message.data[2] = 0;
        SendCan1(&message);
        CAN1F3Cnt++;
    }
}

float slowmovingrandomstandardvalue = 0;
static void period10Hz(void *pvParameters) 
{
    twai_message_t message;
    message.ss = 0;
    message.self = 0;
    message.dlc_non_comp = 0;
    while(1)
    {
        slowmovingrandomstandardvalue *= 0.95;
        slowmovingrandomstandardvalue += (UINT32_MAX/2 - esp_random()) * 0.1f / UINT32_MAX;
        vTaskDelay(pdMS_TO_TICKS(100));

        if(IgnitionSwitchState != IgnitionSwitchState_Off && IgnitionSwitchState != IgnitionSwitchState_Lock) {
            //AVI
            // message.identifier = 0x208;
            // message.extd = 0;
            // message.rtr = 0;
            // message.data_length_code = 8;
            // message.data[0] = 0x3C;
            // message.data[1] = 0x00;
            // message.data[2] = 0x08;
            // message.data[3] = 0x00;
            // message.data[4] = 0x00;
            // message.data[5] = 0x02;
            // message.data[6] = 0x00;
            // message.data[7] = CalcJ1850CRC(message.data, 7);;
            // SendCan2(&message);

            //ECM_TEMP
            message.identifier = 0x30D;
            message.extd = 0;
            message.rtr = 0;
            message.data_length_code = 8;
            message.data[0] = CLT + 40;
            message.data[1] = IAT + 40;
            message.data[2] = OilTemp + 40;
            message.data[3] = OilLvl * 3.175f;
            message.data[4] = OilQuality * 2.55f;
            const uint16_t fuelConsumption = FuelConsumption * 4.615f;
            message.data[5] = fuelConsumption >> 8;
            message.data[6] = fuelConsumption;
            message.data[7] = Baro * 1.283f;
            SendCan2(&message);

            //ECM_INFO1
            message.identifier = 0x349;
            message.extd = 0;
            message.rtr = 0;
            message.data_length_code = 8;
            message.data[0] = ((AuxWaterPumpRequest & 0x1) << 7) | (HeatingCutoffValveState & 0x3);// | 0x7C;        
            // if(ACCompressorMaxTorque * 4 < 0xFE)
            //     message.data[1] = ACCompressorMaxTorque * 4;
            // else
                message.data[1] = 0xFE;
            if(AcceleratorPedalPosition > 75 || EngineRPM > 4000)
                message.data[1] = 0;
            const uint16_t desiredEngineIdleRpm = DesiredEngineIdleRpm;
            message.data[3] = ((ClutchDisengaged & 0x1) << 7) | ((FuelPumpOnRequest & 0x1) << 6) | ((desiredEngineIdleRpm >> 8) & 0x3F);
            message.data[4] = desiredEngineIdleRpm;
            message.data[5] = EngineEfficiency * 2;
            message.data[6] = (FSCMAlive & 0x1) << 7;
            message.data[7] = FuelPressureRequested * 20;
            SendCan2(&message);

            //CEL
            CEL = gpio_get_level(GPIO_NUM_0);
            if(EngineRPM < 200)
                CEL = true;
            message.identifier = 0x33D;
            message.extd = 0;
            message.rtr = 0;
            message.data_length_code = 4;
            message.data[0] = (CEL & 0x1) << 3;
            message.data[1] = 0;
            message.data[2] = 0;
            message.data[3] = 0x3F;
            SendCan2(&message);
        }

        //PPEI Platform General Status
        message.identifier = 0x1F1;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 8;
        if(IgnitionSwitchState == IgnitionSwitchState_Start) {\
            message.data[0] = 0x3B;
            AdditionalPowerConsumersOnRequest = 0;
        } else if(IgnitionSwitchState == IgnitionSwitchState_On) {
            message.data[0] = 0x2E;
            AdditionalPowerConsumersOnRequest = 1;
        } else if(IgnitionSwitchState == IgnitionSwitchState_Acc) {
            message.data[0] = 0x15;
            AdditionalPowerConsumersOnRequest = 1;
        } else {
            message.data[0] = 0x00;
            AdditionalPowerConsumersOnRequest = 0;
        }
        if(ACCompressorOnRequest) {
            message.data[1] = 0x12;
        } else {
            message.data[1] = 0x0A;
        }
        message.data[2] = 0;
        message.data[3] = 0;
        message.data[4] = 0x08 | (ParkBrakeApplied << 4);
        message.data[5] = 0;
        message.data[6] = ACCompressorTorque * 4; // this is a wild ass guess, GM sends back and forth displacement and torque is calculated in ECU. Mercedes calculates torque in AC system and sends back and forth torque values
        message.data[7] = 0x18;
        SendCan1(&message);

        //Antilock_Brake_and_TC_Status_HS
        message.identifier = 0x17D;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 8;
        message.data[0] = (ESPEnable & 0x1) << 2;
        message.data[1] = 0;
        message.data[2] = 0;
        message.data[3] = 0;
        message.data[4] = 0;
        message.data[5] = 0;
        message.data[6] = ESPEnable & 0x1;
        message.data[7] = 0;
        SendCan1(&message);
    }
}

static void period4Hz(void *pvParameters) 
{
    twai_message_t message;
    message.ss = 0;
    message.self = 0;
    message.dlc_non_comp = 0;
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(250));

        if(IgnitionSwitchState != IgnitionSwitchState_Off && IgnitionSwitchState != IgnitionSwitchState_Lock) {
            //0x202 - this value keeps the radio from saying theft protection. 
            //data[0] must be 0x14, I tried 0x00 and 0xFF but those both place it into theft protection
            //I tried 0xFF in data[1] and data[2] and radio didn't care.
            //data[3] is crc
            message.identifier = 0x202;
            message.extd = 0;
            message.rtr = 0;
            message.data_length_code = 4;
            message.data[0] = 0x14;
            message.data[1] = 0x00;
            message.data[2] = 0x00;
            message.data[3] = CalcJ1850CRC(message.data, 3);
            SendCan2(&message);
        }
    }
}

bool elapsedReset = true;
uint32_t prevTimestamp = 0;
uint32_t EngineRunElapsedSeconds = 0;
float CATTemp = -1337;
static void period1Hz(void *pvParameters) 
{
    twai_message_t message;
    message.ss = 0;
    message.self = 0;
    message.dlc_non_comp = 0;
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if(EngineRPM > 200) {
            if(CATTemp != -1337) {
                if(CATTemp < 400)
                    CATTemp += (slowmovingrandomstandardvalue + 1) * 2;
                else if(CATTemp < 600)
                    CATTemp += slowmovingrandomstandardvalue * 2;
                else 
                    CATTemp -= (slowmovingrandomstandardvalue + 1) * 2;
            }
            EngineRunElapsedSeconds++;
        }
        else {
            if(CATTemp != -1337 && CATTemp > IAT)
                CATTemp -= (slowmovingrandomstandardvalue + 1);
            if(IgnitionSwitchState != IgnitionSwitchState_On)
            EngineRunElapsedSeconds = 0;
        }

        uint32_t timestamp = esp_log_timestamp() / 60000;
        if(timestamp < prevTimestamp)
            elapsedReset = true;
        prevTimestamp = timestamp;

        //PPEI Platform Configuration Data
        message.identifier = 0x4E9;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 6;
        message.data[0] = 0xE1;
        message.data[1] = 0x60;
        message.data[2] = timestamp >> 16;
        message.data[3] = timestamp >> 8;
        message.data[4] = timestamp;
        message.data[5] = 0xFF;
        SendCan1(&message);
        elapsedReset = false;

        bool pin50State = _embeddedIOServiceCollection.DigitalService->ReadPin(digitalpin_t(50));
        ESP_LOGI("PIN50", "Pin 50 state: %d", pin50State);
    }
}

uint32_t gearMismatchTimestamp = 0;
uint8_t gearMismatchGearCommanded = 0;
uint32_t CurrentSpeedLastTS = 0;

void GMCanRX3D1(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    ACCompressorMaxTorque = (data.Data[4] & 0x20) != 0? 63.5 : 0;
    FuelConsumption = (((data.Data[4] & 0xF) << 8) | data.Data[5]) * 1.736111f;
    FullFuelCut = (data.Data[2] & 0x40) != 0;
    CruiseSetSpeed = (((data.Data[2] & 0xF) << 8) | data.Data[3]) / 16.0f;
}

void GMCanRX3E9(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    CurrentSpeedLastTS = usec();
    CurrentSpeed = (((data.Data[0] & 0x7F) << 8) | data.Data[1]) * 0.015625f;
}

void GMCanRX4C1(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    PreHeat = (data.Data[0] & 0x01) == 0;
    if((data.Data[0] & 0x80) == 0) {
        Baro = data.Data[1] * 0.5f;
    }
    if((data.Data[0] & 0x40) == 0) {
        CLT = data.Data[2] - 40;
    }
    if((data.Data[0] & 0x20) == 0) {
        if(CATTemp == -1337)
            CATTemp = IAT;
        IAT = data.Data[3] - 40;
    }
}

void GMCanRX4C9(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    if((data.Data[0] & 0xC0) == 0) {
        TransmissionOilTemp = data.Data[1] - 40;
    }
    ExcessiveTransmissionTemperature = (data.Data[0] & 0x3) != 0;
}

void GMCanRX4D1(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    if((data.Data[0] & 0x80) == 0) {
        OilTemp = data.Data[1] - 40;
    }
    // if(data.Data[0] & 0x08) {
    //     OilQuality = 0;
    // } else {
    //     OilQuality = 100;
    // }
    if(data.Data[0] & 0x20 || data.Data[0] & 0x10) {
        OilLvl = 0;
    } else {
        OilLvl = 80;
    }
}

void GMCanRX1A1(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    DesiredEngineIdleRpm = data.Data[5] * 8;
}

void GMCanRX2C3(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    if((data.Data[0] & 0x10) == 0) {
        EngineMaxTorque = (((data.Data[0] & 0xF) << 8) | data.Data[1]) * 0.5 - 848;
    }
    if((data.Data[2] & 0x10) == 0) {
        EngineMinTorque = (((data.Data[2] & 0xF) << 8) | data.Data[3]) * 0.5 - 848;
    }
    if((data.Data[0] & 0x10) == 0 && (data.Data[2] & 0x10) == 0) {
        EngineStaticTorque = (EngineMaxTorque - EngineMinTorque) / 2 + EngineMinTorque;
    } else if((data.Data[0] & 0x10) == 0) {
        EngineStaticTorque = EngineMinTorque = EngineMaxTorque;
    } else if((data.Data[2] & 0x10) == 0) {
        EngineStaticTorque = EngineMaxTorque = EngineMinTorque;
    }
}

void GMCanRX1F5(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    uint8_t tcc = (data.Data[0] >> 5) & 0x7;
    switch(tcc) {
        case 0:
            TCCNoLoad = 1;
            TCCStatus = TCCStatus_Disengaged;
            break;
        case 1:
            TCCNoLoad = 0;
            TCCStatus = TCCStatus_Disengaged_Slipping;
            break;
        case 2:
            TCCNoLoad = 0;
            TCCStatus = TCCStatus_Engaged_Slipping;
            break;
        case 3:
            TCCNoLoad = 0;
            TCCStatus = TCCStatus_Engaged;
            break;
    }
    switch(data.Data[0] & 0xF) {
        case 1:
            Gear = Gear_D1;
            break;
        case 2:
            Gear = Gear_D2;
            break;
        case 3:
            Gear = Gear_D3;
            break;
        case 4:
            Gear = Gear_D4;
            break;
        case 5:
            Gear = Gear_D5;
            break;
        case 6:
            Gear = Gear_D6;
            break;
        case 7:
            Gear = Gear_D7;
            break;
        case 8:
            Gear = Gear_D7;
            break;
        case 0xD:
            Gear = Gear_N;
            break;
        case 0xF:
            Gear = Gear_P;
            break;
    }
    switch(data.Data[1] & 0xF) {
        case 1:
            GearTarget = Gear_D1;
            break;
        case 2:
            GearTarget = Gear_D2;
            break;
        case 3:
            GearTarget = Gear_D3;
            break;
        case 4:
            GearTarget = Gear_D4;
            break;
        case 5:
            GearTarget = Gear_D5;
            break;
        case 6:
            GearTarget = Gear_D6;
            break;
        case 7:
            GearTarget = Gear_D7;
            break;
        case 8:
            GearTarget = Gear_D7;
            break;
    }
    tutdEnabled = (data.Data[5] & 0x3) > 0;
    if(tutdEnabled) {
        gearCommanded = 0;
        TransmissionDrivingProgramManualActive = 1;
        switch(data.Data[2] & 0xF) {
            case 1:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D1;
                GearTargetDisplay = GearTargetDisplay_G1;
                break;
            case 2:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D2;
                GearTargetDisplay = GearTargetDisplay_G2;
                break;
            case 3:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D3;
                GearTargetDisplay = GearTargetDisplay_G3;
                break;
            case 4:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D4;
                GearTargetDisplay = GearTargetDisplay_G4;
                break;
            case 5:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D5;
                GearTargetDisplay = GearTargetDisplay_G5;
                break;
            case 6:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D6;
                GearTargetDisplay = GearTargetDisplay_G6;
                break;
            case 7:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D7;
                GearTargetDisplay = GearTargetDisplay_G7;
                break;
            case 8:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D;
                GearTargetDisplay = GearTargetDisplay_G8;
                TapShiftEnable = false;
                break;
        }
    } else if(gearCommanded != 0) {
        if((data.Data[1] & 0x30) == 0x20) {//downshift
            if(gearCommanded > (data.Data[1] & 0xF)) { //if transmission is downshifting and gearCommanded is higher than what we are downshifting to, update gearCommanded
                ESP_LOGI("Mercedes", "Gear automatic downshift | Commanded:%i\tTransmission Commanded:%i\tTransmission Estimated:%i\tCurrentSpeed%f", gearCommanded, (data.Data[1] & 0xF), (data.Data[0] & 0xF),CurrentSpeed);
                gearCommanded = (data.Data[1] & 0xF);
            }
        } else if ((data.Data[1] & 0x30) == 0x10) {//upshift
            if(gearCommanded < (data.Data[1] & 0xF)) { //if transmission is upshifting and gearCommanded is lower than what we are upshifting to, update gearCommanded
                ESP_LOGI("Mercedes", "Gear automatic upshift | Commanded:%i\tTransmission Commanded:%i\tTransmission Estimated:%i\tCurrentSpeed%f", gearCommanded, (data.Data[1] & 0xF), (data.Data[0] & 0xF),CurrentSpeed);
                gearCommanded = (data.Data[1] & 0xF);
            }
        }
        //if gear commanded mismatch longer than 110ms, update gearCommanded. ideally we should never enter here with the min max limits set correctly
        if(gearCommanded != (data.Data[1] & 0xF)) {
            const uint32_t ts = usec();
            if(gearMismatchTimestamp == 0 || gearCommanded != gearMismatchGearCommanded) {
                gearMismatchTimestamp = ts;
                gearMismatchGearCommanded = gearCommanded;
            } else if(ts - gearMismatchTimestamp > 110000) {
                // ESP_LOGI("Mercedes", "Gear mismatch | Commanded:%i\tTransmission Commanded:%i\tTransmission Estimated:%i\tCurrentSpeed%f", gearCommanded, (data.Data[1] & 0xF), (data.Data[0] & 0xF),CurrentSpeed);
                // gearCommanded = (data.Data[1] & 0xF);
            }
        } else {
            gearMismatchTimestamp = 0;
        }
        TransmissionDrivingProgramManualActive = 1;
        switch(gearCommanded) {
            case 0:
                TransmissionSelectorPosition = TransmissionSelectorPosition_D;
                break;
            case 1:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D1;
                GearTargetDisplay = GearTargetDisplay_G1;
                break;
            case 2:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D2;
                GearTargetDisplay = GearTargetDisplay_G2;
                break;
            case 3:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D3;
                GearTargetDisplay = GearTargetDisplay_G3;
                break;
            case 4:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D4;
                GearTargetDisplay = GearTargetDisplay_G4;
                break;
            case 5:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D5;
                GearTargetDisplay = GearTargetDisplay_G5;
                break;
            case 6:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D6;
                GearTargetDisplay = GearTargetDisplay_G6;
                break;
            case 7:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D7;
                GearTargetDisplay = GearTargetDisplay_G7;
                break;
            case 8:
                TransmissionDrivingPosition = TransmissionDrivingPosition_D;
                GearTargetDisplay = GearTargetDisplay_G8;
                break;
        }
    } else {
        TransmissionDrivingProgramManualActive = 0;
        GearTargetDisplay = GearTargetDisplay_G1;
        TransmissionDrivingPosition = TransmissionDrivingPosition_D;
    }
    switch(data.Data[3] & 0xF) {
        case 1:
            GearTargetDisplay = GearTargetDisplay_Blank;
            TransmissionDrivingPosition = TransmissionDrivingPosition_P;
            break;
        case 2:
            GearTargetDisplay = GearTargetDisplay_Blank;
            TransmissionDrivingPosition = TransmissionDrivingPosition_R;
            break;
        case 3:
            GearTargetDisplay = GearTargetDisplay_Blank;
            TransmissionDrivingPosition = TransmissionDrivingPosition_N;
            break;
    }
    if((data.Data[3] & 0x10) == 0) {
        switch(data.Data[3] & 0xF) {
            case 1:
                TransmissionSelectorPosition = TransmissionSelectorPosition_P;
                GearTarget = Gear_P;
                break;
            case 2:
                TransmissionSelectorPosition = TransmissionSelectorPosition_R;
                break;
            case 3:
                TransmissionSelectorPosition = TransmissionSelectorPosition_N;
                GearTarget = Gear_N;
                break;
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                TransmissionSelectorPosition = TransmissionSelectorPosition_D;
                break;
        }
    }
    // switch((data.Data[5] & 0x1C) >> 2){
    //     case 0:
    //     case 1:
    //         TransmissionDrivingProgram = TransmissionDrivingProgram_Economy;
    //         VehicleDrivingProgram = VehicleDrivingProgram_Comfort;
    //         break;
    //     case 2:
    //         TransmissionDrivingProgram = TransmissionDrivingProgram_Sport;
    //         VehicleDrivingProgram = VehicleDrivingProgram_Sport;
    //         break;
    // }
    if((data.Data[2] & 0x40) != 0) {
        ShiftRecommendation = ShiftRecommendation_Up;
    } else {
        ShiftRecommendation = ShiftRecommendation_Idle;
    }
    if((data.Data[1] & 0x80) == 0) {
        ClutchDisengaged = (data.Data[1] & 0x40) != 0;
    }
    TCMLimp = (data.Data[3] & 0x80) != 0;
}

void GMCanRX1ED(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    FuelPumpOnRequest = (data.Data[0] >> 6) & 0x1;
    FuelPressureRequested = (((data.Data[0] & 0x3) << 8) | data.Data[1]) / 100.0f;
}

void GMCanRX4F1(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    FinalDriveRatioValid = true;
    FinalDriveRatio = (((data.Data[4] & 0x1) << 8) | data.Data[5]) * 0.01f + 2;
}

void GMCanRX0F9(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    TransmissionTorqueRatioValid = (data.Data[0] & 0x80) == 0;
    if(TransmissionTorqueRatioValid) {
        TransmissionTorqueRatio = ((data.Data[0] * 0x7F) << 8 | data.Data[1]) / 255.0f;
        if(FinalDriveRatioValid) {
            DrivelineRatio = TransmissionTorqueRatio * FinalDriveRatio;
        }
    }
}

void GMCanRX1C3(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    if((data.Data[0] & 0xE0) != 0) {
        ESP_LOGI("MERCEDES", "Torque Reduction Failed: %x", (data.Data[0] & 0xE0));
    }
    if((data.Data[2] & 0x10) == 0) {
        SystemBaseChipSelectedTorque = AssistanceSystemSelectedTorque = DriverSelectedTorque = (((data.Data[2] & 0x0F) << 8) | data.Data[3]) * 0.5f - 848;
    }
    if((data.Data[0] & 0x10) == 0 && TransmissionTorqueRatioValid) {
        TransmissionDriveshaftTorque = ((((data.Data[0] & 0x0F) << 8) | data.Data[1]) * 0.5f - 848) * TransmissionTorqueRatio;
    }
    if((data.Data[2] & 0x20) == 0) {
        AcceleratorPedalPosition = data.Data[6] / 2.55f;
    }
    if((data.Data[7] & 0x40) != 0) {
        TransmissionDrivingProgram = TransmissionDrivingProgram_Sport;
        VehicleDrivingProgram = VehicleDrivingProgram_Sport;
    } else {
        TransmissionDrivingProgram = TransmissionDrivingProgram_Economy;
        VehicleDrivingProgram = VehicleDrivingProgram_Comfort;
    }
}

void GMCanRX0C9(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    // ESP_LOGI("CRUISE", "%x", data.Data[3]);
    CruiseEnabled = (data.Data[3] & 0x20) != 0;
    if(data.Data[0] & 0x40) {
        EngineRunStatus = EngineRunStatus_Start;
    } else if(data.Data[0] & 0x04) {
        if(data.Data[0] & 0x01) {
            EngineRunStatus = EngineRunStatus_Idle_Unstable;
        } else {
            EngineRunStatus = EngineRunStatus_Idle_Stable;
        }
    } else if(data.Data[0] & 0x80) {
        EngineRunStatus = EngineRunStatus_Run_NoRPMLimit;
    }
    EngineRPM = ((data.Data[1] << 8) | data.Data[2]) * 0.25f;
    if((data.Data[3] & 0x80) == 0) {
        AcceleratorPedalPositionFault = (data.Data[3] & 0x80) == 0x80;
        AcceleratorPedalPositionRaw = data.Data[4] / 2.55f;
        if(AcceleratorPedalPositionRaw > 95) { 
            KickDownSwitchPressed = true;
        } else {
            KickDownSwitchPressed = false;
        }
    }
}

void GMCanRX3F9(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    AlternatorCharging = (data.Data[0] & 0x20) == 0;
    if((data.Data[0] & 0x40) == 0) {
        AlternatorLoad = data.Data[3] / 2.55f;
    }
    OilQuality = data.Data[6] / 2.55f;
}


float WheelFLRPM = 0;
float WheelFRRPM = 0;
float WheelRLRPM = 0;
float WheelRRRPM = 0;
uint16_t WheelFL = 0;
uint16_t WheelFR = 0;
uint16_t WheelRL = 0;
uint16_t WheelRR = 0;
uint32_t WheelFLTS = 0;
uint32_t WheelFRTS = 0;
uint32_t WheelRLTS = 0;
uint32_t WheelRRTS = 0;
uint8_t PrevWheelFL = 0;
uint8_t PrevWheelFR = 0;
uint8_t PrevWheelRL = 0;
uint8_t PrevWheelRR = 0;
uint8_t ResetWheelState = 0;
uint32_t ResetWheelTS = 0;
bool disableTUTDWhenReleased = false;
uint32_t doublePressTS = 0;
uint32_t lastTransBrakeTS = 0;
void disableTransbrake() {
    // ESP_LOGI("Mercedes", "Disable Transbrake");
    // twai_message_t message;
    // message.ss = 0;
    // message.self = 0;
    // message.dlc_non_comp = 0;
    // message.identifier = 0x7E2;
    // message.extd = 0;
    // message.rtr = 0;
    // message.data_length_code = 8;
    // message.data[0] = 0x07;
    // message.data[1] = 0xAE;
    // message.data[2] = 0x34;
    // message.data[3] = 0;
    // message.data[4] = 0;
    // message.data[5] = 0;
    // message.data[6] = 0;
    // message.data[7] = 0;
    // SendCan1(&message);
}
void enableTransbrake() {
    const uint32_t ts = usec();
    if(CurrentSpeedLastTS == 0 || ts - CurrentSpeedLastTS > 325000 || !(CurrentSpeed < 0.1 || (lastTransBrakeTS != 0 && ts - lastTransBrakeTS < 1000000 && CurrentSpeed < 10))) {
        disableTransbrake();
        return;
    }
    // lastTransBrakeTS = ts;
    // if(gearCommanded > 2)
    //     gearCommanded = 2;
    // ESP_LOGI("Mercedes", "Enable Transbrake");
                            
    // twai_message_t message;
    // message.ss = 0;
    // message.self = 0;
    // message.dlc_non_comp = 0;
    // message.identifier = 0x7E2;
    // message.extd = 0;
    // message.rtr = 0;
    // message.data_length_code = 8;
    // message.data[0] = 0x07;
    // message.data[1] = 0xAE;
    // message.data[2] = 0x34;
    // message.data[3] = 0xFF;
    // message.data[4] = 0;
    // message.data[5] = 0;
    // message.data[6] = 0;
    // message.data[7] = 0;
    // SendCan1(&message);
}
uint32_t timestampclosetonow(uint32_t now, uint32_t WheelTS) {
    return WheelTS;
    if(!(((uint32_t)(now - WheelTS)) < 5000 || ((uint32_t)(WheelTS - now)) < 5000)) // if its been longer than 5ms, set timestamp to now
        WheelTS = now;
    else
        WheelTS += (now - WheelTS) / 10; //keep Wheel timestamp close to now
    return WheelTS;
}

QueueHandle_t obd2queue;
void SendOBD2(uint8_t data[], uint16_t length) {
    twai_message_t message;
    message.ss = 0;
    message.self = 0;
    message.dlc_non_comp = 0;
    message.identifier = 0x7E8;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;
    if(length < 8) { //single frame
        message.data[0] = length;
        message.data[1] = 0x55;
        message.data[2] = 0x55;
        message.data[3] = 0x55;
        message.data[4] = 0x55;
        message.data[5] = 0x55;
        message.data[6] = 0x55;
        message.data[7] = 0x55;
        memcpy(&message.data[1], data, length);
        SendCan2(&message);
    } else { // multiframe
        message.data[0] = (length>>8) | 0x10;
        message.data[1] = length;
        uint8_t sn = 1;
        memcpy(&message.data[2], data, 6);
        SendCan2(&message);
        uint8_t dataIndex = 6;
        length -= 6;
        while(length > 0) {
            uint8_t buff[9];
            if(xQueueReceive(obd2queue, buff, pdMS_TO_TICKS(50)) == pdTRUE) {
                if((buff[0] & 0xF0) != 0x30)
                    break;
                if((buff[0] & 0x0F) == 2)
                    break;
            } else {
                buff[1] = 4;
                buff[2] = 50;
            }
            uint8_t blocksize = buff[1];
            uint8_t stTicks = pdMS_TO_TICKS(buff[2] + portTICK_PERIOD_MS/2);
            if(stTicks < 1)
                stTicks = 1;
            for(uint8_t i=0; i<blocksize; i++) {
                message.data[0] = (sn++ & 0xF) | 0x20;
                message.data[1] = 0x55;
                message.data[2] = 0x55;
                message.data[3] = 0x55;
                message.data[4] = 0x55;
                message.data[5] = 0x55;
                message.data[6] = 0x55;
                message.data[7] = 0x55;
                memcpy(&message.data[1], &data[dataIndex], length > 7? 7 : length);
                SendCan2(&message);
                dataIndex+=7;
                length-=7;
                if(length < 1)
                    break;
                vTaskDelay(stTicks);
            }
        }
    }
}

void OBD2ResponseService1(int PID, uint8_t data[], uint8_t size) {
    uint8_t *response = (uint8_t *)malloc(size + 2);
    response[0] = 0x41;
    response[1] = PID;
    memcpy(&response[2], data, size);
    SendOBD2(response, size + 2);
    free(response);
}

float snap = 1337;
void OBD2RequestService1(int PID) {
    switch(PID) {
        case 0x00: {
            uint8_t response[] = {0xEE, 0x5A, 0xE8, 0x13};
            return OBD2ResponseService1(PID, response, 4);
        } case 0x01: {
            uint8_t response[] = {0x00, 0x07, 0xFD, 0x00};
            return OBD2ResponseService1(PID, response, 4);
        } case 0x02: {
            uint8_t response[] = {0x00, 0x00};
            return OBD2ResponseService1(PID, response, 2);
        } case 0x03: {
            uint8_t response[] = {0x02, 0x02};
            return OBD2ResponseService1(PID, response, 2);
        } case 0x05: {
            uint8_t response = CLT + 40;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x06: {
            uint8_t response = 128 + 4 * slowmovingrandomstandardvalue;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x07: {
            if(snap == 1337)
                snap = slowmovingrandomstandardvalue;
            uint8_t response = 128 + 4 * snap;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x0A: {
            uint8_t response = FuelPressure * 33;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x0C: {
            uint16_t engineRPMX4 = EngineRPM * 4;
            uint8_t response[] = {(uint8_t)(engineRPMX4>>8), (uint8_t)engineRPMX4};
            return OBD2ResponseService1(PID, response, 2);
        } case 0x0D: {
            uint8_t response = CurrentSpeed;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x0F: {
            uint8_t response = IAT + 40;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x11: {
            uint8_t response = (AcceleratorPedalPosition * 100) / 255;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x12: {
            uint8_t response = 0x01;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x13: {
            uint8_t response = 0x03;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x15: {
            uint8_t response[] = {  (uint8_t)(((usec()+300000) % 2000000 > 1000000? 18 : 162) + 4 * slowmovingrandomstandardvalue), 
                                    (uint8_t)(128 + 4 * slowmovingrandomstandardvalue) };
            return OBD2ResponseService1(PID, response, 2);
        } case 0x1C: {
            uint8_t response = 0x01;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x1F: {
            uint16_t engineRunElapsedSeconds = EngineRunElapsedSeconds > 65535? 65535 : EngineRunElapsedSeconds;
            uint8_t response[] = {(uint8_t)(engineRunElapsedSeconds>>8), (uint8_t)engineRunElapsedSeconds};
            return OBD2ResponseService1(PID, response, 2);
        } case 0x20: {
            uint8_t response[] = {0x80, 0x1D, 0xB0, 0x11};
            return OBD2ResponseService1(PID, response, 4);
        } case 0x21: {
            uint8_t response[] = {0x00, 0x00};
            return OBD2ResponseService1(PID, response, 2);
        } case 0x2C: {
            uint8_t response = 0x00;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x2D: {
            uint8_t response = 128;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x2E: {
            uint8_t response = 0x00;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x30: {
            // uint8_t response = 0xFF; //TODO Warmup Count
            // return OBD2ResponseService1(PID, &response, 1);
            return;
        } case 0x31: {
            // uint8_t response[] = {0xFF, 0xFF};; //TODO Distance
            // return OBD2ResponseService1(PID, response, 2);
            return;
        } case 0x33: {
            uint8_t response = Baro;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x34: {
            uint16_t o2Current = 0x8000 + (usec() % 2000000 > 1000000? -94 : 111) + 75 * slowmovingrandomstandardvalue;
            uint16_t o2Equivalence = 32786 + (usec() % 2000000 > 1000000? -141 : 163) + 100 * slowmovingrandomstandardvalue;
            uint8_t response[] = { (uint8_t)(o2Equivalence >> 8), (uint8_t)o2Equivalence, (uint8_t)(o2Current >> 8), (uint8_t)o2Current};
            return OBD2ResponseService1(PID, response, 4);
        } case 0x3C: {
            uint16_t catTemp = CATTemp != -1337? (CATTemp + 40) * 10 : 700;
            uint8_t response[] = {(uint8_t)(catTemp>>8), (uint8_t)catTemp};
            return OBD2ResponseService1(PID, response, 2);
        } case 0x40: {
            uint8_t response[] = {0xD8, 0xA0, 0x00, 0x00};
            return OBD2ResponseService1(PID, response, 4);
        } case 0x41: {
            uint8_t response[] = {  0x00, 
                                    (uint8_t)(EngineRunElapsedSeconds > 1? 0x07 : 0x37), 
                                    0xFF, 
                                    (uint8_t)(EngineRunElapsedSeconds > 2? (EngineRunElapsedSeconds > 63? (EngineRunElapsedSeconds > 360? 0xFD : 0xFE) : 0xF8) : 0x18) };
            return OBD2ResponseService1(PID, response, 4);
        } case 0x42: {
            uint16_t voltage = (EngineRPM > 400? 14387 : 11892) + slowmovingrandomstandardvalue * 30;
            uint8_t response[] = { (uint8_t)(voltage>>8), (uint8_t)voltage };
            return OBD2ResponseService1(PID, response, 2);
        } case 0x44: {
            uint16_t equivalencRatio = 32786;
            uint8_t response[] = { (uint8_t)(equivalencRatio>>8), (uint8_t)equivalencRatio };
            return OBD2ResponseService1(PID, response, 2);
        } 
        case 0x45:
        case 0x47:
        case 0x48:
        case 0x49:
        case 0x4A: 
        case 0x4B: 
        case 0x4C:  {
            uint8_t response = (AcceleratorPedalPosition * 100) / 255;
            return OBD2ResponseService1(PID, &response, 1);
        } case 0x4D: {
            uint16_t response = 0x0000;
            return OBD2ResponseService1(PID, (uint8_t *)(&response), 2);
        } case 0x4E: {
            // uint16_t response = 0xFFFF; //TODO DTC Reset
            // return OBD2ResponseService1(PID, &response, 2);
            return;
        } case 0x60: {
            uint8_t response[] = {0x00, 0x00, 0x00, 0x00};
            return OBD2ResponseService1(PID, response, 4);
        } case 0x80: {
            uint8_t response[] = {0x00, 0x00, 0x00, 0x00};
            return OBD2ResponseService1(PID, response, 4);
        } case 0xA0: {
            uint8_t response[] = {0x00, 0x00, 0x00, 0x00};
            return OBD2ResponseService1(PID, response, 4);
        } case 0xC0: {
            uint8_t response[] = {0x00, 0x00, 0x00, 0x00};
            return OBD2ResponseService1(PID, response, 4);
        } case 0xE0: {
            uint8_t response[] = {0x00, 0x00, 0x00, 0x00};
            return OBD2ResponseService1(PID, response, 4);
        } 
    }
}

void OBD2ResponseService6(int MID, uint8_t data[], uint8_t size) {
    uint8_t *response = (uint8_t *)malloc(size + 2);
    response[0] = 0x46;
    response[1] = MID;
    memcpy(&response[2], data, size);
    SendOBD2(response, size + 2);
    free(response);
}

void OBD2RequestService6(int MID) {
    switch(MID) {
        case 0: {
            uint8_t response[] = { 0xC0, 0x00, 0x00, 0x00 };
            return OBD2ResponseService6(MID, response, 4);
        } case 0x1: {
        } case 0x2: {
        } case 0x20: {
            uint8_t response[] = { 0x80, 0x00, 0x80, 0x88 };
            return OBD2ResponseService6(MID, response, 4);
        } case 0x21: {
        } case 0x31: {
        } case 0x39: {
        } case 0x3D: {
        } case 0x40: {
            uint8_t response[] = { 0xC0, 0x00, 0x00, 0x00 };
            return OBD2ResponseService6(MID, response, 4);
        } case 0x41: {
        } case 0x42: {
        } case 0x60: {
            uint8_t response[] = { 0x00, 0x00, 0x80, 0x00 };
            return OBD2ResponseService6(MID, response, 4);
        } case 0x71: {
        } case 0x80: {
            uint8_t response[] = { 0x80, 0x00, 0x00, 0x00 };
            return OBD2ResponseService6(MID, response, 4);
        } case 0x81: {
        } case 0xA0: {
            uint8_t response[] = { 0xF8, 0x00, 0x00, 0x00 };
            return OBD2ResponseService6(MID, response, 4);
        } case 0xA1: {
        } case 0xA2: {
        } case 0xA3: {
        } case 0xA4: {
        } case 0xA5: {
        }
    }
}

void OBD2ResponseService8(int TID, uint8_t data[], uint8_t size) {
    uint8_t *response = (uint8_t *)malloc(size + 2);
    response[0] = 0x48;
    response[1] = TID;
    memcpy(&response[2], data, size);
    SendOBD2(response, size + 2);
    free(response);
}

uint8_t OBD2RequestService8(int TID, uint8_t data[], uint8_t size) {
    if(TID == 1) {
        uint8_t response[] = { 0x00, 0x00, 0x00, 0x00 };
        OBD2ResponseService8(TID, response, 4);
        return 4;
    }
    return 0;
}

void OBD2ResponseService9(int SID, uint8_t data[], uint8_t size) {
    uint8_t *response = (uint8_t *)malloc(size + 2);
    response[0] = 0x49;
    response[1] = SID;
    memcpy(&response[2], data, size);
    SendOBD2(response, size + 2);
    free(response);
}

void OBD2RequestService9(int SID) {
    ESP_LOGI("OBD2", "SID: %02x", SID);
    switch(SID) {
        case 1:
        case 3:
        case 5:
        case 7:
        {
            uint8_t response = 1;
            return OBD2ResponseService9(SID, &response, 1);
        }
        case 2: { //VIN
            uint8_t response[] = { 0x01, 'W', 'D', 'D', 'G', 'J', '4', 'H', 'B', '0', 'D', 'G', '0', '3', '2', '3', '6', '2' };
            return OBD2ResponseService9(SID, response, sizeof(response));
        } case 4: { //CALID
            // uint8_t response[] = { 0x01, '0', '9', '8', 'B', 'E', 'C', '6', '9', 'D', '1', '3', 'B', '1', '6', 'E', 'A' };
            // return OBD2ResponseService9(SID, &response, sizeof(response));
        } case 6: { //CVN
            // uint8_t response[] = { 0x01, 0x6A, 0xCE, 0xA7, 0x36 };
            // return OBD2ResponseService9(SID, &response, sizeof(response));
        } case 8: { //Performance tracking
            // uint8_t response[] = {};
            // return OBD2ResponseService9(SID, &response, sizeof(response));
        }
    }
}

void ParseOBD2(uint8_t *data, uint32_t OBD2_RXBytes) {
    uint8_t dataIndex = 0;
    if(OBD2_RXBytes < 1)
        return;
    uint8_t OBD2_Mode = data[dataIndex++];
    OBD2_RXBytes--;
    
    twai_message_t message;
    message.ss = 0;
    message.self = 0;
    message.dlc_non_comp = 0;
    message.identifier = 0x7E8;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    switch(OBD2_Mode) {
        case 1: { //Read PID
            while(OBD2_RXBytes-- > 0)
                OBD2RequestService1(data[dataIndex++]);
            return;
        } case 2: { //Read Freeze Frame
            while(OBD2_RXBytes-- > 0) {
                if(data[dataIndex++] == 2) {
                    uint8_t response[] = { 0x42, 0x02, 0x00, 0x00, 0x00 };
                    SendOBD2(response, 5);
                }
                dataIndex++;
                OBD2_RXBytes--;
            }
            return;
        } 
        case 3: //Read DTCs
        case 7: //Read Pending DTCs
        case 10: //Read Permanent DTCs
        {
            if(dataIndex > 0) {
                uint8_t response[] = { (uint8_t)(0x40 + OBD2_Mode), 0x00 };
                SendOBD2(response, 2);
            }
            return;
        } case 4: { //Clear DTCs
            if(dataIndex > 0) {
                uint8_t response[] = { 0x44 };
                SendOBD2(response, 1);
            }
            return;
        } case 6: { //Request On-Board Monitoring Test Results
            while(OBD2_RXBytes-- > 0)
                OBD2RequestService6(data[dataIndex++]);
            return;
        } case 8: { //Request  Control of On-Board System
            while(OBD2_RXBytes-- > 0) {
                uint8_t processedBytes = OBD2RequestService8(data[dataIndex], &data[dataIndex + 1], OBD2_RXBytes);
                OBD2_RXBytes -= processedBytes;
                dataIndex += processedBytes + 1;
            }
            return;
        } case 9: { //Request Vehicle Information
            while(OBD2_RXBytes-- > 0)
                OBD2RequestService9(data[dataIndex++]);
            return;
        }
    }
}

static void OBD2_Task(void *arg) {
    obd2queue = xQueueCreate( 20, 9 );
        twai_message_t message;
        message.ss = 0;
        message.self = 0;
        message.dlc_non_comp = 0;
        message.identifier = 0x7E8;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 8;

    while(1) {
        uint8_t buff[9];
        if(xQueueReceive(obd2queue, buff, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
            if((buff[0] & 0xF0) == 0) { //single frame
                ParseOBD2(&buff[1], buff[0]);
            } else if((buff[0] & 0xF0) == 0x10) { //multframe
                uint8_t *messagebuff;
                uint32_t messageIndex = 0;
                uint32_t size = ((buff[0] & 0x0F) << 8) | buff[1];
                if(size==0) {
                    size = (buff[2] << 24) | (buff[3] << 16) | (buff[4] << 8) | buff[5];
                    if(size > 0x1000){
                        //send flow control overflow frame
                        message.data[0] = 0x32;
                        message.data[1] = 0;
                        message.data[2] = 0;
                        message.data[3] = 0x55;
                        message.data[4] = 0x55;
                        message.data[5] = 0x55;
                        message.data[6] = 0x55;
                        message.data[7] = 0x55;
                        SendCan2(&message);
                        continue;
                    }
                    messagebuff = (uint8_t *)malloc(size);
                } else {
                    messagebuff = (uint8_t *)malloc(size);
                    messageIndex = buff[8]-2;
                    memcpy(messagebuff, &buff[2], messageIndex);
                }

                //send flow control frame
                const uint8_t blocksize = 1;
                const uint8_t st = 100;
                while(messageIndex < size) {
                    message.data[0] = 0x30;
                    message.data[1] = blocksize;
                    message.data[2] = st;
                    message.data[3] = 0x55;
                    message.data[4] = 0x55;
                    message.data[5] = 0x55;
                    message.data[6] = 0x55;
                    message.data[7] = 0x55;
                    SendCan2(&message);
                    for(uint8_t sn = 0; sn < blocksize; sn++) {
                        if(xQueueReceive(obd2queue, buff, 1000 / portTICK_PERIOD_MS) != pdTRUE)
                            goto OBD2_Taskcleanup;
                        if((buff[0] & 0xF0) != 0x20)
                            goto OBD2_Taskcleanup;
                        memcpy(&messagebuff[messageIndex], &buff[1], buff[8]-1);
                    }
                }

                ParseOBD2(messagebuff, size);
OBD2_Taskcleanup:
                free(messagebuff);
            }
        }
    }
}


void MercedesCanRX7DF(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    uint8_t obd2message[9];
    memcpy(obd2message, data.Data, 8);
    obd2message[8] = dataLength;
    xQueueSend(obd2queue, obd2message, 0);
}

void MercedesCanRX7E8(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    uint8_t obd2message[9];
    memcpy(obd2message, data.Data, 8);
    obd2message[8] = dataLength;
    xQueueSend(obd2queue, obd2message, 0);
}

void MercedesCanRX001(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    // ESP_LOGI(EXAMPLE_TAG, "0x1 = %i %02x %02x %02x %02x %02x %02x %02x %02x", message->message.data_length_code, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);  
    IgnitionOnStartInactive = ((data.Data[0] >> 3) & 0x1);
    IgnitionSwitchState = (IgnitionSwitchState_t)((data.Data[0] >> 0) & 0x7);
    // ESP_LOGI("MERCEDES", "IgnitionOnStartInactive: %i, IgnitionSwitchState: %i", IgnitionOnStartInactive, IgnitionSwitchState);
}

void MercedesCanRX003(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    SteeringWheelAngle = (((data.Data[0] & 0x3F) << 8) | data.Data[1]) * 0.5f - 2048;
    SteeringWheelAngleSpeed = (((data.Data[2] & 0x3F) << 8) | data.Data[3]) * 0.5f - 2048;
    // ESP_LOGI("MERCEDES", "SteeringWheelAngle: %f, SteeringWheelAngleSpeed: %f", SteeringWheelAngle, SteeringWheelAngleSpeed);
}

void MercedesCanRX005(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    BrakeApplied = (data.Data[0] & 0x3) == 0x1;
    if(BrakeApplied) {
        gpio_set_level(GPIO_NUM_3, 0);
        // gpio_set_level(GPIO_NUM_10, 0);
    } else  {
        // gpio_set_level(GPIO_NUM_10, 1);
        gpio_set_level(GPIO_NUM_3, 1);
    }
    // ESP_LOGI("MERCEDES", "BrakeApplied: %i", BrakeApplied);
}

void MercedesCanRX05F(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    ESPEnable = (data.Data[4] * 0x20) == 0;
    // ESP_LOGI("MERCEDES", "ESPEnable: %i", ESPEnable);
}

void MercedesCanRX073(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    SportSwitch = (data.Data[0] & 0x40) != 0;
    const uint8_t sbwPos = data.Data[0] & 0xF;
    if(sbwPos == 9 && !SBWUp) {
        TapShiftEnable = true;
        CommandGearUp();
    } else if(sbwPos == 10 && !SBWDown) {
        TapShiftEnable = true;
        CommandGearDown();
    }
    SBWUp = sbwPos == 9;
    SBWDown = sbwPos == 10;
    // ESP_LOGI("MERCEDES", "SportSwitch: %i, SBWUp: %i, SBWDown: %i", SportSwitch, SBWUp, SBWDown);
}

void MercedesCanRX06D(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    const uint32_t ts = usec();
    if(lastTransBrakeTS != 0 && ts - lastTransBrakeTS > 2000000)
        lastTransBrakeTS = 0;
    if((data.Data[0] & 0x3) == 0x3) {
        if(!disableTUTDWhenReleased) {
            disableTUTDWhenReleased = true;
            doublePressTS = ts;
        } else if(ts - doublePressTS > 500000) {
            if(GovernSpeed == 510 && CurrentSpeed > 40) {
                GovernSpeed = CurrentSpeed;
                for(int i = sizeof(maxDownshiftKph) / sizeof(uint16_t); i > 0; i--) {
                    if(CurrentSpeed < maxDownshiftKph[i-1]) {
                        gearCommanded = i;
                    } else {
                        break;
                    }
                }
            } else if(CurrentSpeed < 10){
                enableTransbrake();
            }
        }
        if(!(!STWUp && !STWDown)) {
            if(!STWUp)
                CommandGearUp();
            if(!STWDown)
                CommandGearDown();
        }
    } else {
        if(disableTUTDWhenReleased) {
            GovernSpeed = 510;
            disableTransbrake();
            TapShiftEnable = false;
            if((data.Data[0] & 0x3) == 0) {
                gearCommanded = 0;
                disableTUTDWhenReleased = false;
            }
        } else if((data.Data[0] & 0x1) != 0 && !STWUp){
            TapShiftEnable = true;
            CommandGearUp();
        } else if((data.Data[0] & 0x2) != 0 && !STWDown){
            TapShiftEnable = true;
            CommandGearDown();
        }
    }
    STWUp = (data.Data[0] & 0x1) != 0;
    STWDown = (data.Data[0] & 0x2) != 0;
    // ESP_LOGI("MERCEDES", "STWUp: %i, STWDown: %i", STWUp, STWDown);
}

void MercedesCanRX045(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    CruiseCancel = (data.Data[0] & 0x01) != 0;
    CruiseResume = (data.Data[0] & 0x02) != 0;
    CruiseAccelHigh = (data.Data[0] & 0x04) != 0;
    CruiseDeccelHigh = (data.Data[0] & 0x08) != 0;
    CruiseAccelLow = (data.Data[0] & 0x10) != 0;
    CruiseDeccelLow = (data.Data[0] & 0x20) != 0;
    // ESP_LOG("MERCEDES", "CruiseCancel: %i, CruiseResume: %i, CruiseAccelHigh: %i, CruiseDeccelHigh: %i, CruiseAccelLow: %i, CruiseDeccelLow: %i", 
    //     CruiseCancel, CruiseResume, CruiseAccelHigh, CruiseDeccelHigh, CruiseAccelLow, CruiseDeccelLow);
}

void MercedesCanRX0F9(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    ACCompressorOn = (data.Data[0] >> 7) & 0x1;
    ACCompressorTorque = data.Data[0] * 4;
    ACCompressorOnRequest = (data.Data[6] >> 1) & 0x1;
}

void MercedesCanRX245(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    YawRate = ((data.Data[0] << 8) | data.Data[1]) * 0.009999999776482582f - 327.67999267578125f;
    LateralAcceleration = data.Data[5] * 0.07999999821186066f - 10.239999771118164f;
}

void MercedesCanRX201(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    //timestamp is 1/128 msec
    uint32_t ts = usec();// * 128 / 1000;
    bool reset = false;
    switch(ResetWheelState) {
        case 0:
            ResetWheelTS = ts;
            ResetWheelState = 1;
            WheelFLTS = ts;
            WheelFRTS = ts;
            WheelRLTS = ts;
            WheelRRTS = ts;
        case 1:
            reset = true;
            if(ts - ResetWheelTS > 200 * 128)
                ResetWheelState = 2;
            else 
                break;
        case 2:
            reset = false;
    }
    uint8_t sn = data.Data[6] >> 4;
    
    if(PrevWheelFL != data.Data[0] && WheelFLRPM > 0) {
        const float tsperpulse = 1000 * 1000 * 60 / (WheelFLRPM * 96);
        const uint8_t pulses = data.Data[0] - PrevWheelFL;
        WheelFLTS += tsperpulse * pulses;
        WheelFL += pulses;
        WheelFLTS = timestampclosetonow(ts, WheelFLTS);
    }
    PrevWheelFL = data.Data[0];

    if(PrevWheelFR != data.Data[1] && WheelFRRPM > 0) {
        const float tsperpulse = 1000 * 1000 * 60 / (WheelFRRPM * 96);
        const uint8_t pulses = data.Data[1] - PrevWheelFR;
        WheelFRTS += tsperpulse * pulses;
        WheelFR += pulses;
        WheelFRTS = timestampclosetonow(ts, WheelFRTS);
    }
    PrevWheelFR = data.Data[1];

    if(PrevWheelRL != data.Data[2] && WheelRLRPM > 0) {
        const float tsperpulse = 1000 * 1000 * 60 / (WheelRLRPM * 96);
        const uint8_t pulses = data.Data[2] - PrevWheelRL;
        WheelRLTS += tsperpulse * pulses;
        WheelRL += pulses;
        WheelRLTS = timestampclosetonow(ts, WheelRLTS);
    }
    PrevWheelRL = data.Data[2];

    if(PrevWheelRR != data.Data[3] && WheelRRRPM > 0) {
        const float tsperpulse = 1000 * 1000 * 60 / (WheelRRRPM * 96);
        const uint8_t pulses = data.Data[3] - PrevWheelRR;
        WheelRRTS += tsperpulse * pulses;
        WheelRR += pulses;
        WheelRRTS = timestampclosetonow(ts, WheelRRTS);
    }
    PrevWheelRR = data.Data[3];

    twai_message_t smessage;
    smessage.identifier = 0xC1;
    smessage.extd = 0;
    smessage.rtr = 0;
    smessage.data_length_code = 8;
    smessage.data[0] = (((WheelRLTS >> 10) & 0xC0) | (sn & 0x3) << 4) | (reset << 3) | ((WheelRL >> 8) & 0x3);
    smessage.data[1] = WheelRL;
    smessage.data[2] = WheelRLTS >> 8;
    smessage.data[3] = WheelRLTS;
    smessage.data[4] = (((WheelRRTS >> 10) & 0xC0) | (sn & 0x3) << 4) | (reset << 3) | ((WheelRR >> 8) & 0x3);
    smessage.data[5] = WheelRR;
    smessage.data[6] = WheelRRTS >> 8;
    smessage.data[7] = WheelRRTS;
    SendCan1(&smessage);
    smessage.identifier = 0xC5;
    smessage.extd = 0;
    smessage.rtr = 0;
    smessage.data_length_code = 8;
    smessage.data[0] = (((WheelFLTS >> 10) & 0xC0) | (sn & 0x3) << 4) | (reset << 3) | ((WheelFL >> 8) & 0x3);
    smessage.data[1] = WheelFL;
    smessage.data[2] = WheelFLTS >> 8;
    smessage.data[3] = WheelFLTS;
    smessage.data[4] = (((WheelFRTS >> 10) & 0xC0) | (sn & 0x3) << 4) | (reset << 3) | ((WheelFR >> 8) & 0x3);
    smessage.data[5] = WheelFR;
    smessage.data[6] = WheelFRTS >> 8;
    smessage.data[7] = WheelFRTS;
    SendCan1(&smessage);
}

void MercedesCanRX203(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    WheelFLRPM = (((data.Data[0] & 0x3F) << 8) | data.Data[1]) * 0.5f;
    WheelFRRPM = (((data.Data[2] & 0x3F) << 8) | data.Data[3]) * 0.5f;
    WheelRLRPM = (((data.Data[4] & 0x3F) << 8) | data.Data[5]) * 0.5f;
    WheelRRRPM = (((data.Data[6] & 0x3F) << 8) | data.Data[7]) * 0.5f;
}

void MercedesCanRX283(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    ParkBrakeApplied = (data.Data[3] & 0x10) != 0;
}

void MercedesCanRX2E5(can_send_callback_t send, const CANData_t data, const uint8_t dataLength) {
    FuelPressure = data.Data[0] / 20.0f;
}

void RegisterCANTasks() {
    _embeddedIOServiceCollection.DigitalService->WritePin(digitalpin_t(50), false);
    _embeddedIOServiceCollection.DigitalService->InitPin(digitalpin_t(50), Out);

    xTaskCreate(OBD2_Task, "obd2_task", 4096, NULL, 8, NULL);
    xTaskCreate(CANTask100Hz, "CANTask100Hz", 4096, NULL, 10, NULL);
    xTaskCreate(period50Hz, "Period50Hz", 4096, NULL, 10, NULL);
    xTaskCreate(period33Hz, "Period33Hz", 4096, NULL, 10, NULL);
    xTaskCreate(period10Hz, "Period10Hz", 4096, NULL, 10, NULL);
    xTaskCreate(period4Hz, "Period4Hz", 4096, NULL, 10, NULL);
    xTaskCreate(period1Hz, "Period1Hz", 4096, NULL, 10, NULL);

    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x3D1, .CANBusNumber = 0 }, GMCanRX3D1);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x3E9, .CANBusNumber = 0 }, GMCanRX3E9);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x4C1, .CANBusNumber = 0 }, GMCanRX4C1);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x4C9, .CANBusNumber = 0 }, GMCanRX4C9);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x4D1, .CANBusNumber = 0 }, GMCanRX4D1);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x1A1, .CANBusNumber = 0 }, GMCanRX1A1);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x2C3, .CANBusNumber = 0 }, GMCanRX2C3);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x1F5, .CANBusNumber = 0 }, GMCanRX1F5);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x1ED, .CANBusNumber = 0 }, GMCanRX1ED);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x4F1, .CANBusNumber = 0 }, GMCanRX4F1);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x0F9, .CANBusNumber = 0 }, GMCanRX0F9);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x1C3, .CANBusNumber = 0 }, GMCanRX1C3);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x0C9, .CANBusNumber = 0 }, GMCanRX0C9);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x3F9, .CANBusNumber = 0 }, GMCanRX3F9);

    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x7DF, .CANBusNumber = 1 }, MercedesCanRX7DF);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x7E8, .CANBusNumber = 1 }, MercedesCanRX7E8);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x001, .CANBusNumber = 1 }, MercedesCanRX001);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x003, .CANBusNumber = 1 }, MercedesCanRX003);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x005, .CANBusNumber = 1 }, MercedesCanRX005);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x05F, .CANBusNumber = 1 }, MercedesCanRX05F);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x073, .CANBusNumber = 1 }, MercedesCanRX073);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x06D, .CANBusNumber = 1 }, MercedesCanRX06D);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x045, .CANBusNumber = 1 }, MercedesCanRX045);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x0F9, .CANBusNumber = 1 }, MercedesCanRX0F9);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x245, .CANBusNumber = 1 }, MercedesCanRX245);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x201, .CANBusNumber = 1 },MercedesCanRX201);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x203, .CANBusNumber = 1 },MercedesCanRX203);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x283, .CANBusNumber = 1 },MercedesCanRX283);
    _embeddedIOServiceCollection.CANService->RegisterReceiveCallBack({.CANIdentifier = 0x2E5, .CANBusNumber = 1 },MercedesCanRX2E5);
}