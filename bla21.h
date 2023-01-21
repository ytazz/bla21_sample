#pragma once

#include <cstdint>
#include <vector>
#include <deque>
#include <string>

#include "serial.h"

using namespace std;

#pragma pack(push, 1)
struct Frame{
	uint32_t         id;	///< 29bit id
	vector<uint8_t>  data;
};
struct GetSet{
	uint8_t index;
	uint8_t tag  ;
	int64_t  val;
	char     name[4];
};
struct BroadcastFrameID{
	uint8_t  priority;
	uint16_t serviceId;
	uint8_t  isService;
	uint8_t  srcId;
		
	uint32_t  Encode();
	void      Decode(uint32_t val);
};
struct UnicastFrameID{
	uint8_t  priority;
	uint8_t  serviceId;
	uint8_t  isRequest;
	uint8_t  destId;
	uint8_t  isService;
	uint8_t  srcId;
		
	uint32_t  Encode();
	void      Decode(uint32_t val);
};
#pragma pack(pop)

struct Address{
	enum{
		CommandPosition = 0x00,  ///< command position [0.1deg] positive value means CW
		CommandVelocity = 0x01,  ///< command velocity [rpm]
		CommandTorque   = 0x02,  ///< command torque [-100 - +100]
		
		CurrentPosition = 0x08,  ///< current position [0.1deg]
		CurrentVelocity = 0x09,  ///< current velocity [rpm]
		CurrentTorque   = 0x0A,  ///< current torque [-100 - +100]
		CurrentTemperature = 0x0B,  ///< current temperature [degC]
		CurrentVoltage     = 0x0C,  ///< current voltage [0.1V]

		Initialize = 0x10,  ///< write 1 and reset RAM data to default values
		Reboot     = 0x11,  ///< write 1 and reboot servo
		WriteToRom = 0x12,  ///< write 1 and write RAM data to ROM

		TorqueOnOff = 0x20, ///< 0: on  1: off  2: brake
		                    /// to be torque-disabled at power-on, you need to write 1 and save it to ROM!

		PositionPGain = 0x2C,  ///< position control P gain [1 - 100] default 60
		PositionDGain = 0x2D,  ///< not implemented
		PositionDeadband = 0x2E, ///< position control deadband [0.1deg] [0 - 3600] default 3

		VelocityPGain    = 0x30,
		VelocityIGain    = 0x31,
		VelocityDeadband = 0x32,
		VelocityILimit   = 0x33,

		PositionLimitCW = 0x38,
		PositionLimitCCW = 0x39,
		VelocityLimitCW = 0x3A,
		VelocityLimitCCW = 0x3B,
		TorqueLimitCW = 0x3C,
		TorqueLimitCCW = 0x3D,
		TemperatureUpperLimit = 0x3E,
		TemperatureLowerLimit = 0x3F,
		VoltageUpperLimit = 0x40,
		VoltageLowerLimit = 0x41,

		Origin  = 0x44,
		ServoID = 0x45,  ///< ID used in ArrayCommand
		NodeID  = 0x46,  ///< ID used in UAVCAN

		Bootloader = 0x49,

		FirmwareVersion = 0x52,
		HardwareVersion = 0x53,
	};
};
struct DataType{
	enum{
		GetSet               =   11,
		NodeStatus           =  341,
		ActuatorArrayCommand = 1010,
		ActuatorStatus       = 1011,
	};
};
struct Torque{
	enum{
		On    = 0x00,
		Off   = 0x01,
		Brake = 0x02,
	};
};

class BLA21{
public:
	int  hostId;
	int  frameCount;
	bool verbose;
	deque<uint8_t>	 buffer;
	
	void SendData    (Serial* serial, uint32_t id, const uint8_t* sig, const vector<uint8_t>& raw);
	void SendFrame   (Serial* serial, const Frame& frame);
	bool ParseFrame  (const string& data, Frame& frame);
	void ProcessFrame(const Frame& frame);

public:
	void OpenChannel (Serial* serial);
	void CloseChannel(Serial* serial);
	void SetProperty (Serial* serial, int canId, int index, int value);
	void EnableTorque(Serial* serial, int id, bool on);
	void SetGoalPosition(Serial* serial, int id, float pos);
	void SetGoalPositionForAll(Serial* serial, const vector<int>& idArray, const vector<float>& posArray);
	void Read        (Serial* serial);

	 BLA21();
	~BLA21();
};
