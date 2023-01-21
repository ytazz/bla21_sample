#include "bla21.h"
#include "crc16.h"
#include "float16.h"

// see manual for other signatures
const uint8_t signatureGetSet      [] = { 0xd5, 0xa4, 0xd1, 0x39, 0xf9, 0x22, 0xb6, 0xa7 };  //< 0xa7b622f939d1a4d5
const uint8_t signatureArrayCommand[] = { 0xf3, 0x3a, 0xec, 0x38, 0x62, 0x48, 0xa7, 0xd8 };  //< 0xd8a7486238ec3af3

uint32_t BroadcastFrameID::Encode(){
	return
		(priority  & 0x1f  ) << 24 |
		(serviceId & 0xffff) <<  8 |
		((isService & 0x01) << 7 | (srcId  & 0x7f));
}

void BroadcastFrameID::Decode(uint32_t val){
	priority  = (val >> 24) & 0x1f;
	serviceId = (val >>  8) & 0xffff;
	isService = (val >> (0+7)) & 0x01;
	srcId     = (val >> (0+0)) & 0x7f;
}

uint32_t UnicastFrameID::Encode(){
	return
		(priority  & 0x1f) << 24 |
		(serviceId & 0xff) << 16 |
		((isRequest & 0x01) << 7 | (destId & 0x7f)) << 8 |
		((isService & 0x01) << 7 | (srcId  & 0x7f));
}

void UnicastFrameID::Decode(uint32_t val){
	priority  = (val >> 24) & 0x1f;
	serviceId = (val >> 16) & 0xff;
	isRequest = (val >> (8+7)) & 0x01;
	destId    = (val >> (8+0)) & 0x7f;
	isService = (val >> (0+7)) & 0x01;
	srcId     = (val >> (0+0)) & 0x7f;
}

BLA21::BLA21(){
	hostId  = 127;
	verbose = false;
}

BLA21::~BLA21(){

}

void BLA21::SetProperty(Serial* serial, int canId, int index, int value){
	UnicastFrameID id;
	id.priority   = 0;
	id.serviceId  = DataType::GetSet;
	id.isRequest  = 1;
	id.destId     = canId;
	id.isService  = 1;
	id.srcId      = hostId;

	GetSet  gs;
	gs.index = index;
	gs.tag   = 1;
	gs.val   = value;

	char tmp[5];
	sprintf(tmp, "0x%02x", gs.index);
	strncpy(gs.name, tmp, 4);

	const int sz = sizeof(GetSet);
	vector<uint8_t> raw(sz);
	copy((const uint8_t*)&gs, (const uint8_t*)&gs + sz, raw.begin());

	SendData(serial, id.Encode(), signatureGetSet, raw);
}

void BLA21::EnableTorque(Serial* serial, int canId, bool on){
	SetProperty(serial, canId, Address::TorqueOnOff, (on ? Torque::On : Torque::Off));
}

void BLA21::SetGoalPosition(Serial* serial, int canId, float deg){
	// note: 
	// when you set goal position using GetSet,
	// the goal position is expressed in integer with the unit [0.1deg]
	SetProperty(serial, canId, Address::CommandPosition, (int)(10.0f*deg));
}

void BLA21::OpenChannel(Serial* serial){
	// set bitrate to 1000000
	// disable timestamp appending
	// open channel
	serial->TextOut("S8\rZ0\rO\r");
	
	// reset frame count to zero
	frameCount = 0;
}

void BLA21::CloseChannel(Serial* serial){
	serial->TextOut("C\r");
}

void BLA21::SendData(Serial* serial, uint32_t id, const uint8_t* sig, const vector<uint8_t>& raw){
	vector<Frame> frame;

	int sz = (int)raw.size();

	// single frame
	if(sz <= 7){
		// send single frame
		frame.resize(1);
		frame[0].id = id;
		frame[0].data.resize(sz + 1);
		copy(raw.begin(), raw.end(), frame[0].data.begin());

		frame[0].data[sz] = 0x80 | 0x40 | (frameCount & 0x1f);
	}
	// multiple frames
	else{
		// calc crc
		// note:
		// concatenate data type signature (see above)
		// and data payload into a single byte array, and feed it to CRC16 calculation
		vector<uint8_t> tmp(8 + sz);
		copy(sig, sig + 8, tmp.begin());
		copy(raw.begin(), raw.end(), tmp.begin() + 8);

		uint16_t crc = CalcCRC(tmp);

		// calculate number of frames to send
		// note:
		// first frame can carry 5 bytes (because you need 2 bytes for CRC)
		// and second and later frames can carry 7 bytes
		int n = (sz + 2)/7 + ((sz + 2)%7 == 0 ? 0 : 1);
		frame.resize(n);

		// first frame
		frame[0].id = id;
		frame[0].data.resize(8);
		frame[0].data[0] = (crc >> 0) & 0xff;
		frame[0].data[1] = (crc >> 8) & 0xff;
		int idx = 0;
		copy(raw.begin(), raw.begin() + 5, frame[0].data.begin() + 2);
		idx += 5;
		frame[0].data[7] = 0x80 | (frameCount & 0x1f);

		// remaining frames
		for(int i = 1; i < n; i++){
			frame[i].id = id;
			int _sz = (i == n-1 ? sz - idx : 7);  //< data length of this frame
			frame[i].data.resize(_sz + 1);
			copy(raw.begin() + idx, raw.begin() + idx + _sz, frame[i].data.begin());
			idx += _sz;

			frame[i].data[_sz] = (i == n-1 ? 0x40 : 0x00) | (i % 2 ? 0x20 : 0x00) | (frameCount & 0x1f);
		}
	}

	for(auto& fr : frame){
		SendFrame(serial, fr);
	}

	// increment frame count
	frameCount = (frameCount + 1) & 0x1f;

}

void BLA21::SetGoalPositionForAll(Serial* serial, const vector<int>& idArray, const vector<float>& posArray){
	BroadcastFrameID id;
	id.priority   = 0;
	id.serviceId  = DataType::ActuatorArrayCommand;
	id.isService  = 0;
	id.srcId      = hostId;

	int num = (int)idArray.size();
	vector<uint8_t> raw(4*num);
	for(int i = 0; i < num; i++){
		raw[4*i + 0] = idArray[i];
		// note:
		// choost unitless, meaning position is normalized [-180, +180][deg] -> [-1, +1]
		// and then encoded in float16
		raw[4*i + 1] = 0;

		uint16_t pos16 = Float16::Encode(std::min(std::max(-1.0f, posArray[i]/180.0f), +1.0f));
		raw[4*i + 2] = (pos16 >> 0) & 0xff;
		raw[4*i + 3] = (pos16 >> 8) & 0xff;
	}
	
	SendData(serial, id.Encode(), signatureArrayCommand, raw);
}

void BLA21::SendFrame(Serial* serial, const Frame& frame){
	char str[1024];
	char* ptr = str;

	ptr += sprintf(ptr, "T%08x%x", frame.id, (uint32_t)frame.data.size());
	for(int i = 0; i < frame.data.size(); i++)
		ptr += sprintf(ptr, "%02x", frame.data[i]);
	sprintf(ptr, "\r");

	serial->TextOut(str);

	if(verbose){
		printf("<< %s\n", str);
	}
}

bool BLA21::ParseFrame(const string& data, Frame& frame){
	string::const_iterator it = data.begin();
	
	// first comes 'T' (29bit CAN frame)
	char head = *it++;
	if(head != 'T')
		return false;

	// 8 hex digits must follow
	if(data.end() - it < 8)
		return false;

	// read 8 hex digits as CAN frame id (29bit)
	string str;
	str.resize(8);
	copy(it, it + 8, str.begin());
	it += 8;
	sscanf(str.c_str(), "%x", &frame.id);

	// next must be data length
	if(data.end() - it < 1)
		return false;

	// read data length
	int sz;
	str.resize(1);
	copy(it, it + 1, str.begin());
	it++;
	sscanf(str.c_str(), "%x", &sz);
	if(!(0 <= sz && sz <= 0x0f))
		return false;

	frame.data.resize(sz);

	// 2*(data length) hex digits must follow
	if(data.end() - it < 2*sz)
		return false;

	// read data
	for(int i = 0; i < sz; i++){
		str.resize(2);
		copy(it, it + 2, str.begin());
		it += 2;

		uint32_t val;
		sscanf(str.c_str(), "%x", &val);
		frame.data[i] = val;
	}

	if(data.end() - it < 1)
		return false;

	// frame must be terminted by '\r'
	if(*it != '\r')
		return false;

	return true;
}

void BLA21::ProcessFrame(const Frame& frame){
	int priority   = (frame.id >> 24) & 0x1f;
	int isService  = (frame.id >>  7) & 0x01;
	int srcId      = (frame.id >>  0) & 0x7f;

	// service frame
	if(isService){
		int type      = (frame.id >> 16) & 0xff;
		int isRequest = (frame.id >> 15) & 0x01;
		int destId    = (frame.id >>  8) & 0x7f;

		if(verbose){
			printf(">> service frame: src: %d  dest: %d  type: %d\n",
				srcId, destId, type);
		}
	}
	// message frame
	else{
		int type = (frame.id >>  8) & 0xffff;

		if(verbose){
			printf(">> message frame: src: %d  type: %d\n",
				srcId, type);
		}
		
		if(type == DataType::NodeStatus){

		}
		if(type == DataType::ActuatorStatus){
			// note:
			// actuator.status frame has 12 byte payload, which means it
			// consists of first frame (8 bytes data) and second frame (4 bytes data)
			// To be secure, you need to receive both frames, do error check using CRC, but the code gets complicated...
			// If you are lazy, current position can be taken from first frame alone.

			if(frame.data.size() == 8){
				int       actuatorId = frame.data[2];

				// note:
				// position in actuator.status is encoded in float16 with unit [0.01deg]
				float pos = 100.0f*Float16::Decode(frame.data[4] << 8 | frame.data[3]);

				printf(">> actuator.status: servo id: %d  position: %f\n",
					actuatorId, pos);
			}
		}

	}

}

void BLA21::Read(Serial* serial){
	// append data in RX buffer
	size_t rx = 0, tx = 0;
	vector<uint8_t> data;

	serial->CountBytes(&rx, &tx);
	if(rx != 0){
		data.resize(rx);

		serial->In(&data[0], rx);
		buffer.insert(buffer.end(), data.begin(), data.end());
	}

	Frame frame;
	string  str;
	while(true){
		// find '\r'
		deque<uint8_t>::iterator it = buffer.begin();
		for( ; it != buffer.end(); it++){
			if(*it == '\r'){
				it++;
				break;
			}
		}

		// not found
		if(it == buffer.end() && (buffer.empty() || buffer.back() != '\r'))
			break;

		str.resize(it - buffer.begin());
		copy(buffer.begin(), it, str.begin());
		buffer.erase(buffer.begin(), it);

		if(ParseFrame(str, frame))
			ProcessFrame(frame);
	}
}
