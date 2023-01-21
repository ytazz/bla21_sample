#ifdef _WIN32
# define WIN32_LEAN_AND_MEAN
# include <windows.h>
#endif

#include "serial.h"

Serial::Serial(){
	handle = 0;
}

Serial::~Serial(){
	Close();
}

bool Serial::Open(const char* port){
	Close();

	handle = CreateFile(
		port,
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);

	if(handle == INVALID_HANDLE_VALUE)
		return false;
	
	return true;
}

void Serial::Close(){
	if(handle){
		CloseHandle(handle);
		handle = 0;
	}
}

size_t Serial::Out(const uint8_t* c, size_t n){
	uint32_t nBytesWritten;

	WriteFile(handle, (LPVOID)c, (DWORD)n, (LPDWORD)&nBytesWritten, 0);
	FlushFileBuffers(handle);

	return nBytesWritten;
}

size_t Serial::In(uint8_t* c, size_t n, bool full){
	uint32_t nreadTotal = 0;
	uint32_t nread;
	while(nreadTotal < n){
		ReadFile(handle, (LPVOID)c, (DWORD)(n - nreadTotal), (LPDWORD)&nread, 0 );
		nreadTotal += nread;

		if(!full || nread == 0 || nreadTotal == n)
			break;

		c += nread;
	}
	
	return nreadTotal;
}

void Serial::CountBytes(size_t* rx, size_t* tx){
	uint32_t _rx, _tx;

	COMSTAT stat;
	ClearCommError(handle, 0, &stat);
	_rx = stat.cbInQue;
	_tx = stat.cbOutQue;

	if(rx)
		*rx = _rx;
	if(tx)
		*tx = _tx;
}
