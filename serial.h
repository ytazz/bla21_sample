#pragma once

#include <stdint.h>
#include <string.h>

class Serial{
public:
	struct Parity{
		enum{
			None = 0,
			Odd  = 1,
			Even = 2
		};
	};

	void*	handle;       ///< ƒnƒ“ƒhƒ‹
		
public:	
	bool Open(const char* port);
	void Close();

	size_t Out(const uint8_t* c, size_t n);

	template<class T>
	size_t Out(const T& c){
		return Out((const byte*)&c, sizeof(T));
	}

	size_t TextOut(const char* c){
		return Out((const uint8_t*)c, strlen(c));
	}
	
	size_t In(uint8_t* c, size_t n, bool full = true);

	template<class T>
	size_t In(T* c, bool full = true){
		return In((byte*)c, sizeof(T), true);
	}

	void CountBytes(size_t* rx, size_t* tx);
	
	 Serial();
	~Serial();
};
