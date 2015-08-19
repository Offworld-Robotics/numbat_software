#ifndef MESSAGE_H
#define MESSAGE_H

#define MESSAGE_MAGIC 0x55AA

// All types are multiples of 16 bits, due to how XC16 optimises for memory access
// All structs are multiples of 32 bits, so this works on x86
	struct outMessage {
		uint16_t magic;
		int16_t lSpeed;
		int16_t rSpeed;
		uint16_t P;
		uint16_t I;
		uint16_t D;
		uint16_t servo3;
		uint16_t servo4;
		uint16_t servo5;
		uint16_t out1:1;
		uint16_t out2:1;
	};

	struct inMessage {
		uint16_t magic;
		uint16_t vref;
		uint16_t vbat;
		uint16_t ain1;
		int16_t encLSpeed;
		int16_t encRSpeed;
		int32_t encLDisp;
		int32_t encRDisp;
		uint16_t in1:1;
		uint16_t in2:1;
		int16_t dummy;
	};

#endif
