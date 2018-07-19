#ifndef VIRTUALREGISTER_H
#define VIRTUALREGISTER_H

#include <stdint.h>

//! Class that controls the virtual registers
class VirtualRegister {
	public:
		//! Constructor
		VirtualRegister();
		//! Writes data to a register
		/*!
		*	\param address Register address
		*	\param data New data to be written to the address
		*/
		static void writeRegister(uint8_t, uint8_t);
		//! Reads data from a register
		/*!
		*	\param address Register address
		*	\return uint8_t
		*/
		static uint8_t readRegister(uint8_t);
		
	private:
		//! Stores all values of the virtual registers
		static uint8_t virtualRegisterFields[255];
};

#endif // VIRTUALREGISTER_H