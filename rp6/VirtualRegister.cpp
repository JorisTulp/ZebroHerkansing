#include "VirtualRegister.h"

uint8_t VirtualRegister::virtualRegisterFields[255];

void VirtualRegister::writeRegister(uint8_t address, uint8_t data) {
	virtualRegisterFields[address] = data;
}

uint8_t VirtualRegister::readRegister(uint8_t address) {
	
	return virtualRegisterFields[address];
}

VirtualRegister::VirtualRegister() {

}