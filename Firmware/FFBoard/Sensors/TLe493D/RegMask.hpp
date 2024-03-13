#ifndef TLE493D_REGMASK_H_INCLUDED
#define TLE493D_REGMASK_H_INCLUDED

#include <stdint.h>
//#include <Arduino.h>

#define REGMASK_READ	0
#define REGMASK_WRITE	1 //rw
/*
#ifdef __cplusplus
extern "C" {
#endif
*/
namespace tle493d
{

typedef struct
{
	uint8_t rw;
	uint8_t byteAdress;
	uint8_t bitMask;
	uint8_t shift;
} RegMask_t;

uint8_t getFromRegs(const RegMask_t *mask, uint8_t *regData);
void setToRegs(const RegMask_t *mask, uint8_t *regData, uint8_t toWrite);

}
/*
#ifdef __cplusplus
}
#endif
*/
#endif
