
#ifndef _TYPEDEF_H_
#define _TYPEDEF_H_


typedef char SBYTE;
typedef unsigned char BYTE;

typedef short SWORD;
typedef unsigned short WORD;

typedef long SDWORD;
typedef unsigned long DWORD;


#define bit_select(B, i)		((B >> i)&0x01ul)
#define bit_set(B, i)			(B |= (1ul << i))
#define bit_reset(B, i)		(B &= ~(1ul << i))

#define bit_selectd(B, i)		((B >> i)&0x01ul)
#define bit_setd(B, i)		(B |= (1ul << i))
#define bit_resetd(B, i)		(B &= ~(1ul << i))

#endif

