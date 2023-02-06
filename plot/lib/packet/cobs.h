//! \file cobs.h

#ifndef COBS_H_
#define COBS_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


size_t COBSEncode(uint8_t* encodedBuffer,const uint8_t* buffer,size_t size);
size_t COBSDecode(uint8_t* decodedBuffer, const uint8_t* encodedBuffer,size_t size);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif