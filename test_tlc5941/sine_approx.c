//*****************************************************************************
//
// sine_approx.c - Sinewave approximation for PWM DAC
//
// Copyright (c) 2013 Idiotronics Incorporated.  All rights reserved.
// 
// This is part of revision 1.0 of the Idiotbox Firmware Package.
//
//*****************************************************************************

//*****************************************************************************
//
// This program will compute a sine value given the phase. Phase runs from 0
// to 65535. The output is scaled so that full scale runs from Y1 to Y2, where
// Y1 is 10% and Y2 is 90% of the counts in the PWM waveform.
//
//*****************************************************************************

#include <stdint.h>
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"

//*****************************************************************************
//
// Initialized data
//
//*****************************************************************************

// Sine wave table with full scale = pi * 2^13
static int16_t sinTab[32] = {
  0, 5021, 9849, 14298, 18198, 21399, 23777, 25241,
  25736, 25241, 23777, 21399, 18198, 14298, 9849, 5021,
  0, -5021, -9849, -14298, -18198, -21399, -23777, -25241,
  -25736, -25241, -23777, -21399, -18198, -14298, -9849, -5021
};

// Table of 2^17/pi * cos(x), x = (-16, ..., 16) * 2*pi/1024
static uint16_t resCosTab[33] = {
  41521, 41545, 41568, 41589, 41608, 41627, 41643, 41658,
  41671, 41683, 41693, 41702, 41709, 41714, 41718, 41721,
  41722, 41721, 41718, 41714, 41709, 41702, 41693, 41683,
  41671, 41658, 41643, 41627, 41608, 41589, 41568, 41545,
  41521
};

int32_t SineApprox(uint16_t phase, uint16_t scale)
{
  uint32_t idx;
  uint32_t resIdx;
  int32_t resQ, sinQ, cosQ, dcos;
  int32_t sinVal;

  idx = (phase + 1024) >> 11;
  resQ = (phase - (idx << 11)) << 2;
  resIdx = (resQ + 4096 + 128) >> 8;

  sinQ = sinTab[idx & 0x1F];
  cosQ = sinTab[(idx + 8) & 0x1F];
  dcos = resCosTab[resIdx];
  sinVal = (sinQ * dcos + cosQ * resQ + 16384) >> 15;
  return((sinVal * scale + 16384) >> 15);
}
