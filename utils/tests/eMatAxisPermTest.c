/**
  \file       eMatAxisPermTest.c
  \brief      Quick test functions for eMatAxisPerm.h
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
  \remark     Need to more than a cursory check.
*/

#include <stdio.h>
#include <inttypes.h>

#include "eMatAxisPerm.h"

typedef struct
  {
  int x;
  int y;
  int z;
  } xyz_t ;

typedef struct
  {
  xyz_t acc;
  xyz_t gyro;
  xyz_t mag;
  } agm_t;

EMAT_DECLARE_PERM_EMAT( BoardChip, P_XYZ | P_PPP );
EMAT_DECLARE_PERM_EMAT( ChipAcc,   P_XYZ | P_PPP );
EMAT_DECLARE_PERM_EMAT( ChipGyro,  P_XYZ | P_PPP );
EMAT_DECLARE_PERM_EMAT( ChipMag,   P_YXZ | P_PPN );

EMAT_MUL_EMAT_EMAT( BoardAcc,  BoardChip, ChipAcc  );
EMAT_MUL_EMAT_EMAT( BoardGyro, BoardChip, ChipGyro );
EMAT_MUL_EMAT_EMAT( BoardMag,  BoardChip, ChipMag  );

void orientSensors(agm_t *o, agm_t *i)
  {
  EMAT_MUL_EMAT_VEC( o->acc,  BoardAcc,  i->acc  );
  EMAT_MUL_EMAT_VEC( o->gyro, BoardGyro, i->gyro );
  EMAT_MUL_EMAT_VEC( o->mag,  BoardMag,  i->mag  );
  }

void test1()
  {
  xyz_t i,j;
  i.x = 1;
  i.y = i.z = 0;

  EMAT_MUL_EMAT_VEC( j, BoardMag, i );
  printf("%d %d %d\n", j.x, j.y, j.z );

  i.y = 1;
  i.x = i.z = 0;

  EMAT_MUL_EMAT_VEC( j, BoardMag, i );
  printf("%d %d %d\n", j.x, j.y, j.z );

  i.z = 1;
  i.y = i.x = 0;

  EMAT_MUL_EMAT_VEC( j, BoardMag, i );
  printf("%d %d %d\n", j.x, j.y, j.z );
  }

int main()
  {
  test1();
  return 0;
  }
