/**
  \file       eMatAxisPerm.h
  \brief      Provide compile time calculation of axis permutations (i.e. 90 degree
              rotations and reflections) for aligning sensors.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
  \remark     This file contains some borderline macro abuse to do compile time
              code generation in C without C++ templates by using enums as compile 
              time variables i.e. constants :p
  \remark     The goal here is to produced well optimized, correct code; NOT
              to have a nicely wrapped operators that make expressions look
              nice! (but leave a lot of mess behind). 
*/

/*  Usage :

  // Declare an axis permutation enum-matrix with a basename in the first parameter 
  // using the perm_t enum in the second parameter.

  M_DECLARE_PERM_ENUM( BoardChip, P_XYZ | P_PPP );
  M_DECLARE_PERM_ENUM( ChipAcc,   P_XYZ | P_PPP );
  M_DECLARE_PERM_ENUM( ChipGyro,  P_XYZ | P_PPP );
  M_DECLARE_PERM_ENUM( ChipMag,   P_YXZ | P_PPN );
  
  // The can be combined with a matrix matrix multiply

  EMAT_MUL_EMAT_EMAT( BoardAcc,  BoardChip, ChipAcc  );
  EMAT_MUL_EMAT_EMAT( BoardGyro, BoardChip, ChipGyro );
  EMAT_MUL_EMAT_EMAT( BoardMag,  BoardChip, ChipMag  );

  // and used with any structures that define x,y and z with a matrix vector multiply and
  // turned into an inlinable function.

  struct xyz_t { int x,y,x; } ;

  inline void permAcc(struct xyz_t *o, struct  xyz_t *i)
    {
    EMAT_MUL_EMAT_VEC( o->acc,  BoardAcc, i->acc  );
    }

*/

// Enum for axis permutations, and axis orientations.
// The permutation encoding is little endian, i.e. X is bit 0, Y is bit 1, Z is bit 2 for
// signedness. For permutations X is bits 01, Y is bits 23 and give which column the '1'
// is in for the resultant matrix. The last row is computed from the first two.
// 
// Nomenclature is left to right, so referenced to XYZ.
//
enum perm_t {
  P_XYZ = 0x9 << 4,     P_YXZ = 0x6 << 4,     P_ZYX = 0xb << 4,
  P_XZY = 0xd << 4,     P_YZX = 0xe << 4,     P_ZXY = 0x7 << 4,

  P_PPP = 0,     P_PPN = 4,     P_PNP = 2,     P_PNN = 6,
  P_NPP = 1,     P_NPN = 5,     P_NNP = 3,     P_NNN = 7
  };

#define PERM_T_Lx(X) ( 3 & ((X)>>4)             )
#define PERM_T_Ly(X) ( 3 & ((X)>>6)             )
#define PERM_T_Lz(X) ( 3 & (((X)>>6)^((X)>>4))  )
#define PERM_T_Px(X) ( 1 - (2 & ((X)<<1))       )
#define PERM_T_Py(X) ( 1 - (2 & ((X)   ))       )
#define PERM_T_Pz(X) ( 1 - (2 & ((X)>>1))       )

// Multiply a enum-matrix by an enum-matrix to produce a new enum-matrix : A = B * C
#define EMAT_MUL_EMAT_EMAT(/* name */ A, /* name */ B, /* name */ C) \
  enum A##_t { \
    A##_xx = B##_xx * C##_xx + B##_xy * C##_yx + B##_xz * C##_zx, \
    A##_xy = B##_xx * C##_xy + B##_xy * C##_yy + B##_xz * C##_zy, \
    A##_xz = B##_xx * C##_xz + B##_xy * C##_yz + B##_xz * C##_zz, \
    \
    A##_yx = B##_yx * C##_xx + B##_yy * C##_yx + B##_yz * C##_zx, \
    A##_yy = B##_yx * C##_xy + B##_yy * C##_yy + B##_yz * C##_zy, \
    A##_yz = B##_yx * C##_xz + B##_yy * C##_yz + B##_yz * C##_zz, \
    \
    A##_zx = B##_zx * C##_xx + B##_zy * C##_yx + B##_zz * C##_zx, \
    A##_zy = B##_zx * C##_xy + B##_zy * C##_yy + B##_zz * C##_zy, \
    A##_zz = B##_zx * C##_xz + B##_zy * C##_yz + B##_zz * C##_zz, \
    }

// Multiply a vector (structure that contains x, y and z members).
#define EMAT_MUL_EMAT_VEC( /* structure */ U, /* name */ M, /* structure */ X) \
  U.x = M##_xx * (X).x + M##_xy * (X).y + M##_xz * (X).z; \
  U.y = M##_yx * (X).x + M##_yy * (X).y + M##_yz * (X).z; \
  U.z = M##_zx * (X).x + M##_zy * (X).y + M##_zz * (X).z;

// 
// Make and declare a permutation enum-matrix
#define EMAT_DECLARE_PERM_EMAT( /* name */ M, /* perm_t */ X) \
  enum M##_t { \
    M##_Lx = PERM_T_Lx(X), \
    M##_Ly = PERM_T_Ly(X), \
    M##_Lz = PERM_T_Lz(X), \
    M##_Px = PERM_T_Px(X), \
    M##_Py = PERM_T_Py(X), \
    M##_Pz = PERM_T_Pz(X), \
    \
    M##_xx = (M##_Lx==1)*M##_Px, M##_xy = (M##_Lx==2)*M##_Px, M##_xz = (M##_Lx==3)*M##_Px, \
    M##_yx = (M##_Ly==1)*M##_Py, M##_yy = (M##_Ly==2)*M##_Py, M##_yz = (M##_Ly==3)*M##_Py, \
    M##_zx = (M##_Lz==1)*M##_Pz, M##_zy = (M##_Lz==2)*M##_Pz, M##_zz = (M##_Lz==3)*M##_Pz, \
    }

#define EMAT_FLATTEN_EMAT( /* name */ M ) \
   ( { M##_xx , M##_xy, M##_xz, M##_yx , M##_yy, M##_yz, M##_zx , M##_zy, M##_zz } )

