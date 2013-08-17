/**
  \file       model_eqs.c
  \brief      Model equations for Bispoke Kalman filter for height state from a BMP180.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Compiled from model_eqs.tex.
  \remark     Ported for AQ32Plus.
*/

inline float_tt sqr(float_tt x) {
  return x * x;
  }

inline float_tt pf4(float_tt x) {
  return sqr(sqr(x));
  }

inline float_tt pf5(float_tt x) {
  return x*sqr(sqr(x));
  }

inline float_tt zeta_T(float_tt T)
  {
  float_tt s = T - T_0 ;
  return ((x2  * s) + x1) *s + x0 ;
  }

inline float_tt xi_T(float_tt T)
  {
  float_tt s = T - T_0;
  return ((y2  * s) + y1_) *s + y0_ ;
  }

inline float_tt gamma_T(float_tt T) {
  return sqrtf( sqr(T + md_) - 4.0 * mc_ );
  }

inline float_tt theta_T(float_tt T) {
  return c6 + 0.5 * ( T - md_ + gamma_T(T)) / c5;
  }

inline float_tt gamma_P(float_tt P) {
  return sqrtf(sqr(z1) - 4.0 * z2 * (z0 - P) );
  }

inline float_tt phi_PT(float_tt P, float_tt T) {
  return (zeta_T(T) + 0.5 * xi_T(T) * ( - z1 + gamma_P(P)) / z2) / ps;
  }

inline float_tt P_z(float_tt z) {
  return scale * P_0 * pf5( 1.0 - z / z_max);
  }

inline float_tt phi_zT(float_tt z, float_tt T) {
  return phi_PT(P_z(z),T);
  }

inline float_tt d_Tzeta_T(float_tt  T) {
  return 2.0 * x2 * (T - T_0) + x1;
  }

inline float_tt d_Txi_T(float_tt T) {
  return 2.0 * y2 * (T - T_0) + y1_;
  }

inline float_tt d_Tgamma_T(float_tt T) {
  return (T + md_) / gamma_T(T);
  }

inline float_tt d_Ttheta_T(float_tt T) {
  return 0.5 * (1.0 + d_Tgamma_T(T) ) / c5;
  }

inline float_tt d_Tphi_zT(float_tt z, float_tt T ) {
  return d_Tzeta_T(T) + 0.5*d_Txi_T(T) * (-z1 + gamma_P(P_z(z)));
  }

inline float_tt d_zP_z(float_tt z ) {
  return - (scale * rho * P_0 / z_max ) * pf4(1.0 - (z/z_max));
  }

inline float_tt d_Pphi_PT(float_tt P, float_tt T ) {
  return xi_T(T) / gamma_P(P) / ps;
  }

inline float_tt d_zphi_zT(float_tt z, float_tt T ) {
  return d_Pphi_PT(P_z(z), T) * d_zP_z(z);
  }


