#include <inttypes.h>


#ifdef __cplusplus
extern "C" {
#endif
  extern int32_t filter_dt;

  extern void hsf_init();
  extern void hsf_step();
  extern float* hsf_getState();
  extern void hsf_update_a();
  extern void hsf_update_t();
  extern void hsf_update_p();

  enum hsfStateBits { 
    hsfBMPTemp = 0,
    hsfBMPTempDot = 1, 
    hsfZ = 2,
    hsfZdot = 3,
    hsfZ2dot = 4,
    hsfZ3dot = 5,
    };

#ifdef __cplusplus
}
#endif


