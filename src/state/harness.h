#include <inttypes.h>


#ifdef __cplusplus
extern "C" {
#endif
  extern int32_t filter_dt;

  extern void hsf_init();
  extern float* hsf_getState();
  extern void hsf_step_tp();
  extern void hsf_step_t();
  extern void hsf_step_p();


#ifdef __cplusplus
}
#endif


