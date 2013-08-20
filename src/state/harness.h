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


#ifdef __cplusplus
}
#endif


