#include <inttypes.h>


#ifdef __cplusplus
extern "C" {
#endif

  extern void hsf_init(float);
  extern void hsf_step();
  extern float* hsf_getState();
#ifdef __cplusplus
}
#endif


