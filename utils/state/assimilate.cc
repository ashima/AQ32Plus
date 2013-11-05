#include <inttypes.h>
#include <cstdio>
#include <math.h>

#include "char_telem.h"
#include "harness.h"

float aOfp(float p);
float pOfDN(int up,float t);
float tOfDN(int ut);

uint8_t overSamplingSetting = 0;
int ac1 = 0, ac2 = 0, ac3 = 0;
unsigned int ac4 = 0, ac5 = 0, ac6 = 0;
int b1 = 0, b2 = 0, mb = 0, mc = 0, md = 0;

uint32_t rawTemperature;
uint32_t rawPressure;
float earthAxisAccels[3];
float accelOneG = 9.8;

uint32_t lastTimeCode = 0 ;
uint32_t lastTimeSeen = 0 ;
extern "C" {
void dump_filter();
uint32_t micros()
  {
  return lastTimeCode;
  }
}
extern uint32_t lt;

void dump_filter()
  {
  float *st = hsf_getState();
  printf("%f %f %f %f :\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n",
st[0], st[1], st[2], st[3],
st[4], st[5], st[6], st[7],
st[8], st[9], st[10], st[11],
st[12], st[13], st[14], st[15],
st[16], st[17], st[18], st[19] );
  }

void dump_hsf()
  {
//  float *stpt = hsf_getStatePT();
//  printf("%f %f ",stpt[0],stpt[1]);
//  float *stp = hsf_getStatePos();
//  printf("%f %f %f ", stp[0], stp[1], stp[2] );
//  printf("%f %f %f ", stp[3], stp[4], stp[5] );
//  printf("%f %f %f ", stp[6], stp[7], stp[8] );
//  printf("%f %f %f ", stp[9], stp[10], stp[11] );
  float *st = hsf_getState();
  printf("%f %f %f %f ", st[0], st[1], st[2], st[3] );
  printf("%f %f", st[4], st[5] );
  printf("\n");
  }

int main()
  {
  union {
    struct __attribute__((__packed__)) {
      uint16_t mid;
      uint32_t tim;
    };
    uint32_t l_ptr[1];
    uint16_t s_ptr[1];
    uint8_t  c_ptr[64];
  } b;

  int n; int dt = 0;
  float *st = hsf_getState();
  float lastt = 28. , lastp = 1.0e5 , lasta = 0.;

earthAxisAccels[0] = 0. ;
earthAxisAccels[1] = 0. ;
earthAxisAccels[2] = 0. ;

printf("%10d %5d %f %f %f ",lastTimeSeen, dt, earthAxisAccels[0], earthAxisAccels[1], earthAxisAccels[2] );
printf("%f %f %f ", lastt, lastp, lasta );
dump_hsf();

  while (!feof(stdin) )
    {
    n = fread(b.c_ptr, 1, sizeof(b), stdin ) ;
    if ( n == sizeof(b) )
      {
      //printf ("msg : %d, time :%d\n", b.mid, b.tim );
      lastTimeSeen = b.tim;

      switch(b.mid)
        {
        case ctIDBMP180Params:
          ac1 = ((int16_t *)b.s_ptr)[ 3];
          ac2 = ((int16_t *)b.s_ptr)[ 4];
          ac3 = ((int16_t *)b.s_ptr)[ 5];
          ac4 = ((uint16_t *)b.s_ptr)[ 6];
          ac5 = ((uint16_t *)b.s_ptr)[ 7];
          ac6 = ((uint16_t *)b.s_ptr)[ 8];
          b1  = ((int16_t *)b.s_ptr)[ 9];
          b2  = ((int16_t *)b.s_ptr)[10];
          mb  = ((int16_t *)b.s_ptr)[11];
          mc  = ((int16_t *)b.s_ptr)[12];
          md  = ((int16_t *)b.s_ptr)[13];
          overSamplingSetting = b.s_ptr[14];
          //printf("%hd %hd %hd %hu %hu %hu : %hd %hd : %hd %hd %hd : %hd\n", ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md, overSamplingSetting ) ;

          hsf_init();
          //printf("%10d %5d %6d HSF(p): %f %f %f %f %f %f\n",lastTimeSeen, 1, 0, st[0], st[1], st[2], st[3], st[4], st[5] );
printf("%10d %5d %f %f %f ",lastTimeSeen, dt, earthAxisAccels[0], earthAxisAccels[1], earthAxisAccels[2] );
printf("%f %f %f ", lastt, lastp, lasta );
dump_hsf();
          break;
        case ctIDPressure:
          rawPressure = b.c_ptr[6] | (b.c_ptr[7] << 8) | (b.c_ptr[8] << 16) ;
          dt = b.s_ptr[5];
          lastTimeCode = lt + dt;
          hsf_step();
          hsf_update_p();
          //printf("%10d %5d %6d HSF(p): %f %f %f %f %f %f\n",lastTimeSeen, dt, rawPressure, st[0], st[1], st[2], st[3], st[4], st[5] );
          lastp = pOfDN(rawPressure, lastt);
          lasta = aOfp(lastp);
          break;
        case ctIDTemperature:
          rawTemperature = b.s_ptr[3];
          dt = b.s_ptr[4];
          lastTimeCode = lt + dt;
          hsf_step();
          hsf_update_t();
           //printf("%10d %5d %6d HSF(t): %f %f %f %f %f %f\n",lastTimeSeen, dt, rawTemperature, st[0], st[1], st[2], st[3], st[4], st[5] );
          lastt = tOfDN(rawTemperature);
          break;
        case ctIDWAcc100:
          //earthAxisAccels[0] = *((float*) (&b.s_ptr[3]) );
          //earthAxisAccels[1] = *((float*) (&b.s_ptr[5]) );
          earthAxisAccels[2] = *((float*) (&b.s_ptr[7]) );
          if ( fabs(earthAxisAccels[2]) < 100.0 )
            hsf_update_a();
          else
            earthAxisAccels[2] = 0.;
          printf("%10d %5d %f %f %f ",lastTimeSeen, dt, earthAxisAccels[0], earthAxisAccels[1], earthAxisAccels[2] );
          printf("%f %f %f ", lastt, lastp, lasta );
          dump_hsf();
          break;
        }
      }
//    dump_filter();
    }
  
  return 0;
  }
