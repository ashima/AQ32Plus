/**
  \brief      DualShock 3 SixAxis to Spectrum Satillite Rx over UDP/Serial.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <libusb-1.0/libusb.h>
#include <signal.h>
#include <termios.h>

enum 
  {
  false = 0,
  true = 1
  } ;

enum 
  {
  use_ip = true,
  print_every = 20, 
  interface = 0,
  };

/* hard coded for now
 */
char deststr[]="192.168.1.64";
int destport=9750;

char serfile[]="/dev/tty.usbserial-A700eEnw";
int serspeed = B115200;

/*
 Clean up.
 */
libusb_device_handle *dev= NULL;

void sig_handler(int signo)
  {
  if (signo == SIGINT)
    fprintf(stderr, "SIGINT\n");
  if (dev)
    { // I think there needs to be more USB clean up here!
    usleep(300*1000);
    libusb_release_interface(dev,interface);
    libusb_attach_kernel_driver(dev,interface);
    libusb_close(dev);  
    }
  exit(1);
  }

/*
 Internet setup.
 */

struct sockaddr_in dest;

int netInit(char* deststr, uint32_t port)
  {
  int fd;
  int i;  
  int o;
  struct timeval tv;

  if ( (fd=socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
    perror("socket error");
    exit(1);
    }

  if ( inet_aton( deststr, &dest.sin_addr) == 0)
    {
    perror("address error");
    exit(1);
    }
  o = 64;
  i=setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &o, sizeof(o) );
  //fprintf(stderr,"Sendbuf = %d, %d\n", i, errno);

#if 0
  o = 1;
  i=setsockopt(fd, SOL_SOCKET, SO_SNDLOWAT, &o, sizeof(o) );
  fprintf(stderr,"Sendlowat = %d, %d\n", i, errno);
  o = 6;
  i=setsockopt(fd, SOL_SOCKET, SO_PRIORITY, &o, sizeof(o) );
  fprintf(stderr,"priority = %d, %d\n", i, errno);
#endif

  tv.tv_sec = 0;
  tv.tv_usec = 1;
  i=setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
//  fprintf(stderr,"\nSendTimeOut = %d, %d\n", i, errno);

  dest.sin_family = AF_INET;
  dest.sin_port   = htons(port);

  return fd;
  }


/*
  Serial Initalization.
 */

int serInit(char *file, int speed)
  {
  int fd;
  int i;
  struct termios tty;

  if (-1 == (fd = open(file,O_RDWR | O_NONBLOCK)))
    perror("Couldn't open serial file");
  if (-1 == (i = fcntl(fd, F_SETFL, O_NONBLOCK)))
    perror("Couldn't set serial fd to non blocking");

  memset (&tty, 0, sizeof tty);
  if (-1 == tcgetattr (fd, &tty) )
    perror("Error getting serial attributes");

  if (-1 == cfsetospeed (&tty, speed) || -1 == cfsetispeed (&tty, speed) )
    perror("Error setting serial baud rage");

  tty.c_cflag      = (tty.c_cflag & ~CSIZE) | CS8; 
  tty.c_iflag     &= ~IGNBRK;
  tty.c_lflag      = 0;
  tty.c_oflag      = 0;
  tty.c_cc[VMIN]   = 0;
  tty.c_cc[VTIME]  = 5;
  tty.c_iflag     &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag     |=  (CLOCAL | CREAD);
  tty.c_cflag     &= ~(PARENB | PARODD);
  tty.c_cflag     |= 0;
  tty.c_cflag     &= ~CSTOPB;
  tty.c_cflag     &= ~CRTSCTS;

  if (-1 == tcsetattr (fd, TCSANOW, &tty))
    perror("Couldn't set serial port attribtues");

  return fd;
  }

/*
 USB initalization section.
 */
libusb_device_handle* usbInit()
  {
  libusb_device **devs;
  libusb_context *ctx = NULL;

  int i;

  if ((i=libusb_init(&ctx)) < 0 )
    {
    fprintf(stderr, "Usb Init failure (%d)\n", i);
    exit(1); 
    }
  libusb_set_debug(ctx,2);
  if (( i = libusb_get_device_list(ctx, &devs) ) < 0)
    {
    fprintf(stderr, "Usb Device list failure (%d)\n", i);
    exit(1); 
    };
   
  //fprintf(stderr, "Found %d devices\n", i);
  
  if (NULL == (dev = libusb_open_device_with_vid_pid( ctx, 0x054c, 0x0268)))
    {
    fprintf(stderr, "Usb Device open failure. Is the controler plugged in?\n");
    exit(1); 
    };
    
  i = libusb_detach_kernel_driver(dev,interface); 
  //fprintf(stderr,"detach kernal returned %d\n", i);

  i = libusb_claim_interface(dev, interface);
  //fprintf(stderr,"claim interface returned %d\n", i);
  return dev;
  }

int get6AxisPacket(libusb_device_handle *dev, uint8_t *buf)
  {
  return libusb_control_transfer(dev, 0xa1, 1, 0x0101, 0, buf, 0x31, 0);
  }

/*
  Dualshock 3 structures.
 */
enum {
  un0,un1,
  leftD, rightD,
  PS_POWER_BUTTON,
  un5,
  LS_leftright, LS_downup,
  RS_leftright, RS_downup,
  un10,u11,u12,u13,
  LD_U,LD_R, LD_D, LD_L,
  LT_lower, RT_lower,
  LT_upper, RT_upper,
  RD_U, RD_R, RD_D, RD_L,
  un26,un27,un28,un29,un30,
  MOTOR1,
  un32,un33,un34,un35,un36,un37,un38,un39,
  MOTOR2,
  HITROLL, ROLL,
  HITPITCH,PITCH,
  HITYAW,  YAW,
  un47,un48,
  };

typedef union
  {
  uint8_t pkt[49];
  struct
    {
    uint8_t un0[2];
    uint8_t leftD, rightD;
    uint8_t PS_POWER_BUTTON;
    uint8_t un5;
    uint8_t LS_leftright, LS_downup;
    uint8_t RS_leftright, RS_downup;
    uint8_t un10[4]; //10-13
    uint8_t LD_U,LD_R, LD_D, LD_L;
    uint8_t LT_lower, RT_lower;
    uint8_t LT_upper, RT_upper;
    uint8_t RD_U, RD_R, RD_D, RD_L;
    uint8_t un26[5]; // 26-30
    uint8_t MOTOR1;
    uint8_t un32[8];  // 32-39
    uint8_t MOTOR2;
    uint8_t HITROLL, ROLL;
    uint8_t HITPITCH,PITCH;
    uint8_t HITYAW,  YAW;
    uint8_t un47[2];
    };
  } sixaxis_t;

/*
 Rx structures.
 */
typedef union
  {
  uint8_t pkt[16];
  struct
    {
    uint16_t s;
    uint16_t ch[7];
    };
  } rx_t;
  

/* 
  Globak buffers
 */
sixaxis_t sixbuf[4];
rx_t      rx_buf;

/*
 Conversion Functions.
 */
uint16_t sw(uint16_t x)
  {
  return (x>>8)|(x<<8);
  }

uint16_t encode(uint16_t x, uint8_t axis, int rev)
  {
  if (rev)
     x = 1023 - x;
  return sw( (axis<<10)|x ) ; 
  }

uint16_t scaleDX(uint8_t x, uint8_t axis, int rev)
  {
  if (rev)
     x = 255 - x;
  return (axis<<10)|(x*4);
  }

uint16_t accChan(sixaxis_t* sb, uint32_t ch, uint32_t N)
  {
  uint32_t v = 0; 
  uint32_t i;

  for (i=0; i < N; ++i)
    v += sb[i].pkt[ch] ;

  return v;
  }

void sixToSRx(sixaxis_t *s, rx_t *t)
  {
  t->s = sw(0x0301);
#ifndef ACCUMULATE
  t->ch[0] = /* aileron */  sw(scaleDX(s->RS_leftright, 1, false));
  t->ch[1] = /* Elevator */ sw(scaleDX(s->RS_downup,    2, true ));
  t->ch[2] = /* Rudder */   sw(scaleDX(s->LS_leftright, 3, false));
  t->ch[3] = /* Throttle */ sw(scaleDX(s->LS_downup,    0, true ));
#else
  int n = 2;
  t->ch[0] = /* aileron */  encode( 4*accChan(s, RS_leftright, n)/n, 1, false);
  t->ch[1] = /* Elevator */ encode( 4*accChan(s, RS_downup,    n)/n, 2, true );
  t->ch[2] = /* Rudder */   encode( 4*accChan(s, LS_leftright, n)/n, 3, false);
  t->ch[3] = /* Throttle */ encode( 4*accChan(s, LS_downup,    n)/n, 0, true );
#endif
  t->ch[4] = /* flaps */     sw((4 << 10) | 0);
  t->ch[5] = /* gear */      sw((5 << 10) | 0);
  t->ch[6] = /* Aux2 */      sw((6 << 10) | 0);
  }

uint64_t gettime()
  {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return (((uint64_t)tv.tv_sec)<<32) | (uint64_t)tv.tv_usec ;
  }

int main(int argc, char *argv[])
  {
  int i,j,c=0;
  uint64_t t1,t2;
  int fd;
  
  if (SIG_ERR == signal(SIGINT, sig_handler) )
    fprintf(stderr,"\ncan't catch SIGINT\n");

  fd = use_ip ? netInit(deststr, destport) : serInit(serfile,serspeed);

  libusb_device_handle *dev = usbInit();
  while(1) {
    t1 = gettime();
    if ( 0x31 == get6AxisPacket(dev,sixbuf[0].pkt) )
      { 
      sixToSRx(sixbuf, &rx_buf);
      i = sendto(fd,rx_buf.pkt, sizeof(rx_buf), MSG_DONTWAIT,
           (struct sockaddr*)&dest, sizeof(dest) );
      if (0==(c % print_every))
        { 
        for (j=0; j < 16 ; j+=2)
          printf ( "%02hhx%02hhx ", rx_buf.pkt[j],rx_buf.pkt[j+1] );
        printf ("(%4d, %4d)", i, errno);
        }
      }
    t2 = gettime();
    if (0==(c % print_every))
      printf ("(%d)\n", (uint32_t)(t2-t1 ));
    ++c;
    usleep(12000);
    }
  }
