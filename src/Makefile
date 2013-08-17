
BUILD=../$(BUILDDIR)
LIBS=../Libraries

STDPERIPH=$(LIBS)/STM32F4xx_StdPeriph_Driver
STDUSB=$(LIBS)/STM32_USB_Device_Library
STDOTG=$(LIBS)/STM32_USB_OTG_Driver

CFLAGS= -O3 -fsigned-char -mthumb -mthumb-interwork -mcpu=cortex-m4 \
        -Wl,--gc-sections -ffunction-sections -fdata-sections -Wall \
	-march=armv7e-m -fno-math-errno -fno-trapping-math -fno-signaling-nans 
#-mfpu=fpv4-sp-d16 -mfloat-abi=hard -fsingle-precision-constant --fast-math 

CCFLAGS= -fno-rtti -Wnoexcept -fno-exceptions -Weffc++ -fnothrow-opt -std=c++11

INCDIRS=$(LIBS)/CMSIS/Include \
     $(LIBS)/CMSIS/Device/ST/STM32F4xx/Include \
     $(STDUSB)/Core/inc \
     $(wildcard $(STDUSB)/Class/*/inc/) \
     $(STDOTG)/inc \
     $(STDPERIPH)/inc \
     vcp drv sensors calibration gps max7456 state .
     
DEFS=-DUSE_USB_OTG_FS -DUSE_STDPERIPH_DRIVER -DSTM32F40XX -DHSE_VALUE=16000000
DEFS+=-DASHIMACORE -DSTVGPS -DNOSPI
#DEFS+=-D__FPU_PRESENT=1 -D__FPU_USED=1

INCS=$(patsubst %, -I %,$(INCDIRS))
USBSRC= cdc/src/usbd_cdc_core.c 
DRVSRC=drv_adc.c drv_cli.c drv_i2c.c drv_led.c \
	drv_pwmEsc.c drv_pwmServo.c drv_rx.c drv_system.c \
	drv_telemetry.c drv_timingFunctions.c \
	drv_crc.c
GPSSRC=drv/drv_gps.c gps/skyTraq.c gps/gps.c
#GPSSRC=
SENSRC=hmc5883.c bmp085.c mpu6000_I2C.c
OSDSRC=$(wildcard max7456/*.c)
STATE=char_telem.c

SRCS=$(wildcard *.c) \
     $(patsubst %,drv/%,$(DRVSRC)) \
     $(patsubst %,sensors/%,$(SENSRC)) \
     $(patsubst %,state/%,$(STATE)) \
     $(wildcard vcp/*.c) \
     $(wildcard calibration/*.c) \
     $(STDOTG)/src/usb_core.c \
     $(STDOTG)/src/usb_dcd_int.c \
     $(STDOTG)/src/usb_dcd.c \
     $(wildcard $(STDUSB)/Core/src/*.c) \
     $(patsubst %, $(STDUSB)/Class/%, $(USBSRC) ) \
     $(wildcard $(STDPERIPH)/src/*.c) \
     $(GPSSRC)

CCSRCS=$(wildcard state/*.cc)

OBJS=$(CCSRCS:.cc=.o) $(SRCS:.c=.o) startup/startup_stm32f40xx.o
LINKFILE=../stm32_flash.ld

all:
	@echo 'Target not set. Should be called from parent Makefile.'
	@echo 'To call this makefile directly use:'
	@echo '  env TARGET=<target> make <target>.elf'

$(TARGET).elf: $(OBJS)
	$(ARMCC) $(CFLAGS) -T $(LINKFILE) -nodefaultlibs \
	-o $@ $^  -Wl,--print-map > $(TARGET).mapfile

%.o: %.S
	$(ARMCC) $(CFLAGS) -o $@ -c $^
 
%.o: %.c
	$(ARMCC) $(CFLAGS) $(DEFS) $(CMSISINC) $(INCS) -o $@ -c $^

%.o: %.cc
	$(ARMCCC) $(CFLAGS) $(CCFLAGS) $(DEFS) $(CMSISINC) $(INCS) -o $@ -c $^

%.s: %.c
	$(ARMCC) $(CFLAGS) $(DEFS) $(CMSISINC) $(INCS) -o $@ -S $^

%.s: %.cc
	$(ARMCCC) $(CFLAGS) $(CCFLAGS) $(DEFS) $(CMSISINC) $(INCS) -o $@ -S $^

.PHONY: clean 

clean:
	-rm *.o */*.o $(TARGET).elf $(TARGET).mapfile

