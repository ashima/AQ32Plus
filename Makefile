
export TARGET=AQ32Plus
export SRC=src
export BUILD=build

export ARMCC=arm-none-eabi-gcc
export ARMCCC=arm-none-eabi-g++
export ARMAR=arm-none-eabi-ar
export ARMOBJCOPY=arm-none-eabi-objcopy
export ARMOBJDUMP=arm-none-eabi-objdump

TARGETEXT=text.bin isr.bin lst hex

all: $(TARGET).all

.PHONY: clean realclean flash $(SRC)/$(TARGET).elf $(TARGET).all

$(TARGET).all: $(patsubst %,$(BUILD)/$(TARGET).%,$(TARGETEXT))

$(BUILD):
	-mkdir $(BUILD)

$(SRC)/$(TARGET).elf: 
	cd $(SRC) ; make -j2 $(TARGET).elf

clean:
	cd $(SRC) ; make clean

realclean: clean
	-find . -name '*.o' | xargs rm
	-rmdir $(BUILD)

flash:
	dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D $(BUILD)/$(TARGET).isr.bin
	dfu-util -d 0483:df11 -a 0 -s 0x08008000 -D $(BUILD)/$(TARGET).text.bin

$(BUILD)/%.text.bin: $(SRC)/%.elf | $(BUILD)
	$(ARMOBJCOPY) -R .isr_vector -R .eeprom -O binary $^ $@

$(BUILD)/%.isr.bin: $(SRC)/%.elf | $(BUILD)
	$(ARMOBJCOPY) -j .isr_vector -O binary $^ $@

$(BUILD)/%.lst: $(SRC)/%.elf | $(BUILD)
	$(ARMOBJDUMP) -SD $^ > $@

$(BUILD)/%.hex: $(SRC)/%.elf | $(BUILD)
	$(ARMOBJCOPY) -O ihex $^ $@

