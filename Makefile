
export TARGET=AQ32Plus
export SRC=src
export BUILD=build

export ARMCC=arm-none-eabi-gcc
export ARMAR=arm-none-eabi-ar
export ARMOBJCOPY=arm-none-eabi-objcopy
export ARMOBJDUMP=arm-none-eabi-objdump

TARGETEXT=text.bin isr.bin lst hex



all: $(TARGET).all

.PHONY: clean realclean flash $(SRC)/$(TARGET).elf $(TARGET).all

$(TARGET).all: $(patsubst %,$(BUILD)/$(TARGET).%,$(TARGETEXT)) | $(BUILD)

$(BUILD):
	-mkdir $(BUILD)

$(SRC)/$(TARGET).elf:
	cd $(SRC) ; make -j2 $(TARGET).elf

clean:
	-rm *.o */*.o $(patsubst %,$(BUILD)/$(TARGET).%,$(TARGETEXT))

realclean: clean
	-cd .. ; find . -name '*.o' | xargs rm

flash:
	dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D $(BUILD)/$(TARGET).isr.bin
	dfu-util -d 0483:df11 -a 0 -s 0x08008000 -D $(BUILD)/$(TARGET).text.bin

$(BUILD)/%.text.bin: $(SRC)/%.elf
	$(ARMOBJCOPY) -R .isr_vector -R .eeprom -O binary $^ $@

$(BUILD)/%.isr.bin: $(SRC)/%.elf
	$(ARMOBJCOPY) -j .isr_vector -O binary $^ $@

$(BUILD)/%.lst: $(SRC)/%.elf
	$(ARMOBJDUMP) -SD $^ > $@

$(BUILD)/%.hex: $(SRC)/%.elf
	$(ARMOBJCOPY) -O ihex $^ $@

