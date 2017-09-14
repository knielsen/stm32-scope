TARGET=stm32-scope

OBJS = $(TARGET).o led.o dbg.o util.o adc.o st7787.o fft.o

STM_DIR=/kvm/src/STM32F4xx_DSP_StdPeriph_Lib_V1.6.1
STM_SRC = $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src
vpath %.c $(STM_SRC)
STM_OBJS = system_stm32f4xx.o
STM_OBJS  += stm32f4xx_rcc.o
STM_OBJS  += stm32f4xx_gpio.o
STM_OBJS  += stm32f4xx_usart.o
STM_OBJS  += stm32f4xx_tim.o
STM_OBJS  += stm32f4xx_dma.o
STM_OBJS  += stm32f4xx_adc.o
STM_OBJS  += stm32f4xx_syscfg.o
STM_OBJS  += stm32f4xx_exti.o
STM_OBJS  += misc.o

INC_DIRS += $(STM_DIR)/Libraries/CMSIS/Include
INC_DIRS += $(STM_DIR)/Libraries/CMSIS/Device/ST/STM32F4xx/Include
INC_DIRS += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc
INC_DIRS += .
INC = $(addprefix -I,$(INC_DIRS))


CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy


STARTUP_OBJ=startup_stm32f4xx.o
STARTUP_SRC=$(STM_DIR)/Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/TrueSTUDIO/startup_stm32f40xx.s
LINKSCRIPT=$(TARGET).ld

ARCH_FLAGS=-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -ffast-math

CFLAGS=-ggdb -O3 -std=c99 -Wall -Wextra -Warray-bounds -Wno-unused-parameter $(ARCH_FLAGS) $(INC) -DSTM32F40XX -DUSE_STDPERIPH_DRIVER
LDFLAGS=-Wl,--gc-sections -lm


.PHONY: all flash clean tty cat

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf

$(TARGET).elf: $(OBJS) $(STM_OBJS) $(STARTUP_OBJ) $(LINKSCRIPT)
	$(LD) $(ARCH_FLAGS) -T $(LINKSCRIPT) -o $@ $(STARTUP_OBJ) $(OBJS) $(STM_OBJS) $(LDFLAGS)

$(TARGET).o: $(TARGET).c $(TARGET).h

$(STARTUP_OBJ): $(STARTUP_SRC)
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.c $(TARGET).h stm32f4xx_conf.h
	$(CC) $(CFLAGS) -c $< -o $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x8000000

clean:
	rm -f $(OBJS) $(STM_OBJS) $(TARGET).elf $(TARGET).bin $(STARTUP_OBJ)

tty:
	stty -F/dev/stellaris raw -echo -hup cs8 -parenb -cstopb 115200

cat:
	cat /dev/stellaris
