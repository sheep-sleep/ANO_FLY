-c --cpu Cortex-M3 -D__EVAL -g -O0 --apcs=interwork -I..\CMSIS -I..\USER -I..\FWLIB\inc -I..\FWLIB\src -I "D:\Keil 4\ARM\INC" -I "D:\Keil 4\ARM\INC\ST\STM32F10x" -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -o "..\OUTPUT\systick.o" --omf_browse "..\OUTPUT\systick.crf" --depend "..\OUTPUT\systick.d" "systick.c"