#include "memory_type.h"
Memory cpuMemory_init(void);
#define CPU_MEMORY_FIRST_ADDRESS 0x0000
#define CPU_MEMORY_LAST_ADDRESS 0xFFFF
#define CPU_TOTAL_MEMORY_ADDRESSES (CPU_MEMORY_LAST_ADDRESS + 1)
#define CPU_RESET_VECTOR_LOWER_ADDRESS 0xFFFC
#define CPU_RESET_VECTOR_UPPER_ADDRESS (CPU_RESET_VECTOR_LOWER_ADDRESS + 1)
#define CPU_IRQ_VECTOR_LOWER_ADDRESS 0xFFFE
#define CPU_IRQ_VECTOR_UPPER_ADDRESS (CPU_IRQ_VECTOR_LOWER_ADDRESS + 1)
#define CPU_NMI_VECTOR_LOWER_ADDRESS 0xFFFA
#define CPU_NMI_VECTOR_UPPER_ADDRESS (CPU_NMI_VECTOR_LOWER_ADDRESS + 1)
#define CPU_PROGRAM_LOWER_FIRST_ADDRESS 0x8000
#define CPU_PROGRAM_LOWER_LAST_ADDRESS 0xBFFF
#define CPU_PROGRAM_UPPER_FIRST_ADDRESS 0xC000
#define CPU_PROGRAM_UPPER_LAST_ADDRESS 0xFFFF
#define CPU_GENUINE_RAM_FIRST_ADDRESS 0x000
#define CPU_GENUINE_RAM_LAST_ADDRESS 0x07FF
#define CPU_RAM_MIRRORED_SIZE (CPU_GENUINE_RAM_LAST_ADDRESS -
CPU_GENUINE_RAM_FIRST_ADDRESS + 1)
#define CPU_RAM_MIRROR_FIRST_ADDRESS (CPU_GENUINE_RAM_LAST_ADDRESS + 1)
#define CPU_RAM_MIRROR_LAST_ADDRESS 0x1FFF
#define CPU_RAM_MIRROR_SIZE (CPU_RAM_MIRROR_LAST_ADDRESS - CPU_RAM_MIRROR_FIRST_ADDRESS
+ 1)
#define CPU_NUM_RAM_MIRRORS (CPU_RAM_MIRROR_SIZE / CPU_RAM_MIRRORED_SIZE)
#define CPU_GENUINE_PPU_FIRST_ADDRESS 0x2000
#define CPU_GENUINE_PPU_LAST_ADDRESS 0x2007
#define CPU_PPU_MIRRORED_SIZE (CPU_GENUINE_PPU_LAST_ADDRESS -
CPU_GENUINE_PPU_FIRST_ADDRESS +1)
#define CPU_PPU_MIRROR_FIRST_ADDRESS (CPU_GENUINE_PPU_LAST_ADDRESS + 1)
#define CPU_PPU_MIRROR_LAST_ADDRESS 0x3FFF
#define CPU_PPU_MIRROR_SIZE (CPU_PPU_MIRROR_LAST_ADDRESS - CPU_PPU_MIRROR_FIRST_ADDRESS
+ 1)
#define CPU_NUM_PPU_MIRRORS (CPU_PPU_MIRROR_SIZE / CPU_PPU_MIRRORED_SIZE)
#define CPU_PPU_CONTROL_REGISTER_ADDRESS                0x2000 // write
#define CPU_PPU_MASK_REGISTER_ADDRESS                   0x2001 // write
#define CPU_PPU_STATUS_REGISTER_ADDRESS                 0x2002 // read
// internal object attribute memory index pointer (64 attributes, 32 bits
// each, byte granular access). stored value post-increments on access to port 4.
#define CPU_PPU_SPRITE_ADDRESS_REGISTER_ADDRESS         0x2003 // write
// returns object attribute memory location indexed by port 3, then increments port 3.
#define CPU_PPU_SPRITE_DATA_REGISTER_ADDRESS            0x2004 // write
// 5    -      scroll offset port.
#define CPU_PPU_SCROLL_REGISTER_ADDRESS                 0x2005 // write
// 6    -      PPU address port to access with port 7.
#define CPU_PPUMEMORY_ADDRESS_REGISTER_ADDRESS          0x2006 // write
// 7    -      PPU memory write port.
#define CPU_PPUMEMORY_DATA_REGISTER_ADDRESS             0x2007 // read/write
#define CPU_APU_PULSE_1_CONTROL_REGISTER_ADDRESS        0x4000 // write
#define CPU_APU_PULSE_1_RAMP_CONTROL_REGISTER_ADDRESS   0x4001 // write
#define CPU_APU_PULSE_1_FINE_TUNE_REGISTER_ADDRESS      0x4002 // write
#define CPU_APU_PULSE_1_COARSE_TUNE_REGISTER_ADDRESS    0x4003 // write
#define CPU_APU_PULSE_2_CONTROL_REGISTER_ADDRESS        0x4004 // write
#define CPU_APU_PULSE_2_RAMP_CONTROL_REGISTER_ADDRESS   0x4005 // write
#define CPU_APU_PULSE_2_FINE_TUNE_REGISTER_ADDRESS      0x4006 // write
#define CPU_APU_PULSE_2_COARSE_TUNE_REGISTER_ADDRESS    0x4007 // write
#define CPU_APU_TRIANGLE_CONTROL_REGISTER_1_ADDRESS     0x4008 // write
#define CPU_APU_TRIANGLE_CONTROL_REGISTER_2_ADDRESS     0x4009 // write
#define CPU_APU_TRIANGLE_FREQUENCY_REGISTER_1_ADDRESS   0x400A // write
#define CPU_APU_TRIANGLE_FREQUENCY_REGISTER_2_ADDRESS   0x400B // write
#define CPU_APU_NOISE_CONTROL_REGISTER_1_ADDRESS        0x400C // write
                                                 //     0x400D ??
#define CPU_APU_NOISE_FREQUENCY_REGISTER_1_ADDRESS      0x400E // write
#define CPU_APU_NOISE_FREQUENCY_REGISTER_2_ADDRESS      0x400F // write
#define CPU_APU_DELTA_CONTROL_REGISTER_ADDRESS          0x4010 // write
#define CPU_APU_DELTA_DA_REGISTER_ADDRESS               0x4011 // write
#define CPU_APU_DELTA_ADDRESS_REGISTER_ADDRESS          0x4012 // write
#define CPU_APU_DELTA_DATE_LENGTH_REGISTER_ADDRESS      0x4013 // write
#define CPU_SPRITE_DMA_REGISTER_ADDRESS                 0x4014 // write
#define CPU_APU_VERTICAL_CLOCK_REGISTER_ADDRESS         0x4015 // read/write
#define CPU_JOYPAD_0_ADDRESS                            0x4016 // read/write
#define CPU_JOYPAD_1_ADDRESS                            0x4017 // read/write