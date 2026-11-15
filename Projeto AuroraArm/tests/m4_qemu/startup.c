#include <stdint.h>

extern int main(void);
extern uint32_t _estack;
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

void Reset_Handler(void) {
    uint32_t *src = &_sidata;
    for (uint32_t *dst = &_sdata; dst < &_edata; ) *dst++ = *src++;
    for (uint32_t *dst = &_sbss; dst < &_ebss; ) *dst++ = 0;
    (void)main();
    for (;;) { __asm volatile("wfi"); }
}

void Default_Handler(void) { for (;;) { __asm volatile("wfi"); } }

__attribute__((section(".isr_vector"), used))
const uintptr_t vectors[] = {
    (uintptr_t)&_estack,
    (uintptr_t)Reset_Handler,
    (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler,
    (uintptr_t)0,
    (uintptr_t)0,
    (uintptr_t)0,
    (uintptr_t)0,
    (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler,
    (uintptr_t)0,
    (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler,
};
