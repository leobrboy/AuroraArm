#include <stdint.h>
#include "../m4_vectors.h"

float auroraarm_voltage(float theta_ref, float theta, float *integrator,
                        float kp, float ki, float kd, float dtheta,
                        float current, float omega);
float auroraarm_safe_speed_ref(float omega_ref, int collaborative_zone);

static void semihost_write0(const char *text) {
    register uint32_t op __asm("r0") = 0x04; /* SYS_WRITE0 */
    register const char *arg __asm("r1") = text;
    __asm volatile("bkpt 0xab" : "+r"(op) : "r"(arg) : "memory");
}

static void semihost_exit(int status) {
    register uint32_t op __asm("r0") = 0x18; /* SYS_EXIT */
    register uint32_t reason __asm("r1") = status ? 0x20023u : 0x20026u;
    __asm volatile("bkpt 0xab" : "+r"(op) : "r"(reason) : "memory");
    for (;;) { __asm volatile("wfi"); }
}

static unsigned decimal(unsigned value, char *out) {
    char rev[12]; unsigned n=0;
    do { rev[n++]=(char)('0'+value%10u); value/=10u; } while(value && n<sizeof(rev));
    for (unsigned i=0; i<n; ++i) out[i]=rev[n-1-i];
    out[n]='\0'; return n;
}

int main(void) {
    float integral=0.0f, max_error=0.0f;
    unsigned bad_voltage=0;
    for (unsigned i=0; i<M4_VECTOR_COUNT; ++i) {
        const M4Vector *v=&m4_vectors[i];
        float out=auroraarm_voltage(v->ref,v->theta,&integral,800.0f,400.0f,
                                    45.0f,v->dtheta,v->current,v->omega);
        float e=out-v->expected; if(e<0) e=-e;
        if(e>max_error) max_error=e;
        if(out>48.0f || out< -48.0f) ++bad_voltage;
    }
    float safe=auroraarm_safe_speed_ref(450.0f,1);
    float safe_abs=safe<0?-safe:safe;
    unsigned error_microvolts=(unsigned)(max_error*1000000.0f+0.5f);
    if(safe_abs>71.4287f || max_error>1.0e-4f || bad_voltage!=0) {
        char number[12];
        semihost_write0("M4_QEMU_FAIL max_error_uV="); decimal(error_microvolts,number); semihost_write0(number);
        semihost_write0(" voltage_clamp_violations="); decimal(bad_voltage,number); semihost_write0(number);
        semihost_write0(" safe_speed="); decimal((unsigned)(safe_abs*1000.0f),number); semihost_write0(number);
        semihost_write0("\n"); semihost_exit(1);
    }
    char number[12];
    semihost_write0("M4_QEMU_PASS vectors=");
    decimal(M4_VECTOR_COUNT,number); semihost_write0(number);
    semihost_write0(" max_error_uV=");
    decimal(error_microvolts,number); semihost_write0(number);
    semihost_write0(" safe_speed_cap=PASS voltage_clamp=PASS\n");
    semihost_exit(0);
    return 0;
}
