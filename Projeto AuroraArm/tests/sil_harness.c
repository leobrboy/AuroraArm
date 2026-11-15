#include <stdio.h>
#include <time.h>

float auroraarm_voltage(float theta_ref, float theta, float *integrator,
                        float kp, float ki, float kd, float dtheta,
                        float current, float omega);
float auroraarm_safe_speed_ref(float omega_ref, int collaborative_zone);

int main(void) {
    FILE *in = fopen("tests/sil_vectors.csv", "r");
    FILE *out = fopen("tests/sil_output.csv", "w");
    if (!in || !out) return 2;
    float r, theta, current, omega, dtheta, integral = 0.0f;
    while (fscanf(in, "%f,%f,%f,%f,%f", &r, &theta, &current, &omega, &dtheta) == 5) {
        float u = auroraarm_voltage(r, theta, &integral, 800.0f, 400.0f,
                                    45.0f, dtheta, current, omega);
        fprintf(out, "%.9g\n", u);
    }
    fclose(in);
    fclose(out);
    if (auroraarm_safe_speed_ref(450.0f, 1) > 71.4287f) return 3;
    if (auroraarm_safe_speed_ref(450.0f, 0) != 450.0f) return 4;
    /* Host-side timing proxy; target-specific WCET still needs Cortex-M4 measurement. */
    volatile float sink = 0.0f;
    integral = 0.0f;
    clock_t start = clock();
    for (int k = 0; k < 1000000; ++k) {
        sink += auroraarm_voltage(0.1f, 0.04f, &integral, 800.0f, 400.0f,
                                  45.0f, 2.0f, 1.0f, 10.0f);
    }
    clock_t elapsed = clock() - start;
    FILE *timing = fopen("tests/host_timing.txt", "w");
    if (!timing) return 5;
    double total_ms = 1000.0 * (double)elapsed / CLOCKS_PER_SEC;
    fprintf(timing, "iterations=1000000\nhost_total_ms=%.6f\nhost_mean_us=%.9f\nsink=%g\n",
            total_ms, total_ms * 1000.0 / 1000000.0, sink);
    fclose(timing);
    return 0;
}
