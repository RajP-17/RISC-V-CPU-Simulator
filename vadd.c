// Base program for CPU0.s

#define ARRAY_A_ADDR 0x0400
#define ARRAY_B_ADDR 0x0800
#define ARRAY_C_ADDR 0x0C00

float * ARRAY_A = (float *)(void *)ARRAY_A_ADDR;
float * ARRAY_B = (float *)(void *)ARRAY_B_ADDR;
float * ARRAY_C = (float *)(void *)ARRAY_C_ADDR;

int main() {
	int i;
	for (i=0; i<256; i++) {
		ARRAY_C[i] = ARRAY_A[i] + ARRAY_B[i];
	}
}