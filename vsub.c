// Base program for CPU1.s

#define ARRAY_A_ADDR 0x0400
#define ARRAY_B_ADDR 0x0800
#define ARRAY_D_ADDR 0x1000

float * ARRAY_A = (float *)(void *)ARRAY_A_ADDR;
float * ARRAY_B = (float *)(void *)ARRAY_B_ADDR;
float * ARRAY_D = (float *)(void *)ARRAY_D_ADDR;

int main() {
	int i;
	for (i=0; i<256; i++) {
		ARRAY_D[i] = ARRAY_A[i] - ARRAY_B[i];
	}
}