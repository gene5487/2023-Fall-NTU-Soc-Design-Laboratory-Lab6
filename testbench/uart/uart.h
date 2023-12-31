// fir.h
#ifndef __FIR_H__
#define __FIR_H__

#define N 11

int taps[N] = {0,-10,-9,23,56,63,56,23,-9,-10,0};
int inputbuffer[N];
int inputsignal[N] = {1,2,3,4,5,6,7,8,9,10,11};
int outputsignal[N];
#endif

// matmul.h
#ifndef _MATMUL_H
#define _MATMUL_H

#define SIZE 4
	int A[SIZE*SIZE] = {0, 1, 2, 3,
			0, 1, 2, 3,
			0, 1, 2, 3,
			0, 1, 2, 3,
	};
	int B[SIZE*SIZE] = {1, 2, 3, 4,
		5, 6, 7, 8,
		9, 10, 11, 12,
		13, 14, 15, 16,
	};
	int result[SIZE*SIZE];
#endif

// qsort.h
#ifndef _QSORT_H
#define _QSORT_H

#define SIZE_Q 10
int A_Q[SIZE_Q] = {893, 40, 3233, 4267, 2669, 2541, 9073, 6023, 5681, 4622};
#endif
