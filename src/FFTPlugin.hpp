#pragma once
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef float real;
typedef struct
{
	real Re;
	real Im;
} complex;

#ifndef PI
#define PI 3.14159265358979323846264338327950288
#endif

inline void fft(complex *v, int n, complex *tmp)
{
	if (n > 1)
	{
		int k, m;
		complex z, w, *vo, *ve;
		ve = tmp;
		vo = tmp + n / 2;
		for (k = 0; k < n / 2; k++)
		{
			ve[k] = v[2 * k];
			vo[k] = v[2 * k + 1];
		}
		fft(ve, n / 2, v);
		fft(vo, n / 2, v);
		for (m = 0; m < n / 2; m++)
		{
			w.Re = cos(2 * PI * m / (double)n);
			w.Im = -sin(2 * PI * m / (double)n);
			z.Re = w.Re * vo[m].Re - w.Im * vo[m].Im;
			z.Im = w.Re * vo[m].Im + w.Im * vo[m].Re;
			v[m].Re = ve[m].Re + z.Re;
			v[m].Im = ve[m].Im + z.Im;
			v[m + n / 2].Re = ve[m].Re - z.Re;
			v[m + n / 2].Im = ve[m].Im - z.Im;
		}
	}
	return;
}
