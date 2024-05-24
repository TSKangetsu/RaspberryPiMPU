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

inline void fftConvert(complex *v, int n, float *PowerInline)
{
	for (size_t i = 0; i < n; i++)
	{
		PowerInline[i] = sqrt((v[i].Re * v[i].Re +
							   v[i].Im * v[i].Im));
	}
}

inline float freqPeak(float *FFTSampleResult, int n, int index, int samplerate)
{
	double delta = 0.5 *
				   ((FFTSampleResult[index - 1] - FFTSampleResult[index + 1]) /
					(FFTSampleResult[index - 1] - (2.0 * FFTSampleResult[index]) +
					 FFTSampleResult[index + 1]));

	double interpolatedX = ((index + delta) * samplerate) / (n - 1);

	if (index == (n >> 1))
		interpolatedX = ((index + delta) * samplerate) / (n);

	return interpolatedX;
}

//====================FFT-APPLICATION=====================//

template <typename T>
std::vector<float> FFTFrequencyAnalyzer(std::vector<T> data, std::vector<T> windowSample)
{
	std::vector<float> FFTout(data.size());
	complex FFTData[data.size()];
	complex FFTDatatmp[data.size()];

	for (size_t i = 0; i < data.size(); i++)
	{
		FFTData[i].Re = (data[i] * 100) * windowSample[i];
		FFTData[i].Im = 0;
	}

	fft(FFTData, data.size(), FFTDatatmp);
	fftConvert(FFTData, data.size(), FFTout.data());

	return FFTout;
}

//===================FFT-WINDOW================================//

template <typename T>
void VecBuildBlackmanHarrisWindow(T *pOut, unsigned int num)
{
	const float a0 = 0.35875f;
	const float a1 = 0.48829f;
	const float a2 = 0.14128f;
	const float a3 = 0.01168f;

	unsigned int idx = 0;
	while (idx < num)
	{
		pOut[idx] = a0 - (a1 * cosf((2.0f * M_PI * idx) / (num - 1))) + (a2 * cosf((4.0f * M_PI * idx) / (num - 1))) - (a3 * cosf((6.0f * M_PI * idx) / (num - 1)));
		idx++;
	}
};

template <typename T>
void VecBuildBlackmanWindow(T *pOut, unsigned int num)
{
	const float a0 = 0.42;
	const float a1 = 0.5;
	const float a2 = 0.08;

	unsigned int idx = 0;
	while (idx < num)
	{
		pOut[idx] = a0 - (a1 * cosf((2.0f * M_PI * idx) / (num - 1))) + (a2 * cosf((4.0f * M_PI * idx) / (num - 1)));
		idx++;
	}
};

template <typename T>
void VecBuildHann_PoissonWindow(T *pOut, unsigned int num)
{
	const float alpha = 2.f;

	for (size_t i = 0; i < num; i++)
	{
		float scd = std::abs(int(num - (2 * i)));
		pOut[i] = ((1.0f - cosf((2.0f * M_PI * (float)i) / ((float)num - 1.f))) * powf(M_E, ((-alpha * scd / ((float)num - 1.f))))) / 2.f;
	}
}