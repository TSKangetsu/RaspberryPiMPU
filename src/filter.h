/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

//this file is redesign by TSKangetsu
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#define UNUSED(x) (void)(x)
#define M_PIf 3.14159265358979323846f

#define sinPolyCoef3 -1.666665710e-1f // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5 8.333017292e-3f  // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9 2.600054768e-6f  // Double:  2.600054767890361277123254766503271638682e-6

inline float sin_approxCF(float x)
{
    int32_t xint = x;
    if (xint < -32 || xint > 32)
        return 0.0f; // Stop here on error input (5 * 360 Deg)
    while (x > M_PIf)
        x -= (2.0f * M_PIf); // always wrap input angle to -PI..PI
    while (x < -M_PIf)
        x += (2.0f * M_PIf);
    if (x > (0.5f * M_PIf))
        x = (0.5f * M_PIf) - (x - (0.5f * M_PIf)); // We just pick -90..+90 Degree
    else if (x < -(0.5f * M_PIf))
        x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

inline float cos_approxCF(float x)
{
    return sin_approxCF(x + (0.5f * M_PIf));
}

inline float constrainf(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

typedef struct rateLimitFilter_s
{
    float state;
} rateLimitFilter_t;

typedef struct pt1Filter_s
{
    float state;
    float RC;
    float dT;
    float alpha;
} pt1Filter_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s
{
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;

typedef union
{
    biquadFilter_t biquad;
    pt1Filter_t pt1;
} filter_t;

typedef enum
{
    FILTER_PT1 = 0,
    FILTER_BIQUAD
} filterType_e;

typedef enum
{
    FILTER_LPF,
    FILTER_NOTCH
} biquadFilterType_e;

typedef struct firFilter_s
{
    float *buf;
    const float *coeffs;
    uint8_t bufLength;
    uint8_t coeffsLength;
} firFilter_t;

typedef struct alphaBetaGammaFilter_s
{
    float a, b, g, e;
    float ak; // derivative of system velociy (ie: acceleration)
    float vk; // derivative of system state (ie: velocity)
    float xk; // current system state (ie: position)
    float jk; // derivative of system acceleration (ie: jerk)
    float rk; // residual error
    float dT, dT2, dT3;
    float halfLife, boost;
    pt1Filter_t boostFilter;
} alphaBetaGammaFilter_t;

// typedef float (*filterApplyFnPtr)(void *filter, float input);
// typedef float (*filterApply4FnPtr)(void *filter, float input, float f_cut, float dt);

#define BIQUAD_BANDWIDTH 1.9f       /* bandwidth in octaves */
#define BIQUAD_Q 1.0f / sqrtf(2.0f) /* quality factor - butterworth*/

inline float nullFilterApply(void *filter, float input)
{
    UNUSED(filter);
    return input;
};
inline float nullFilterApply4(void *filter, float input, float f_cut, float dt)
{
    UNUSED(filter);
    UNUSED(f_cut);
    UNUSED(dt);
    return input;
};

inline float pt1ComputeRC(const float f_cut)
{
    return 1.0f / (2.0f * M_PIf * f_cut);
}

inline void pt1FilterInitRC(pt1Filter_t *filter, float tau, float dT)
{
    filter->state = 0.0f;
    filter->RC = tau;
    filter->dT = dT;
    filter->alpha = filter->dT / (filter->RC + filter->dT);
};
inline void pt1FilterInit(pt1Filter_t *filter, float f_cut, float dT)
{
    pt1FilterInitRC(filter, pt1ComputeRC(f_cut), dT);
};
inline void pt1FilterSetTimeConstant(pt1Filter_t *filter, float tau)
{
    filter->RC = tau;
};
inline void pt1FilterUpdateCutoff(pt1Filter_t *filter, float f_cut)
{
    filter->RC = pt1ComputeRC(f_cut);
    filter->alpha = filter->dT / (filter->RC + filter->dT);
};
inline float pt1FilterGetLastOutput(pt1Filter_t *filter)
{
    return filter->state;
};
inline float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->alpha * (input - filter->state);
    return filter->state;
};
inline float pt1FilterApply3(pt1Filter_t *filter, float input, float dT)
{
    filter->dT = dT;
    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);
    return filter->state;
};
inline float pt1FilterApply4(pt1Filter_t *filter, float input, float f_cut, float dT)
{
    // Pre calculate and store RC
    if (!filter->RC)
    {
        filter->RC = pt1ComputeRC(f_cut);
    }
    filter->dT = dT; // cache latest dT for possible use in pt1FilterApply
    filter->alpha = filter->dT / (filter->RC + filter->dT);
    filter->state = filter->state + filter->alpha * (input - filter->state);
    return filter->state;
};
inline void pt1FilterReset(pt1Filter_t *filter, float input)
{
    filter->state = input;
};

inline void rateLimitFilterInit(rateLimitFilter_t *filter)
{
    filter->state = 0;
};
inline float rateLimitFilterApply4(rateLimitFilter_t *filter, float input, float rate_limit, float dT)
{
    if (rate_limit > 0)
    {
        const float rateLimitPerSample = rate_limit * dT;
        filter->state = constrainf(input, filter->state - rateLimitPerSample, filter->state + rateLimitPerSample);
    }
    else
    {
        filter->state = input;
    }

    return filter->state;
};

inline void biquadFilterSetupPassthrough(biquadFilter_t *filter)
{
    // By default set as passthrough
    filter->b0 = 1.0f;
    filter->b1 = 0.0f;
    filter->b2 = 0.0f;
    filter->a1 = 0.0f;
    filter->a2 = 0.0f;
}
inline void biquadFilterInit(biquadFilter_t *filter, uint16_t filterFreq, uint32_t samplingIntervalUs, float Q, biquadFilterType_e filterType)
{
    // Check for Nyquist frequency and if it's not possible to initialize filter as requested - set to no filtering at all
    if (filterFreq < (1000000 / samplingIntervalUs / 2))
    {
        // setup variables
        const float sampleRate = 1.0f / ((float)samplingIntervalUs * 0.000001f);
        const float omega = 2.0f * M_PIf * ((float)filterFreq) / sampleRate;
        const float sn = sin_approxCF(omega);
        const float cs = cos_approxCF(omega);
        const float alpha = sn / (2 * Q);

        float b0, b1, b2;
        switch (filterType)
        {
        case FILTER_LPF:
            b0 = (1 - cs) / 2;
            b1 = 1 - cs;
            b2 = (1 - cs) / 2;
            break;
        case FILTER_NOTCH:
            b0 = 1;
            b1 = -2 * cs;
            b2 = 1;
            break;
        default:
            biquadFilterSetupPassthrough(filter);
            return;
        }
        const float a0 = 1 + alpha;
        const float a1 = -2 * cs;
        const float a2 = 1 - alpha;

        // precompute the coefficients
        filter->b0 = b0 / a0;
        filter->b1 = b1 / a0;
        filter->b2 = b2 / a0;
        filter->a1 = a1 / a0;
        filter->a2 = a2 / a0;
    }
    else
    {
        biquadFilterSetupPassthrough(filter);
    }

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
};
inline float filterGetNotchQ(float centerFrequencyHz, float cutoffFrequencyHz)
{
    return centerFrequencyHz * cutoffFrequencyHz / (centerFrequencyHz * centerFrequencyHz - cutoffFrequencyHz * cutoffFrequencyHz);
};
inline void biquadFilterInitNotch(biquadFilter_t *filter, uint32_t samplingIntervalUs, uint16_t filterFreq, uint16_t cutoffHz)
{
    float Q = filterGetNotchQ(filterFreq, cutoffHz);
    biquadFilterInit(filter, filterFreq, samplingIntervalUs, Q, FILTER_NOTCH);
};
inline void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t filterFreq, uint32_t samplingIntervalUs)
{
    biquadFilterInit(filter, filterFreq, samplingIntervalUs, BIQUAD_Q, FILTER_LPF);
};

inline float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->x1;
    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;
    return result;
};
inline float biquadFilterReset(biquadFilter_t *filter, float value)
{
    filter->x1 = value - (value * filter->b0);
    filter->x2 = (filter->b2 - filter->a2) * value;
    return value;
};
inline float biquadFilterApplyDF1(biquadFilter_t *filter, float input)
{
    /* compute result */
    const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    /* shift x1 to x2, input to x1 */
    filter->x2 = filter->x1;
    filter->x1 = input;

    /* shift y1 to y2, result to y1 */
    filter->y2 = filter->y1;
    filter->y1 = result;

    return result;
};

inline void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType)
{
    // backup state
    float x1 = filter->x1;
    float x2 = filter->x2;
    float y1 = filter->y1;
    float y2 = filter->y2;

    biquadFilterInit(filter, filterFreq, refreshRate, Q, filterType);

    // restore state
    filter->x1 = x1;
    filter->x2 = x2;
    filter->y1 = y1;
    filter->y2 = y2;
};

#ifdef USE_ALPHA_BETA_GAMMA_FILTER
inline void alphaBetaGammaFilterInit(alphaBetaGammaFilter_t *filter, float alpha, float boostGain, float halfLife, float dT)
{
    // beta, gamma, and eta gains all derived from
    // http://yadda.icm.edu.pl/yadda/element/bwmeta1.element.baztech-922ff6cb-e991-417f-93f0-77448f1ef4ec/c/A_Study_Jeong_1_2017.pdf

    const float xi = powf(-alpha + 1.0f, 0.25); // fourth rool of -a + 1
    filter->xk = 0.0f;
    filter->vk = 0.0f;
    filter->ak = 0.0f;
    filter->jk = 0.0f;
    filter->a = alpha;
    filter->b = (1.0f / 6.0f) * powf(1.0f - xi, 2) * (11.0f + 14.0f * xi + 11 * xi * xi);
    filter->g = 2 * powf(1.0f - xi, 3) * (1 + xi);
    filter->e = (1.0f / 6.0f) * powf(1 - xi, 4);
    filter->dT = dT;
    filter->dT2 = dT * dT;
    filter->dT3 = dT * dT * dT;
    pt1FilterInit(&filter->boostFilter, 100, dT);

    const float boost = boostGain * 100;

    filter->boost = (boost * boost / 10000) * 0.003;
    filter->halfLife = halfLife != 0 ? powf(0.5f, dT / halfLife) : 1.0f;
}

inline float alphaBetaGammaFilterApply(alphaBetaGammaFilter_t *filter, float input)
{
    //xk - current system state (ie: position)
    //vk - derivative of system state (ie: velocity)
    //ak - derivative of system velociy (ie: acceleration)
    //jk - derivative of system acceleration (ie: jerk)
    float rk; // residual error

    // give the filter limited history
    filter->xk *= filter->halfLife;
    filter->vk *= filter->halfLife;
    filter->ak *= filter->halfLife;
    filter->jk *= filter->halfLife;

    // update our (estimated) state 'x' from the system (ie pos = pos + vel (last).dT)
    filter->xk += filter->dT * filter->vk + (1.0f / 2.0f) * filter->dT2 * filter->ak + (1.0f / 6.0f) * filter->dT3 * filter->jk;

    // update (estimated) velocity (also estimated dterm from measurement)
    filter->vk += filter->dT * filter->ak + 0.5f * filter->dT2 * filter->jk;
    filter->ak += filter->dT * filter->jk;

    // what is our residual error (measured - estimated)
    rk = input - filter->xk;

    // artificially boost the error to increase the response of the filter
    rk += pt1FilterApply(&filter->boostFilter, fabsf(rk) * rk * filter->boost);
    if ((fabsf(rk * filter->a) > fabsf(input - filter->xk)))
    {
        rk = (input - filter->xk) / filter->a;
    }
    filter->rk = rk; // for logging

    // update our estimates given the residual error.
    filter->xk += filter->a * rk;
    filter->vk += filter->b / filter->dT * rk;
    filter->ak += filter->g / (2.0f * filter->dT2) * rk;
    filter->jk += filter->e / (6.0f * filter->dT3) * rk;

    return filter->xk;
}
#endif