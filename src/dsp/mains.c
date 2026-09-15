#include "mains.h"

#include <math.h>
#include <string.h>

#define TWO_PI 6.283185307179586476925

#define LOWPASS_HZ   1.5  /* the baseband low-pass corner */
#define BASEBAND_HZ  10u  /* baseband values a second */
#define FINE_LAG     10u  /* 1 s, in baseband values */
#define COARSE_LAG   2u   /* 0.2 s */
#define WINDOW       40u  /* lagged products per answer: 4 s */
#define LOCK         0.6  /* coherence across the fine lag to believe it */
#define SEARCH_HZ    3.0  /* how far from nominal an answer may be */
#define AGREE_HZ     0.05 /* two answers this close are the same one */
#define REAIM_HZ     0.005
#define RENORMALISE  8192u

_Static_assert(FINE_LAG == DSP_MAINS_RING, "the ring holds the fine lag");

static void aim(dsp_mains_t *t, double hz)
{
	const double w = TWO_PI * hz / (double)t->fs_hz;

	t->mix_hz = (float)hz;
	t->cw = (float)cos(w);
	t->sw = (float)sin(w);
}

bool dsp_mains_init(dsp_mains_t *t, float fs_hz, float nominal_hz,
		    float start_hz)
{
	if (t == NULL || !(fs_hz >= 250.0f) || !(fs_hz <= 64000.0f) ||
	    !(nominal_hz > 0.0f)) {
		return false;
	}

	const uint32_t sps = (uint32_t)fs_hz;

	if ((float)sps != fs_hz || sps % BASEBAND_HZ != 0u ||
	    (double)nominal_hz + SEARCH_HZ >= (double)fs_hz / 2.0) {
		return false;
	}

	memset(t, 0, sizeof(*t));
	t->fs_hz = fs_hz;
	t->nominal_hz = nominal_hz;
	t->k = (float)(1.0 - exp(-TWO_PI * LOWPASS_HZ / (double)fs_hz));
	t->decim = (uint16_t)(sps / BASEBAND_HZ);
	t->c = 1.0f;

	const double from = (double)start_hz;
	const bool near = start_hz > 0.0f &&
			  fabs(from - (double)nominal_hz) <= SEARCH_HZ;

	aim(t, near ? from : (double)nominal_hz);
	return true;
}

/* Two answers in a row that agree make an estimate. */
static bool settle(dsp_mains_t *t, float hz)
{
	const bool agreed = t->candidate_hz != 0.0f &&
			    fabs((double)hz - (double)t->candidate_hz) < AGREE_HZ;

	if (agreed) {
		t->estimate_hz = hz;
	}
	t->candidate_hz = hz;
	return agreed;
}

/* Every tenth of a second: the low-passed pair as one baseband value. */
static bool baseband(dsp_mains_t *t, float zi, float zq)
{
	if (t->ring_len == FINE_LAG) {
		/*
		 * z times the conjugate of the value one lag back. Its angle is
		 * how far the tone turned over the lag; summed over the window,
		 * noise cancels and a tone adds up.
		 */
		const uint8_t lag = (uint8_t)((t->head + FINE_LAG - COARSE_LAG) %
					      FINE_LAG);
		const float fi = t->ring_i[t->head];
		const float fq = t->ring_q[t->head];
		const float ci = t->ring_i[lag];
		const float cq = t->ring_q[lag];

		t->fine_i += zi * fi + zq * fq;
		t->fine_q += zq * fi - zi * fq;
		t->coarse_i += zi * ci + zq * cq;
		t->coarse_q += zq * ci - zi * cq;
		t->power += zi * zi + zq * zq;
		t->count++;
	}

	t->ring_i[t->head] = zi;
	t->ring_q[t->head] = zq;
	t->head = (uint8_t)((t->head + 1u) % FINE_LAG);
	if (t->ring_len < FINE_LAG) {
		t->ring_len++;
	}

	if (t->count < WINDOW) {
		return false;
	}

	bool agreed = false;
	const double fine = sqrt((double)t->fine_i * (double)t->fine_i +
				 (double)t->fine_q * (double)t->fine_q);

	if (t->power > 0.0f && fine >= LOCK * (double)t->power) {
		/*
		 * The mixer turns the signal the other way, so a tone above the
		 * mixer turns backwards in baseband. The 0.2 s lag finds the
		 * error unambiguously to +/-2.5 Hz; once it is small, the 1 s lag
		 * measures it five times more finely.
		 */
		double df = -atan2((double)t->coarse_q, (double)t->coarse_i) /
			    (TWO_PI * COARSE_LAG / BASEBAND_HZ);

		if (fabs(df) < 0.3) {
			df = -atan2((double)t->fine_q, (double)t->fine_i) /
			     (TWO_PI * FINE_LAG / BASEBAND_HZ);
		}

		double hz = (double)t->mix_hz + df;
		const double lo = (double)t->nominal_hz - SEARCH_HZ;
		const double hi = (double)t->nominal_hz + SEARCH_HZ;

		hz = (hz < lo) ? lo : (hz > hi) ? hi : hz;
		agreed = settle(t, (float)hz);

		if (fabs(hz - (double)t->mix_hz) > REAIM_HZ) {
			/* The values held were mixed at the old aim. */
			aim(t, hz);
			t->ring_len = 0;
			t->head = 0;
		}
	} else {
		/* Not in a row any more. */
		t->candidate_hz = 0.0f;
	}

	t->fine_i = 0.0f;
	t->fine_q = 0.0f;
	t->coarse_i = 0.0f;
	t->coarse_q = 0.0f;
	t->power = 0.0f;
	t->count = 0;
	return agreed;
}

bool dsp_mains_push(dsp_mains_t *t, float x)
{
	if (t == NULL || t->decim == 0u) {
		return false;
	}

	const float c = t->c * t->cw - t->s * t->sw;
	const float s = t->s * t->cw + t->c * t->sw;

	t->i1 += t->k * (x * c - t->i1);
	t->i2 += t->k * (t->i1 - t->i2);
	t->q1 += t->k * (x * s - t->q1);
	t->q2 += t->k * (t->q1 - t->q2);

	/* Rounding would slowly change the phasor's length; put it back. */
	if (++t->turns == RENORMALISE) {
		const float g = (float)(1.0 / sqrt((double)c * (double)c +
						   (double)s * (double)s));

		t->c = c * g;
		t->s = s * g;
		t->turns = 0;
	} else {
		t->c = c;
		t->s = s;
	}

	t->sum_i += t->i2;
	t->sum_q += t->q2;

	if (++t->phase < t->decim) {
		return false;
	}

	/* The average over the tenth of a second: see mains.h for why. */
	const float zi = t->sum_i / (float)t->decim;
	const float zq = t->sum_q / (float)t->decim;

	t->phase = 0;
	t->sum_i = 0.0f;
	t->sum_q = 0.0f;

	return baseband(t, zi, zq);
}
