"""Guarded edits: a restarted cascade primes on its next sample, on the device too."""
import pathlib

ROOT = pathlib.Path(r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS")

FILES = {
"src/dsp/dsp.h": [
('''	uint8_t count;
	uint8_t channels;
} dsp_cascade_t;
''',
'''	uint8_t count;
	uint8_t channels;
	uint8_t prime_mask; /* channels that prime on their next sample */
} dsp_cascade_t;
'''),

('''/*
 * Load sections and start them from rest. A count of 0, with `sections`
 * allowed to be NULL, leaves a pass-through. Returns false, changing nothing,
 * for too many sections or an invalid one.
 */
''',
'''/*
 * Load sections and start them again, each channel primed on its next
 * sample. A count of 0, with `sections` allowed to be NULL, leaves a
 * pass-through. Returns false, changing nothing, for too many sections or an
 * invalid one.
 */
'''),

('''void dsp_cascade_reset_state(dsp_cascade_t *c);
float dsp_cascade_apply(dsp_cascade_t *c, uint8_t channel, float x);
''',
'''/*
 * Start again: each channel primes on its next sample, as if that sample had
 * always been its input. Started from rest instead, a filter takes its first
 * sample as a step from zero, and after a restart mid-stream that is a
 * transient the size of the signal.
 */
void dsp_cascade_reset_state(dsp_cascade_t *c);
float dsp_cascade_apply(dsp_cascade_t *c, uint8_t channel, float x);
'''),
],

"src/dsp/dsp.c": [
('''void dsp_cascade_reset_state(dsp_cascade_t *c)
{
	if (c != NULL) {
		memset(c->state, 0, sizeof(c->state));
	}
}
''',
'''_Static_assert(DSP_MAX_CHANNELS <= 8, "prime_mask holds one bit per channel");

void dsp_cascade_reset_state(dsp_cascade_t *c)
{
	if (c != NULL) {
		memset(c->state, 0, sizeof(c->state));
		c->prime_mask = (uint8_t)((1u << c->channels) - 1u);
	}
}
'''),

('''	if (c == NULL || channel >= c->channels) {
		return x;
	}

	for (uint8_t i = 0; i < c->count; i++) {
''',
'''	if (c == NULL || channel >= c->channels) {
		return x;
	}

	if ((c->prime_mask & (1u << channel)) != 0u) {
		/*
		 * For a steady input the band-pass integrator holds nothing and
		 * the low-pass one holds the input itself, and a section's output
		 * - the next one's input - is (m0 + m2) times it.
		 */
		float in = x;

		for (uint8_t i = 0; i < c->count; i++) {
			c->state[channel][i].ic1 = 0.0f;
			c->state[channel][i].ic2 = in;
			in = (c->run[i].m0 + c->run[i].m2) * in;
		}
		c->prime_mask &= (uint8_t)~(1u << channel);
	}

	for (uint8_t i = 0; i < c->count; i++) {
'''),
],

"tools/dsp_ref.py": [
('''        self.ic2 = [[zero] * MAX_SECTIONS for _ in range(self.channels)]

    def apply(self, ch: int, x):
        x = F32(x)
        ic1, ic2 = self.ic1[ch], self.ic2[ch]
''',
'''        self.ic2 = [[zero] * MAX_SECTIONS for _ in range(self.channels)]
        self.pending = [True] * self.channels

    def apply(self, ch: int, x):
        x = F32(x)
        ic1, ic2 = self.ic1[ch], self.ic2[ch]
        if self.pending[ch]:
            # Primed on this sample, as dsp_cascade_apply primes.
            v = x
            for i, (_, _, _, m0, _, m2) in enumerate(self._run):
                ic1[i] = F32(0.0)
                ic2[i] = v
                v = (m0 + m2) * v
            self.pending[ch] = False
'''),

('''    assert abs(c.settle_samples() - expect) / expect < 0.05, c.settle_samples()
''',
'''    assert abs(c.settle_samples() - expect) / expect < 0.05, c.settle_samples()

    # A restart primes on its next input: a high-pass fed a constant gives
    # nothing from the very first sample, and a low-pass gives the constant.
    c = Cascade(1)
    c.set([design_highpass(fs, 0.5, q)])
    assert all(abs(float(c.apply(0, 1000.0))) < 1e-3 for _ in range(20))
    c.set([lp])
    assert all(abs(float(c.apply(0, 1000.0)) - 1000.0) < 1e-2 for _ in range(20))
'''),
],

"tools/pipeline_ref.py": [
('''        "reset_at": 448,
    }
''',
'''        "reset_at": 448,
        "restart_post_at": 320,
    }
'''),

('''        if events and i == cfg["car_at"]:
            c.set_car(True, cfg["mask_later"])
''',
'''        if events and i == cfg["restart_post_at"]:
            assert not c.set_stage(STAGE_POST, cfg["post"])
        if events and i == cfg["car_at"]:
            c.set_car(True, cfg["mask_later"])
'''),

('''    A(" * gain 12. At RETUNE_AT the notch pair moves 0.2 Hz keeping its state;")
    A(" * at CAR_AT the average takes in channel 7; at RESET_AT the chain")
    A(" * restarts.")
''',
'''    A(" * gain 12. At RETUNE_AT the notch pair moves 0.2 Hz keeping its state;")
    A(" * at RESTART_POST_AT the low-pass restarts, priming on its next input;")
    A(" * at CAR_AT the average takes in channel 7; at RESET_AT the chain")
    A(" * restarts.")
'''),

('''    A(f"#define GOLDEN_B_RESET_AT      {cfg['reset_at']}")
''',
'''    A(f"#define GOLDEN_B_RESET_AT      {cfg['reset_at']}")
    A(f"#define GOLDEN_B_RESTART_POST_AT {cfg['restart_post_at']}")
'''),
],

"tests/pipeline/src/main.c": [
('''			zassert_true(kept, "a same-size retune restarted");
		}
''',
'''			zassert_true(kept, "a same-size retune restarted");
		}
		if (i == GOLDEN_B_RESTART_POST_AT) {
			bool kept = true;

			zassert_ok(chain_set_stage(&chain, CHAIN_STAGE_POST,
						   golden_b_post,
						   GOLDEN_B_POST_COUNT, false,
						   &kept));
			zassert_false(kept, "a restart kept its state");
		}
'''),
],

"tests/dsp/src/main.c": [
('''ZTEST(dsp, test_settle_samples_are_sane)
''',
'''ZTEST(dsp, test_restart_primes_on_next_input)
{
	/*
	 * A restart mid-stream starts as if its next sample had always been the
	 * input: a high-pass fed a constant gives nothing from the first sample,
	 * and a low-pass gives the constant. From rest, both would ring.
	 */
	dsp_cascade_t c;

	dsp_cascade_init(&c, 1);
	zassert_true(dsp_cascade_set(&c, &golden_hp01, 1), NULL);
	for (int i = 0; i < 32; i++) {
		const float y = dsp_cascade_apply(&c, 0, 1000.0f);

		zassert_within(y, 0.0f, 1e-3f, "high-pass gave %f at %d",
			       (double)y, i);
	}

	zassert_true(dsp_cascade_set(&c, &golden_lp40, 1), NULL);
	for (int i = 0; i < 32; i++) {
		const float y = dsp_cascade_apply(&c, 0, 1000.0f);

		zassert_within(y, 1000.0f, 1e-2f, "low-pass gave %f at %d",
			       (double)y, i);
	}
}

ZTEST(dsp, test_settle_samples_are_sane)
'''),
],
}


def main():
    # All or nothing: every edit is checked before any file is written.
    done = []
    for rel, edits in FILES.items():
        p = ROOT / rel
        s = p.read_bytes().decode("utf-8")
        assert "\r\n" not in s, f"{rel} has CRLF line endings"
        for i, (old, new) in enumerate(edits, 1):
            n = s.count(old)
            assert n == 1, f"{rel} edit {i}: found {n} times"
            s = s.replace(old, new)
        done.append((rel, p, s, len(edits)))
    for rel, p, s, n in done:
        p.write_bytes(s.encode("utf-8"))
        print(f"{rel}: {n} edits")


if __name__ == "__main__":
    main()
