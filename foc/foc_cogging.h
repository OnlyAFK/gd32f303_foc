#ifndef FOC_COGGING_H
#define FOC_COGGING_H

#include <stdint.h>

/* Cogging-torque compensation — a position-indexed feed-forward torque-current
 * map, learned by an offline calibration sweep and added on top of the loop's
 * iq command at run time. The scheme descends from the dgm reference firmware
 * (Firmware_app, Source/anticogging.c), adapted to this project's cascade and
 * storage, and extended to a two-pass iterative sweep.
 *
 * Like foc_ident/foc_ltu, the calibration is a commissioning-only sub-state
 * machine that runs inside the FOC control ISR (state FOC_STATE_COG) and
 * returns to IDLE when finished; foc_core.h includes this header to compose
 * cog_ctx_t into motor_t. The map itself is a module-private static in
 * foc_cogging.c (kept out of motor_t so it stays lean).
 *
 * TABLE GEOMETRY. This motor has 24 slots / 28 poles -> LCM = 168 cogging
 * periods per mechanical rev. The table is 4096 points/rev (~24 points per
 * cogging period, matching the reference's density) of int16 fixed point at
 * 5000 counts/A (0.2 mA resolution, +/-6.55 A range) — 8 KB, stored in its
 * own flash pages with a CRC (see app/params), exactly like the reference.
 *
 * HOW IT IS MEASURED. The position->speed->current cascade marches the target
 * angle slowly and continuously (1.5 rpm, 40 s/rev — the same 1-5 rpm region
 * industrial drives use: fast enough to leave the worst of the Stribeck
 * stick-slip regime, slow enough that the ~4.2 Hz cogging disturbance stays
 * below the ~10 Hz speed-loop bandwidth, so the held torque command equals
 * the true cogging torque). The sampled quantity is the SPEED-LOOP PI OUTPUT
 * (target.iq), not measured current: it is the loop's disturbance estimate
 * (what ODrive samples), far cleaner than 20 kHz measured iq, and — crucial
 * for pass 2 — it collapses to the uncompensated RESIDUAL once feed-forward
 * carries the cogging, which measured iq cannot see. Samples are binned by
 * the ACTUAL mech_angle (never the commanded one), each bin keeps its freshest
 * (most settled) value, a velocity gate drops samples taken during
 * break-loose lurches (>2x march rate, or moving backwards), and a LEASH
 * pauses the march (and the phase clock) whenever stiction parks the rotor
 * more than ~30 bins behind the target — the speed-loop integrator then winds
 * up against a fixed error until the rotor breaks loose, so the catch-up
 * chase is always short and the map is never smeared by a long capped-speed
 * pursuit.
 *
 * TWO PASSES (iterative learning). Pass 1: CW warm-up, CW sampling rev, CCW
 * warm-up, CCW sampling rev — CW writes bins, CCW averages into them, so
 * direction-symmetric friction cancels. COG_MID removes the DC component
 * chunk-wise and enables the rough map as LIVE feed-forward. Pass 2 repeats
 * both directions with the detents pre-cancelled (the rotor sticks far less)
 * and adds half of each direction's residual into the map in place. A
 * direction-signed DRAG bias current is added to the speed-loop output for
 * the whole sweep so gearbox stiction cannot stall the march on a weak loop
 * (`cog cal <rpm> <drag_A>`; it cancels in the CW/CCW average like friction).
 * Total ~3-4 min. A `stop` during pass 2 keeps the valid pass-1 map in RAM.
 *
 * The TLE5012B is an *absolute* encoder on the MOTOR rotor (input side of the
 * 1:8 planetary gearbox) and cogging is a fixed function of the rotor-vs-
 * stator mechanical angle, so the map is indexed by mech_angle directly
 * (independent of the electrical zero_offset) and stays valid across power
 * cycles. Gearbox-originated ripple whose period spans one OUTPUT rev (8
 * motor revs) is NOT captured by this table. Re-run `cog cal` after any
 * mechanical change; re-running the encoder ALIGNMENT does not invalidate it,
 * but the LTU table must be final first (it shifts the raw->mech mapping
 * slightly).
 */

#define FOC_COG_TABLE_SIZE   4096u   /* points per mechanical rev (power of two) */
#define FOC_COG_SCALE        5000.0f /* int16 counts per amp (0.2 mA/LSB) */

/* See foc_ident.h: guarded to avoid a duplicate typedef when foc_core.h
 * includes both headers. */
#ifndef FOC_MOTOR_T_DEFINED
#define FOC_MOTOR_T_DEFINED
typedef struct motor_s motor_t;   /* full definition lives in foc_core.h */
#endif

/* Calibration phases (only meaningful in FOC_STATE_COG). Warm-up phases march
 * without sampling (get the rotor moving, pass the static-friction region);
 * sampling phases record the loop output bin by bin. Pass 1 builds the rough
 * map (no feed-forward), COG_MID finalises it chunk-wise and enables it, pass
 * 2 iterates on the residual with the map live. The numeric order is the
 * phase-machine order — do not renumber. */
typedef enum {
  COG_WARMUP_CW = 0,  /* pass 1: march forward ~100 bins, no sampling */
  COG_CW1,            /* pass 1: march forward one rev, record per bin */
  COG_WARMUP_CCW,     /* pass 1: march back ~100 bins, no sampling (remove backlash) */
  COG_CCW1,           /* pass 1: march back one rev, average into the same bins */
  COG_MID,            /* finalise pass-1 map chunk-wise, enable it as feed-forward */
  COG_WARMUP_CW2,     /* pass 2: march forward ~100 bins, no sampling */
  COG_CW2,            /* pass 2: march forward one rev, add residual/2 per bin */
  COG_WARMUP_CCW2,    /* pass 2: march back ~100 bins, no sampling */
  COG_CCW2,           /* pass 2: march back one rev, add residual/2 per bin */
  COG_DONE,           /* hand back to IDLE; the map is finalised in the main loop */
} cog_phase_t;

typedef struct {
  cog_phase_t phase;
  uint8_t  enabled;     /* run-time feed-forward on/off (only if valid) */
  uint8_t  valid;       /* map populated (freshly calibrated or loaded) */
  uint8_t  have_pend;   /* a sample is pending commit for last_bin */
  uint8_t  mid_step;    /* COG_MID sub-step: 0 = accumulate mean, 1 = remove DC */
  uint16_t idx;         /* bins marched (or bins processed in COG_MID) this phase */
  uint16_t covered;     /* bins written by the last finished sweep (info) */
  uint16_t last_bin;    /* bin the rotor is currently inside (sampling phases) */
  float    creep_rpm;   /* rotor speed cap during the march [rpm] (`cog cal <rpm>`) */
  float    max_ff;      /* feed-forward clamp [A] */
  float    drag_a;      /* march drag feed-forward amplitude [A] (`cog cal <rpm> <drag_A>`) */
  float    drag_ff;     /* live direction-signed drag current added to target.iq [A] */
  float    target;      /* running commanded position (marches at a fixed rate) */
  float    saved_max_rpm;    /* pos controller max_rpm to restore when done */
  float    saved_landing_kp; /* pos controller landing_kp to restore when done */
  float    saved_deadband;   /* pos controller deadband to restore when done */
  float    pend_iq;     /* freshest filtered loop output inside the current bin */
  float    peak;        /* |map| peak after compute [A] (info) */
  float    mid_mean;    /* COG_MID: pass-1 DC component being accumulated/removed */
  uint32_t mid_count;   /* COG_MID: covered bins seen while accumulating */
  uint32_t tick;        /* ticks marched within the current bin */
} cog_ctx_t;

/* Start the calibration. creep_rpm > 0 overrides the rotor speed cap [rpm]
 * for this run (else the default); max_ff <= 0 selects the default
 * feed-forward clamp; drag_a <= 0 selects the default drag feed-forward — a
 * direction-signed bias current added to the speed-loop output for the whole
 * sweep so gearbox stiction does not stall the march on a weak loop (raise it
 * if the rotor cannot follow; it cancels in the CW/CCW average and never
 * enters the map). Requires valid current offsets and an aligned encoder (the
 * caller checks the latter). Clears the map + coverage, then enters
 * FOC_STATE_COG. */
void foc_start_cogging(motor_t *motor, float creep_rpm, float max_ff, float drag_a);

/* Per-tick hook called from foc_control_loop BEFORE the position loop while
 * in FOC_STATE_COG: marches the position target at a constant slow rate,
 * filters the speed-loop output into the bin of the actual mech_angle
 * (committing each bin's freshest sample when the rotor leaves it, dropping
 * lurch-window samples), runs the COG_MID pass boundary chunk-wise, advances
 * the phase machine, and hands back to IDLE when both passes are done. */
void foc_cogging_run(motor_t *motor);

/* Finalise the map: remove the DC component over the covered bins (cogging is
 * zero-mean over a rev; the offset is friction that must NOT be fed forward),
 * clamp to max_ff, leave uncovered bins at zero. Runs in the main loop on the
 * COG->IDLE edge (too heavy for the ISR). Returns 1 on success (good
 * coverage), 0 if the sweep failed. */
uint8_t foc_cogging_finish(motor_t *motor);

/* Run-time feed-forward lookup: position-indexed torque current [A], nearest
 * bin (4096 points/rev needs no interpolation). Returns 0 unless the map is
 * valid AND enabled. Cheap enough for the 20 kHz ISR. Also live during the
 * pass-2 calibration revs (enabled by COG_MID) — that is what makes the
 * second pass an iteration on the residual. */
float foc_cogging_lookup(const motor_t *motor);

/* Run-time controls. */
void    foc_cogging_set_enable(motor_t *motor, uint8_t on);
void    foc_cogging_clear(motor_t *motor);
uint8_t foc_cogging_is_valid(const motor_t *motor);
uint8_t foc_cogging_is_enabled(const motor_t *motor);
float   foc_cogging_peak(const motor_t *motor);

/* Bins written by the last finished sweep (0..FOC_COG_TABLE_SIZE) — the
 * coverage figure foc_cogging_finish() checks against the 3/4 minimum. */
uint16_t foc_cogging_coverage(const motor_t *motor);

/* Persistence seam (used by app/params, which owns the flash layout + CRC).
 *  - foc_cogging_table()      returns the live int16 map (FOC_COG_TABLE_SIZE
 *                             entries) for saving, NULL if the map is invalid.
 *  - foc_cogging_load_table() copies a loaded map in and sets valid/enabled. */
const int16_t *foc_cogging_table(const motor_t *motor);
void foc_cogging_load_table(motor_t *motor, const int16_t *table, uint8_t enabled);

#endif /* FOC_COGGING_H */
