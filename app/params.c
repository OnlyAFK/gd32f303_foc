#include "params.h"

#include <string.h>
#include <stddef.h>
#include <fal.h>
#include <flashdb.h>
#include "gd32f30x.h"
#include "motor_hw.h"
#include "version.h"

/* ===================================================================
 * FlashDB KVDB instance + concurrency lock.
 *
 * Flash program/erase stalls the bus (code can't be fetched from the same
 * flash mid-operation) and we only ever write in IDLE, so the lock simply
 * masks interrupts for the duration of each KV operation — this also keeps the
 * 20 kHz control ISR from firing into stalled flash. Acceptable because saves
 * happen only with the motor stopped.
 * =================================================================== */
static struct fdb_kvdb s_kvdb = { 0 };
static bool s_mounted = false;

static void kv_lock(fdb_db_t db)   { (void)db; __disable_irq(); }
static void kv_unlock(fdb_db_t db) { (void)db; __enable_irq(); }

/* Live set, the factory defaults captured from foc_init() at params_init, and a
 * snapshot of what is currently in flash (absent key == its default). s_saved
 * lets params_save() write only the keys that actually changed: FlashDB's
 * set_kv always does a delete+append (it never skips an unchanged value), so
 * writing all keys every save bloats the log and forces frequent sector GC. */
static params_t s_params;
static params_t s_defaults;
static params_t s_saved;
static volatile bool s_dirty = false;

/* Persistent fault log: a ring of the most recent fault snapshots, kept as a
 * single KV blob ("fault_log"). `count` is the total ever pushed; the newest
 * entry is at (count-1) % FAULT_LOG_N. Written immediately on a fault-latch
 * edge (outputs already off), independent of the s_dirty/IDLE path. */
#define FAULT_LOG_N  4u
typedef struct {
  uint32_t       count;
  fault_status_t entry[FAULT_LOG_N];
} fault_log_t;
static fault_log_t s_flog;

/* ===================================================================
 * KV field table: maps a KVDB key to a field in params_t. Adding a new
 * persistent parameter = add a struct field (params.h) + one row here. Keys
 * absent in flash keep their seeded default; new firmware fields therefore
 * default cleanly on an old database without a migration step.
 * =================================================================== */
typedef enum { PT_F32, PT_I32, PT_U8, PT_U16 } ptype_t;

typedef struct {
  const char *key;   /* doubles as the shell name for get/set */
  uint16_t    off;
  ptype_t     type;
} kv_field_t;

static uint16_t field_size(ptype_t t) {
  switch (t) {
  case PT_U8:  return 1u;
  case PT_U16: return 2u;
  default:     return 4u;   /* PT_F32 / PT_I32 */
  }
}

#define KV_F(name, member, ty) { name, (uint16_t)offsetof(params_t, member), ty }

static const kv_field_t s_fields[] = {
  KV_F("cali_ok",   cali_valid,         PT_U8),
  KV_F("enc_zero",  enc_zero_offset,    PT_F32),
  KV_F("enc_dir",   enc_dir,            PT_I32),
  KV_F("id_kp",     id_kp,              PT_F32),
  KV_F("id_ki",     id_ki,              PT_F32),
  KV_F("iq_kp",     iq_kp,              PT_F32),
  KV_F("iq_ki",     iq_ki,              PT_F32),
  KV_F("cur_lim",   cur_limit,          PT_F32),
  KV_F("spd_kp",    spd_kp,             PT_F32),
  KV_F("spd_ki",    spd_ki,             PT_F32),
  KV_F("spd_lim",   spd_limit,          PT_F32),
  KV_F("spd_acc",   spd_accel,          PT_F32),
  KV_F("regen_mrg", regen_margin,       PT_F32),
  KV_F("pos_rpm",   pos_max_rpm,        PT_F32),
  KV_F("pos_acc",   pos_max_accel,      PT_F32),
  KV_F("pos_jerk",  pos_max_jerk,       PT_F32),
  KV_F("pos_kp",    pos_landing_kp,     PT_F32),
  KV_F("pos_db",    pos_deadband,       PT_F32),
  KV_F("rs",        rs,                 PT_F32),
  KV_F("ld",        ld,                 PT_F32),
  KV_F("lq",        lq,                 PT_F32),
  KV_F("flux",      flux,               PT_F32),
  KV_F("ident_ok",  ident_valid,        PT_U8),
  KV_F("inertia",   inertia_j,          PT_F32),
  KV_F("visc_b",    visc_b,             PT_F32),
  KV_F("fric_tc",   friction_tc,        PT_F32),
  KV_F("mech_ok",   mech_valid,         PT_U8),
  KV_F("thr_en",    thr.thr_protect_en, PT_U8),
  KV_F("thr_ovv",   thr.vbus_ov_trip,   PT_F32),
  KV_F("thr_uvv",   thr.vbus_uv_trip,   PT_F32),
  KV_F("thr_ocp",   thr.oc_sw_peak,     PT_F32),
  KV_F("thr_icont", thr.i_cont,         PT_F32),
  KV_F("thr_i2t",   thr.i2t_budget,     PT_F32),
  KV_F("thr_stiq",  thr.stall_iq,       PT_F32),
  KV_F("thr_strpm", thr.stall_rpm,      PT_F32),
  KV_F("thr_ospd",  thr.overspeed_rpm,  PT_F32),
  KV_F("can_id",    can_id,             PT_U8),
  KV_F("can_wdog",  can_wdog_ms,        PT_U16),
  KV_F("mit_kt",    mit_kt,             PT_F32),
  KV_F("mit_clim",  mit_cur_lim,        PT_F32),
};
#define KV_FIELD_COUNT (sizeof(s_fields) / sizeof(s_fields[0]))

/* Read/write one field as a float (the shell's common currency). */
static float field_read_f(const kv_field_t *f) {
  const uint8_t *p = (const uint8_t *)&s_params + f->off;
  switch (f->type) {
  case PT_I32: return (float)(*(const int32_t *)p);
  case PT_U8:  return (float)(*(const uint8_t *)p);
  case PT_U16: return (float)(*(const uint16_t *)p);
  default:     return *(const float *)p;
  }
}

static void field_write_f(const kv_field_t *f, float v) {
  uint8_t *p = (uint8_t *)&s_params + f->off;
  switch (f->type) {
  case PT_I32: *(int32_t *)p = (int32_t)v; break;
  case PT_U8:  *(uint8_t *)p = (uint8_t)v; break;
  case PT_U16: *(uint16_t *)p = (uint16_t)v; break;
  default:     *(float *)p = v;            break;
  }
}

/* Load every field present in flash over the current s_params (which must
 * already hold the defaults). Absent keys (get returns len 0) keep their
 * default. */
static void load_fields_from_flash(void) {
  struct fdb_blob blob;
  uint32_t i;
  for (i = 0; i < KV_FIELD_COUNT; i++) {
    uint8_t *dst = (uint8_t *)&s_params + s_fields[i].off;
    fdb_kv_get_blob(&s_kvdb, s_fields[i].key,
                    fdb_blob_make(&blob, dst, field_size(s_fields[i].type)));
  }
}

static const kv_field_t *field_find(const char *name) {
  uint32_t i;
  for (i = 0; i < KV_FIELD_COUNT; i++) {
    if (strcmp(name, s_fields[i].key) == 0) {
      return &s_fields[i];
    }
  }
  return NULL;
}

static void save_fields(bool force);          /* defined below; used by params_init */
static void load_ltu_from_flash(motor_t *m);  /* defined below; used by params_init */

/* ===================================================================
 * Stored-version guard.
 *
 * The stored parameters are tagged with the firmware version (app/version.h)
 * that wrote them. On boot, if the stored MAJOR.MINOR differs from the current
 * one the layout is treated as incompatible and the params are reset to
 * defaults; PATCH differences are ignored (params kept). See version.h for the
 * bump policy. A normal Keil download does not erase the KVDB region, so params
 * survive firmware updates unless MAJOR.MINOR changes.
 * =================================================================== */
#define KEY_FW_VER  "fw_ver"

static uint32_t read_fw_ver(void) {
  uint32_t v = 0u;  /* 0 == key absent (fresh DB, or pre-versioning layout) */
  struct fdb_blob blob;
  fdb_kv_get_blob(&s_kvdb, KEY_FW_VER, fdb_blob_make(&blob, &v, sizeof(v)));
  return v;
}

static void write_fw_ver(uint32_t v) {
  struct fdb_blob blob;
  fdb_kv_set_blob(&s_kvdb, KEY_FW_VER, fdb_blob_make(&blob, &v, sizeof(v)));
}

/* ---- mapping between params_t and the live motor ---- */

/* Capture the motor's freshly-initialised (DEFAULT_*) values as the parameter
 * set. Used both to seed the live set and to record the factory defaults. */
static void params_from_motor(params_t *p, const motor_t *m) {
  p->enc_zero_offset = m->sensor.encoder.zero_offset;
  p->enc_dir         = m->sensor.encoder.direction;
  p->id_kp           = m->ctrl.id.kp;
  p->id_ki           = m->ctrl.id.ki;
  p->iq_kp           = m->ctrl.iq.kp;
  p->iq_ki           = m->ctrl.iq.ki;
  p->cur_limit       = m->ctrl.id.out_max;
  p->spd_kp          = m->ctrl.speed.kp;
  p->spd_ki          = m->ctrl.speed.ki;
  p->spd_limit       = m->ctrl.speed.out_max;
  p->spd_accel       = m->spd_accel;
  p->regen_margin    = m->regen_margin;
  p->mit_kt          = m->mit_kt;
  p->mit_cur_lim     = m->mit_cur_lim;
  p->pos_max_rpm     = m->ctrl.pos.max_rpm;
  p->pos_max_accel   = m->ctrl.pos.max_accel;
  p->pos_max_jerk    = m->ctrl.pos.max_jerk;
  p->pos_landing_kp  = m->ctrl.pos.landing_kp;
  p->pos_deadband    = m->ctrl.pos.deadband;
}

/* Push the parameter set into the motor through the existing foc setters (no
 * loop-state reset) plus the direct encoder / profile fields. */
static void params_to_motor(const params_t *p, motor_t *m) {
  m->sensor.encoder.zero_offset = p->enc_zero_offset;
  m->sensor.encoder.direction   = (int8_t)p->enc_dir;

  foc_set_pid_gains(m, FOC_GAIN_ID,    p->id_kp,  p->id_ki);
  foc_set_pid_gains(m, FOC_GAIN_IQ,    p->iq_kp,  p->iq_ki);
  foc_set_pid_gains(m, FOC_GAIN_SPEED, p->spd_kp, p->spd_ki);
  foc_set_limit(m, FOC_LIMIT_CURRENT,  p->cur_limit);
  foc_set_limit(m, FOC_LIMIT_SPEED,    p->spd_limit);

  m->spd_accel     = p->spd_accel;
  m->regen_margin  = p->regen_margin;

  /* MIT torque constant: use the user-set value, else derive from the
   * identified flux (Kt = 1.5 * pole_pairs * flux). foc_set_state re-derives
   * this on every MIT entry too, so the value here mainly matters for a
   * `set mit_kt X` while the motor is already in MIT mode. */
  m->mit_kt = p->mit_kt;
  m->mit_cur_lim = p->mit_cur_lim;
  if (p->mit_kt > 0.0f) {
    m->mit_effective_kt = p->mit_kt;
  } else if (p->flux > 0.0f) {
    m->mit_effective_kt = 1.5f * (float)HW_MOTOR_POLE_PAIRS * (p->flux * 1.0e-3f);
  } else {
    m->mit_effective_kt = 0.0f;
  }

  m->ctrl.pos.max_rpm    = p->pos_max_rpm;
  m->ctrl.pos.max_accel  = p->pos_max_accel;
  m->ctrl.pos.max_jerk   = p->pos_max_jerk;
  m->ctrl.pos.landing_kp = p->pos_landing_kp;
  m->ctrl.pos.deadband   = p->pos_deadband;

  /* protection thresholds (fault_set_cfg re-clamps to the hard bounds) */
  fault_set_cfg(&p->thr);
}

/* ---- public API ---- */

void params_init(motor_t *motor) {
  uint32_t stored_ver;

  /* defaults = whatever foc_init() just put in the motor + the fault module's
   * compile-time threshold defaults; no stored calibration */
  params_from_motor(&s_defaults, motor);
  s_defaults.cali_valid  = 0u;
  s_defaults.thr         = *fault_get_cfg();
  s_defaults.can_id      = 1u;   /* node 1 by default (0 is broadcast) */
  s_defaults.can_wdog_ms = 0u;   /* command-loss watchdog off by default */
  s_params = s_defaults;
  s_saved  = s_defaults;   /* an empty/absent flash maps to the defaults */

  if (fal_init() < 0) {
    return;  /* run on RAM defaults, no persistence */
  }
  fdb_kvdb_control(&s_kvdb, FDB_KVDB_CTRL_SET_LOCK,   (void *)kv_lock);
  fdb_kvdb_control(&s_kvdb, FDB_KVDB_CTRL_SET_UNLOCK, (void *)kv_unlock);
  if (fdb_kvdb_init(&s_kvdb, "foc", "fdb_kvdb1", NULL, NULL) != FDB_NO_ERR) {
    return;  /* mount failed: keep defaults */
  }
  s_mounted = true;

  stored_ver = read_fw_ver();
  /* Compatible if the DB is fresh/pre-versioning (0), or its MAJOR.MINOR matches
   * ours. PATCH differences are ignored. */
  bool compatible = (stored_ver == 0u) ||
                    ((stored_ver >> 8) == FW_VER_COMPAT);
  if (compatible) {
    /* load stored values over the defaults; absent key (len 0) keeps default */
    load_fields_from_flash();
    s_saved = s_params;   /* snapshot now reflects what is on flash */
  } else {
    /* incompatible MAJOR.MINOR: discard the stored layout. Keep the RAM
     * defaults and force-overwrite every stale key (auto factory reset). */
    save_fields(true);
  }

  /* Stamp the current version on a fresh / migrated / patch-bumped / just-reset
   * database. No write on a normal boot where the version already matches. */
  if (stored_ver != FW_VER_U32) {
    write_fw_ver(FW_VER_U32);
  }

  /* Load the persistent fault log (a separate blob). On an incompatible layout
   * it is left empty. */
  memset(&s_flog, 0, sizeof(s_flog));
  if (compatible) {
    struct fdb_blob blob;
    fdb_kv_get_blob(&s_kvdb, "fault_log", fdb_blob_make(&blob, &s_flog, sizeof(s_flog)));
    load_ltu_from_flash(motor);    /* restore a stored encoder linearization table */
  }

  params_to_motor(&s_params, motor);
}

params_t *params_get(void) {
  return &s_params;
}

/* Write fields to flash. With force == false, only keys whose value differs
 * from the on-flash snapshot are written (the common path); force == true
 * writes every key (used to overwrite a stale/incompatible schema). */
static void save_fields(bool force) {
  struct fdb_blob blob;
  uint32_t i;

  if (!s_mounted) {
    return;
  }
  for (i = 0; i < KV_FIELD_COUNT; i++) {
    uint16_t sz = field_size(s_fields[i].type);
    uint8_t *cur = (uint8_t *)&s_params + s_fields[i].off;
    uint8_t *old = (uint8_t *)&s_saved  + s_fields[i].off;
    if (force || memcmp(cur, old, sz) != 0) {
      fdb_kv_set_blob(&s_kvdb, s_fields[i].key, fdb_blob_make(&blob, cur, sz));
      memcpy(old, cur, sz);   /* snapshot now matches flash */
    }
  }
}

void params_save(void) {
  save_fields(false);
}

void params_mark_dirty(void) {
  s_dirty = true;
}

bool params_is_dirty(void) {
  return s_dirty;
}

void params_poll_save(motor_t *motor) {
  if (s_dirty && foc_get_state(motor) == FOC_STATE_IDLE) {
    params_save();
    s_dirty = false;
  }
}

void params_capture_calibration(const motor_t *motor) {
  s_params.enc_zero_offset = motor->sensor.encoder.zero_offset;
  s_params.enc_dir         = motor->sensor.encoder.direction;
  s_params.cali_valid      = 1u;
  s_dirty = true;
}

void params_capture_ident(const motor_t *motor) {
  s_params.rs   = motor->ident.rs;
  s_params.ld   = motor->ident.ld;
  s_params.lq   = motor->ident.lq;
  s_params.flux = motor->ident.flux;
  /* Mark the set valid only when the whole electrical sequence completed —
   * flux is measured last, so a non-zero flux implies Rs/Ld/Lq are present too. */
  s_params.ident_valid = (motor->ident.flux > 0.0f) ? 1u : 0u;
  s_dirty = true;
}

void params_capture_identmech(const motor_t *motor) {
  s_params.inertia_j   = motor->imech.j;
  s_params.visc_b      = motor->imech.b;
  s_params.friction_tc = motor->imech.tc;
  s_params.mech_valid  = (motor->imech.j > 0.0f) ? 1u : 0u;
  s_dirty = true;
}

/* Encoder linearization table blob KV. Kept out of the field table (far larger
 * than any scalar) and written immediately like the fault log rather than via
 * the deferred dirty/IDLE path — the callers already guarantee the motor is
 * IDLE. The blob is owned/serialised by foc_ltu (int16 table). */
#define KEY_LTU  "ltu"

void params_capture_ltu(const motor_t *motor) {
  struct fdb_blob blob;
  uint32_t len = 0u;
  const void *p;

  if (!s_mounted) {
    return;
  }
  p = foc_ltu_blob(motor, &len);
  if (p != NULL && len > 0u) {
    fdb_kv_set_blob(&s_kvdb, KEY_LTU, fdb_blob_make(&blob, (void *)p, len));
  } else {
    fdb_kv_del(&s_kvdb, KEY_LTU);   /* invalid table: drop any stale stored copy */
  }
}

/* Load a stored linearization table (if any) straight into foc_ltu's live
 * table. Internal to params_init; absent/short/invalid blob leaves the table
 * disabled. */
static void load_ltu_from_flash(motor_t *motor) {
  struct fdb_blob blob;
  uint32_t maxlen = 0u;
  void *buf;
  size_t got;

  if (!s_mounted) {
    return;
  }
  buf = foc_ltu_load_buffer(&maxlen);
  got = fdb_kv_get_blob(&s_kvdb, KEY_LTU, fdb_blob_make(&blob, buf, maxlen));
  if (got > 0u) {
    foc_ltu_load_commit(motor, (uint32_t)got);
  }
}

void params_tune_current_loop(motor_t *motor, float bw_hz) {
  /* Pole-zero cancellation: Kp = L*w_bw, Ki = R*w_bw (continuous; the PI scales
   * Ki by Ts). Ld/Lq are stored in microhenries. */
  if (s_params.rs <= 0.0f || s_params.ld <= 0.0f || s_params.lq <= 0.0f) {
    return;
  }
  float w_bw = 2.0f * FOC_PI * bw_hz;
  params_set(motor, "id_kp", (s_params.ld * 1.0e-6f) * w_bw);
  params_set(motor, "id_ki", s_params.rs * w_bw);
  params_set(motor, "iq_kp", (s_params.lq * 1.0e-6f) * w_bw);
  params_set(motor, "iq_ki", s_params.rs * w_bw);
}

void params_tune_speed_loop(motor_t *motor, float bw_hz) {
  /* Plant w/iq = Kt/(J*s+B); pole-zero cancellation gives (in A per rpm):
   * Kp = (J*w_bw/Kt)*(2pi/60), Ki = (B*w_bw/Kt)*(2pi/60). J in g.cm^2, B in
   * uNm.s/rad, flux in mWb. */
  if (s_params.mech_valid == 0u || s_params.inertia_j <= 0.0f || s_params.flux <= 0.0f) {
    return;
  }
  float j_si = s_params.inertia_j * 1.0e-7f;
  float b_si = s_params.visc_b * 1.0e-6f;
  float kt   = 1.5f * (float)HW_MOTOR_POLE_PAIRS * (s_params.flux * 1.0e-3f);
  float w_bw = 2.0f * FOC_PI * bw_hz;
  params_set(motor, "spd_kp", (j_si * w_bw / kt) * FOC_RPM_TO_RAD);
  params_set(motor, "spd_ki", (b_si * w_bw / kt) * FOC_RPM_TO_RAD);
}

void params_factory_reset(motor_t *motor) {
  s_params = s_defaults;
  params_to_motor(&s_params, motor);
  params_save();
  s_dirty = false;
}

void params_reload(motor_t *motor) {
  if (!s_mounted) {
    return;
  }
  s_params = s_defaults;          /* so keys absent in flash revert to default */
  load_fields_from_flash();
  params_to_motor(&s_params, motor);
  s_params.thr = *fault_get_cfg();  /* mirror the clamped thresholds back */
  s_saved = s_params;               /* snapshot matches flash (no spurious writes) */
  s_dirty = false;
}

bool params_set(motor_t *motor, const char *name, float value) {
  const kv_field_t *f = field_find(name);
  if (f == NULL) {
    return false;
  }
  field_write_f(f, value);
  /* Apply live: gains/limits via foc setters, thresholds via fault_set_cfg
   * (which clamps). Then mirror the clamped thresholds back so a subsequent
   * get/list shows the value that actually took effect. */
  params_to_motor(&s_params, motor);
  s_params.thr = *fault_get_cfg();
  s_dirty = true;
  return true;
}

bool params_get_value(const char *name, float *out) {
  const kv_field_t *f = field_find(name);
  if (f == NULL) {
    return false;
  }
  *out = field_read_f(f);
  return true;
}

uint32_t params_field_count(void) {
  return KV_FIELD_COUNT;
}

const char *params_field_name(uint32_t i) {
  return (i < KV_FIELD_COUNT) ? s_fields[i].key : NULL;
}

float params_field_value(uint32_t i) {
  return (i < KV_FIELD_COUNT) ? field_read_f(&s_fields[i]) : 0.0f;
}

void params_fault_log_push(const fault_status_t *snap) {
  struct fdb_blob blob;

  s_flog.entry[s_flog.count % FAULT_LOG_N] = *snap;
  s_flog.count++;

  if (s_mounted) {
    /* Immediate write: called on the fault-latch edge with outputs already off,
     * so stalling the (braked) control loop during the flash write is safe. */
    fdb_kv_set_blob(&s_kvdb, "fault_log", fdb_blob_make(&blob, &s_flog, sizeof(s_flog)));
  }
}

uint32_t params_fault_log_depth(void) {
  return (s_flog.count < FAULT_LOG_N) ? s_flog.count : FAULT_LOG_N;
}

const fault_status_t *params_fault_log_entry(uint32_t i) {
  uint32_t depth = params_fault_log_depth();
  if (i >= depth) {
    return NULL;
  }
  /* newest first: i = 0 -> (count-1) % N */
  return &s_flog.entry[(s_flog.count - 1u - i) % FAULT_LOG_N];
}
