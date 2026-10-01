/*
 * AK45SlowPosition.ino
 *
 * Standalone slow position-control test for 4 CubeMars motors
 *   - 3 x AK45-36
 *   - 1 x AK45-10
 * on Teensy 4.1 (CAN1, pins 22 TX / 23 RX) or Teensy 3.6 (CAN0), 1 Mbps.
 *
 * Uses MIT mode, standard (11-bit) frames - same format as _CANMotor in src/Motor.cpp
 * for every AK motor except AK60v3:
 *   enter mode : FF FF FF FF FF FF FF FC
 *   exit  mode : FF FF FF FF FF FF FF FD
 *   set zero   : FF FF FF FF FF FF FF FE
 *   command    : p(16) v(12) kp(12) kd(12) t(12)
 *   reply      : [id] p(16) v(12) i(12) [temp] [err]
 *
 * Why this is robust to unknown AK45 scaling ranges:
 *   Only position (P_MAX = 12.5 rad) and kp/kd (0..500, 0..5) are actually used.
 *   v_des = 0 and t_ff = 0 are always sent, so the V/T ranges only affect the
 *   *displayed* speed/current, not the command.
 *
 * Serial (115200, newline terminated) commands:
 *   e               enable all: enter motor mode, hold CURRENT position (gains ramp up)
 *   d               soft disable: gains ramp down to 0, then exit motor mode
 *   x               immediate stop (exit motor mode now) - joints go limp
 *   g <n> <deg>     move motor n (1-4) to absolute angle [deg]
 *   r <n> <deg>     move motor n by relative angle [deg]
 *   a <d1> <d2> <d3> <d4>  move all motors to absolute angles [deg]
 *   h               move all motors to 0 deg
 *   v <deg/s>       max speed (default 5, capped at MAX_SPEED_CAP_DPS)
 *   z <n>           set current position of motor n as zero (only while disabled)
 *   p               toggle status printing (10 Hz)
 *   ?               help
 *
 * All angles are OUTPUT SHAFT angles (after the internal gearbox).
 */

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

#include <FlexCAN_T4.h>

#if defined(ARDUINO_TEENSY36)
FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can_bus;
#else
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_bus;
#endif

// ======================= USER CONFIG =======================
const int NUM_MOTORS = 4;

struct MotorCfg
{
    const char* name;
    uint8_t can_id;       // driver ID set in the CubeMars upper computer
    float kp;             // [Nm/rad] at output shaft
    float kd;             // [Nm*s/rad]
    float min_deg;        // soft limit
    float max_deg;        // soft limit
    bool flip;            // invert direction
    float v_max;          // protocol speed range [rad/s]  (display only)
    float t_max;          // protocol torque range [Nm]    (display only)
};

// Start with SMALL gains on an unloaded motor, then raise them.
// AK45-36: rated 8 Nm  -> kp 5 Nm/rad gives ~0.44 Nm at 5 deg error
// AK45-10: rated 2.5 Nm -> kp 2 Nm/rad gives ~0.17 Nm at 5 deg error
MotorCfg cfg[NUM_MOTORS] = {
    // name        id  kp    kd    min    max   flip  v_max  t_max
    {"AK45-36 #1",  1, 5.0f, 0.5f, -90.f, 90.f, false, 5.44f, 24.f},
    {"AK45-36 #2",  2, 5.0f, 0.5f, -90.f, 90.f, false, 5.44f, 24.f},
    {"AK45-36 #3",  3, 5.0f, 0.5f, -90.f, 90.f, false, 5.44f, 24.f},
    {"AK45-10",     4, 2.0f, 0.2f, -90.f, 90.f, false, 18.85f, 7.f},
};

const int   MOTOR_POWER_PIN    = -1;     // pin that switches motor power on your board, -1 = not used
const bool  MOTOR_POWER_ON     = HIGH;
const float CONTROL_HZ         = 200.0f;
const float DEFAULT_SPEED_DPS  = 5.0f;   // "very slow"
const float MAX_SPEED_CAP_DPS  = 20.0f;
const float ACCEL_DPS2         = 10.0f;  // smooth start/stop
const float GAIN_RAMP_S        = 1.5f;   // gain ramp time on enable / soft disable
const float MAX_TRACK_ERR_DEG  = 20.0f;  // measured vs setpoint, exceeded -> stop all
const uint32_t REPLY_TIMEOUT_MS = 100;   // no reply while enabled -> stop all
// ===========================================================

const float P_MAX = 12.5f, KP_MAX = 500.f, KD_MAX = 5.f;

enum State { DISABLED, ENABLING, RUNNING, DISABLING };
State state = DISABLED;

struct MotorRt
{
    float set_deg = 0;     // current (rate-limited) setpoint
    float goal_deg = 0;    // target
    float vel_dps = 0;     // setpoint velocity
    float p_deg = 0, v_rads = 0, t_nm = 0;
    int8_t temp = 0;
    uint8_t err = 0;
    uint32_t last_reply_ms = 0;
    bool got_reply = false;
};
MotorRt rt[NUM_MOTORS];

float speed_dps = DEFAULT_SPEED_DPS;
float gain_scale = 0.0f;
uint32_t state_t0 = 0;
bool print_on = true;

// ---------------- packing ----------------
static uint32_t f2u(float x, float lo, float hi, int bits)
{
    x = constrain(x, lo, hi);
    return (uint32_t)((x - lo) * (float)((1UL << bits) - 1) / (hi - lo) + 0.5f);
}
static float u2f(uint32_t x, float lo, float hi, int bits)
{
    return (float)x * (hi - lo) / (float)((1UL << bits) - 1) + lo;
}

static void send_special(uint8_t id, uint8_t last)
{
    CAN_message_t m;
    m.id = id; m.flags.extended = 0; m.len = 8;
    for (int i = 0; i < 7; i++) m.buf[i] = 0xFF;
    m.buf[7] = last;
    can_bus.write(m);
}

static void send_cmd(int i, float p_deg, float kp, float kd)
{
    const MotorCfg& c = cfg[i];
    float dir = c.flip ? -1.f : 1.f;
    float p = dir * p_deg * DEG_TO_RAD;
    uint32_t pi = f2u(p, -P_MAX, P_MAX, 16);
    // v_des = 0, t_ff = 0 -> midpoint code (error < range/4095, negligible)
    uint32_t vi = 2048;
    uint32_t ti = 2048;
    uint32_t kpi = f2u(kp, 0, KP_MAX, 12);
    uint32_t kdi = f2u(kd, 0, KD_MAX, 12);

    CAN_message_t m;
    m.id = c.can_id; m.flags.extended = 0; m.len = 8;
    m.buf[0] = pi >> 8;
    m.buf[1] = pi & 0xFF;
    m.buf[2] = vi >> 4;
    m.buf[3] = ((vi & 0xF) << 4) | (kpi >> 8);
    m.buf[4] = kpi & 0xFF;
    m.buf[5] = kdi >> 4;
    m.buf[6] = ((kdi & 0xF) << 4) | (ti >> 8);
    m.buf[7] = ti & 0xFF;
    can_bus.write(m);
}

static void read_all()
{
    CAN_message_t m;
    while (can_bus.read(m))
    {
        if (m.flags.extended || m.len < 6) continue;
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            if (m.buf[0] != cfg[i].can_id) continue;
            float dir = cfg[i].flip ? -1.f : 1.f;
            uint32_t pi = (m.buf[1] << 8) | m.buf[2];
            uint32_t vi = (m.buf[3] << 4) | (m.buf[4] >> 4);
            uint32_t ii = ((m.buf[4] & 0xF) << 8) | m.buf[5];
            rt[i].p_deg  = dir * u2f(pi, -P_MAX, P_MAX, 16) * RAD_TO_DEG;
            rt[i].v_rads = dir * u2f(vi, -cfg[i].v_max, cfg[i].v_max, 12);
            rt[i].t_nm   = dir * u2f(ii, -cfg[i].t_max, cfg[i].t_max, 12);
            if (m.len >= 8) { rt[i].temp = (int8_t)m.buf[6]; rt[i].err = m.buf[7]; }
            rt[i].last_reply_ms = millis();
            rt[i].got_reply = true;
        }
    }
}

// ---------------- state changes ----------------
static void stop_now(const char* why)
{
    for (int i = 0; i < NUM_MOTORS; i++) send_special(cfg[i].can_id, 0xFD);
    state = DISABLED;
    gain_scale = 0;
    Serial.print("!! STOP: ");
    Serial.println(why);
}

static bool start_enable()
{
    if (state != DISABLED) { Serial.println("already enabled"); return false; }

    for (int i = 0; i < NUM_MOTORS; i++) rt[i].got_reply = false;
    for (int i = 0; i < NUM_MOTORS; i++) { send_special(cfg[i].can_id, 0xFC); delay(2); }

    // Zero-gain frames just to get position replies.
    uint32_t t0 = millis();
    while (millis() - t0 < 300)
    {
        for (int i = 0; i < NUM_MOTORS; i++) send_cmd(i, 0, 0, 0);
        delay(5);
        read_all();
    }

    bool ok = true;
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (!rt[i].got_reply)
        {
            Serial.print("no reply from "); Serial.print(cfg[i].name);
            Serial.print(" (id "); Serial.print(cfg[i].can_id); Serial.println(")");
            ok = false;
        }
        else if (rt[i].p_deg < cfg[i].min_deg || rt[i].p_deg > cfg[i].max_deg)
        {
            Serial.print(cfg[i].name); Serial.print(" is outside soft limits: ");
            Serial.print(rt[i].p_deg); Serial.println(" deg  -> fix zero/limits first");
            ok = false;
        }
    }
    if (!ok) { stop_now("enable aborted"); return false; }

    // Hold where we are - no jump.
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        rt[i].set_deg = rt[i].goal_deg = rt[i].p_deg;
        rt[i].vel_dps = 0;
    }
    gain_scale = 0;
    state = ENABLING;
    state_t0 = millis();
    Serial.println("enabling: holding current position, gains ramping up");
    return true;
}

static void set_goal(int i, float deg)
{
    if (state != RUNNING) { Serial.println("not running (send 'e' first, wait for ramp)"); return; }
    float g = constrain(deg, cfg[i].min_deg, cfg[i].max_deg);
    if (g != deg) { Serial.print("clamped to "); Serial.println(g); }
    rt[i].goal_deg = g;
}

// Rate + accel limited setpoint toward goal.
static void step_setpoint(int i, float dt)
{
    float err = rt[i].goal_deg - rt[i].set_deg;
    float v = rt[i].vel_dps;
    float dir = (err > 0) ? 1.f : -1.f;
    float stop_dist = v * v / (2.f * ACCEL_DPS2);

    float v_target;
    if (fabsf(err) < 0.01f && fabsf(v) < 0.5f) { rt[i].set_deg = rt[i].goal_deg; rt[i].vel_dps = 0; return; }
    if (fabsf(err) <= stop_dist && v * dir > 0) v_target = 0;   // decelerate
    else v_target = dir * speed_dps;

    float dv = constrain(v_target - v, -ACCEL_DPS2 * dt, ACCEL_DPS2 * dt);
    v += dv;
    float step = v * dt;
    if (fabsf(step) > fabsf(err) && step * err > 0) { step = err; v = 0; }  // don't overshoot
    rt[i].set_deg += step;
    rt[i].vel_dps = v;
}

// ---------------- serial ----------------
static void help()
{
    Serial.println("e enable | d soft disable | x STOP | g n deg | r n deg | a d1 d2 d3 d4 | h home");
    Serial.println("v deg/s | z n (zero, disabled only) | p print on/off | ? help");
}

static void handle_line(char* s)
{
    char c = s[0];
    float a[4] = {0, 0, 0, 0};
    int n = 0;
    char* tok = strtok(s + 1, " ,\t");
    while (tok && n < 4) { a[n++] = atof(tok); tok = strtok(NULL, " ,\t"); }

    switch (c)
    {
        case 'e': start_enable(); break;
        case 'd':
            if (state == RUNNING || state == ENABLING) { state = DISABLING; state_t0 = millis(); Serial.println("soft disable..."); }
            break;
        case 'x': stop_now("user"); break;
        case 'g':
        case 'r':
        {
            int i = (int)a[0] - 1;
            if (n < 2 || i < 0 || i >= NUM_MOTORS) { Serial.println("usage: g|r <1-4> <deg>"); break; }
            set_goal(i, (c == 'g') ? a[1] : rt[i].goal_deg + a[1]);
            break;
        }
        case 'a':
            if (n < NUM_MOTORS) { Serial.println("usage: a d1 d2 d3 d4"); break; }
            for (int i = 0; i < NUM_MOTORS; i++) set_goal(i, a[i]);
            break;
        case 'h': for (int i = 0; i < NUM_MOTORS; i++) set_goal(i, 0); break;
        case 'v':
            speed_dps = constrain(a[0], 0.5f, MAX_SPEED_CAP_DPS);
            Serial.print("speed = "); Serial.print(speed_dps); Serial.println(" deg/s");
            break;
        case 'z':
        {
            int i = (int)a[0] - 1;
            if (state != DISABLED) { Serial.println("disable first ('d')"); break; }
            if (n < 1 || i < 0 || i >= NUM_MOTORS) { Serial.println("usage: z <1-4>"); break; }
            send_special(cfg[i].can_id, 0xFE);
            Serial.print("zeroed "); Serial.println(cfg[i].name);
            break;
        }
        case 'p': print_on = !print_on; break;
        case '?': help(); break;
        default: break;
    }
}

static void poll_serial()
{
    static char buf[64];
    static uint8_t len = 0;
    while (Serial.available())
    {
        char ch = Serial.read();
        if (ch == '\r') continue;
        if (ch == '\n') { buf[len] = 0; if (len) handle_line(buf); len = 0; }
        else if (len < sizeof(buf) - 1) buf[len++] = ch;
    }
}

static void print_status()
{
    static const char* names[] = {"OFF", "ENABLING", "RUN", "DISABLING"};
    Serial.print(names[state]);
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        Serial.print(" | M"); Serial.print(i + 1);
        Serial.print(" set "); Serial.print(rt[i].set_deg, 1);
        Serial.print(" meas "); Serial.print(rt[i].p_deg, 1);
        Serial.print(" T "); Serial.print(rt[i].t_nm, 2);
        Serial.print(" tmp "); Serial.print(rt[i].temp);
        if (rt[i].err) { Serial.print(" ERR "); Serial.print(rt[i].err); }
    }
    Serial.println();
}

// ---------------- main ----------------
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    if (MOTOR_POWER_PIN >= 0)
    {
        pinMode(MOTOR_POWER_PIN, OUTPUT);
        digitalWrite(MOTOR_POWER_PIN, MOTOR_POWER_ON);
        delay(1000);   // let the drivers boot
    }

    can_bus.begin();
    can_bus.setBaudRate(1000000);

    Serial.println("AK45 slow position test (MIT mode, standard frame)");
    help();
}

void loop()
{
    poll_serial();
    read_all();

    static uint32_t last_us = micros();
    uint32_t now_us = micros();
    const uint32_t period_us = (uint32_t)(1e6f / CONTROL_HZ);
    if (now_us - last_us < period_us) return;
    float dt = (now_us - last_us) * 1e-6f;
    last_us = now_us;

    if (state == DISABLED) return;

    // gain ramp
    float ramp = constrain((millis() - state_t0) / (GAIN_RAMP_S * 1000.f), 0.f, 1.f);
    if (state == ENABLING)
    {
        gain_scale = ramp;
        if (ramp >= 1.f) { state = RUNNING; Serial.println("running"); }
    }
    else if (state == DISABLING)
    {
        gain_scale = 1.f - ramp;
        if (ramp >= 1.f) { stop_now("soft disable done"); return; }
    }

    // safety checks
    uint32_t now_ms = millis();
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (now_ms - rt[i].last_reply_ms > REPLY_TIMEOUT_MS) { stop_now("reply timeout"); Serial.println(cfg[i].name); return; }
        if (rt[i].err) { stop_now("motor error flag"); Serial.println(cfg[i].name); return; }
        if (state == RUNNING && fabsf(rt[i].p_deg - rt[i].set_deg) > MAX_TRACK_ERR_DEG) { stop_now("tracking error"); Serial.println(cfg[i].name); return; }
    }

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (state == RUNNING) step_setpoint(i, dt);
        send_cmd(i, rt[i].set_deg, cfg[i].kp * gain_scale, cfg[i].kd * gain_scale);
        delayMicroseconds(100);   // spread frames, avoid TX mailbox overflow
    }

    static uint32_t last_print = 0;
    if (print_on && now_ms - last_print >= 100) { last_print = now_ms; print_status(); }
}

#else
void setup() {}
void loop() {}
#endif
