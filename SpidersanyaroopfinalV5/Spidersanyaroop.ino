/*Creted from scratch the ultimate version of 4 leged 12 dof spider robot developed by Sanyaroop */


#include <Servo.h>
#include "FlexiTimer2.h"

/* ── Leg indices ──────────────────────────────────────────── */
#define LEG_FR 0
#define LEG_BR 1
#define LEG_FL 2
#define LEG_BL 3

/* ── Servo pins [leg][joint: 0=hip 1=femur 2=tibia] ──────── */
Servo servo[4][3];
const int SERVO_PIN[4][3] PROGMEM = {
  {2,3,4},{5,6,7},{8,9,10},{11,12,13}
};

/* ── Robot geometry (mm) ─────────────────────────────────── */
#define LEN_A    55.0f
#define LEN_B    77.5f
#define LEN_C    27.5f
#define LEN_SIDE 71.0f

/* ── Motion constants ────────────────────────────────────── */
#define Z_ABS  -28.0f          // lowest safe Z (sit on floor)
#define Z_DEF  -43.0f          // body lower to ground (was -50)
#define Z_UP   -39.0f          // less lift, foot barely clears (was -30)
#define X_DEF   62.0f          // default X reach
#define Y_0      0.0f          // Y origin
#define Y_STEP  26.0f          // shorter stride, less drag on BR (was 40)

#define SPEED_TURN  4.0f
#define SPEED_LEG   8.0f
#define SPEED_BODY  2.0f      // gentler body slide (was 3.0)
#define SPEED_SEAT  1.0f

#define KEEP  255.0f           // sentinel: "don't change this axis"
#define PI_F  3.14159265f

/* ── Shorthand aliases (used inside trick frames only) ───── */
#define _X    x_default         // runtime value of X_DEF (after offset)
#define _Y    Y_0
#define _ZF   z_front           // front leg Z (balance-aware)
#define _ZB   z_back            // back  leg Z (balance-aware)
#define _ZBT  Z_ABS
#define _ZU   Z_UP
#define K     KEEP

/* ── Load-balance parameters (tune these for your robot) ─── */
// FRONT_COMP  : front legs stand TALLER by this many mm → lifts sagging nose
//               YOUR CURRENT PROBLEM → increase this until front is level
//               Start at 10. Send '<'/'>' to nudge live.
// LOAD_COMP   : back  legs stand TALLER by this many mm → levels heavy rear
//               Send '+'/'-' to nudge live.
// REAR_STANCE : back feet placed this many mm FURTHER BACK → stable support
//               Send '('/')' to nudge live.
// FRONT_XTRA  : front feet reach this many mm FURTHER OUT → counter-balance
float FRONT_COMP  = 10.0f;   // mm  ← START HERE for your front-sag problem
float LOAD_COMP   = 10.0f;   // mm
float REAR_STANCE = 15.0f;   // mm
float FRONT_XTRA  =  8.0f;   // mm

// Derived values (recomputed in recalc_balance())
float x_default;    // = X_DEF + FRONT_XTRA   for front legs
float z_front;      // = Z_DEF - FRONT_COMP   (more negative = more extended = body higher)
float z_back;       // = Z_DEF - LOAD_COMP
float y_back_xtra;  // = REAR_STANCE

/* ── Runtime IK state ────────────────────────────────────── */
volatile float site_now[4][3];
volatile float site_expect[4][3];
float temp_speed[4][3];
float move_speed;
float speed_mul = 1.0f;
volatile uint16_t rest_counter;

/* ── Battery (A7, 10k+3.3k divider → 3.3k side to AVR) ──── */
#define BATT_PIN      A7
#define BATT_WARN_MV  6800
#define BATT_FACTOR   19.84f
#define BATT_N        8          // power-of-2 → fast >>3 average
int16_t batt_buf[BATT_N];
uint8_t batt_idx = 0;
int32_t batt_sum = 0;

/* ── Pre-calculated turn geometry ────────────────────────── */
// (computed once in setup after geometry is frozen)
float turn_x0, turn_y0, turn_x1, turn_y1;

/* ── Command table (replaces giant switch) ───────────────── */
void step_forward(); void step_back();
void turn_left();    void turn_right();
void stand();        void sit();
void robot_sleep();  void wake_up();
void pushups_3()    { extern void pushups(int); pushups(3); }
void jump();         void happy_jump();
void handshake_3()  { extern void hand_shake(int); hand_shake(3); }
void dance_10()     { extern void body_dance(int); body_dance(10); }
void body_twist();   void body_roll();
void headshake_3()  { extern void head_shake(int); head_shake(3); }
void scared();       void shiver();
void fighting_mode();void play_dead();
void moonwalk();     void dog_pee();
void angry_stomp();  void high_five();
void bow();          void sneak();
void dizzy();        void happy_wiggle();
void handstand();    void scratch();
void tiptoe();       void propose();
void tap_foot();     void blow_kiss();
void digging();      void salute();
void shy();          void radar_scan();
void caterpillar();  void surrender();
void breathe_5()    { extern void breathing_idle(int); breathing_idle(5); }
void print_battery();void print_help();
void nudge_fc_up()  { FRONT_COMP+=1; recalc_balance(); stand(); Serial.print(F("FC=")); Serial.println(FRONT_COMP); }
void nudge_fc_dn()  { FRONT_COMP=max(0.f,FRONT_COMP-1); recalc_balance(); stand(); Serial.print(F("FC=")); Serial.println(FRONT_COMP); }
void nudge_lc_up()  { LOAD_COMP+=1;  recalc_balance(); stand(); Serial.print(F("LC=")); Serial.println(LOAD_COMP); }
void nudge_lc_dn()  { LOAD_COMP=max(0.f,LOAD_COMP-1); recalc_balance(); stand(); Serial.print(F("LC=")); Serial.println(LOAD_COMP); }
void nudge_rs_up()  { REAR_STANCE+=2; recalc_balance(); stand(); Serial.print(F("RS=")); Serial.println(REAR_STANCE); }
void nudge_rs_dn()  { REAR_STANCE=max(0.f,REAR_STANCE-2); recalc_balance(); stand(); Serial.print(F("RS=")); Serial.println(REAR_STANCE); }
void speed_half()   { speed_mul=0.5f; Serial.println(F("Spd:0.5x")); }
void speed_norm()   { speed_mul=1.0f; Serial.println(F("Spd:1.0x")); }
void speed_fast()   { speed_mul=2.0f; Serial.println(F("Spd:2.0x")); }

struct Cmd { char key; void(*fn)(); };
static const Cmd CMD[] = {
  {'F',step_forward}, {'B',step_back},   {'L',turn_left},   {'R',turn_right},
  {'X',stand},        {'x',sit},         {'W',robot_sleep}, {'w',wake_up},
  {'n',pushups_3},    {'J',jump},        {'q',happy_jump},
  {'V',handshake_3},  {'v',handshake_3}, {'U',dance_10},    {'u',dance_10},
  {'c',body_twist},   {'o',body_roll},   {'y',headshake_3},
  {'b',scared},       {'f',shiver},      {'a',fighting_mode},{'d',play_dead},
  {'m',moonwalk},     {'p',dog_pee},     {'e',angry_stomp},
  {'/',high_five},    {'k',bow},         {'t',sneak},
  {'z',dizzy},        {'h',happy_wiggle},{'j',handstand},
  {'s',scratch},      {'T',tiptoe},      {'P',propose},
  {'I',tap_foot},     {'K',blow_kiss},   {'D',digging},
  {'G',salute},       {'Y',shy},         {'M',radar_scan},
  {'C',caterpillar},  {'Z',surrender},   {'i',breathe_5},
  {'r',print_battery},{'?',print_help},
  {'<',nudge_fc_up},  {'>',nudge_fc_dn},
  {'+',nudge_lc_up},  {'-',nudge_lc_dn},
  {'(',nudge_rs_up},  {')',nudge_rs_dn},
  {'1',speed_half},   {'2',speed_norm},  {'3',speed_fast},
};
static const uint8_t CMD_N = sizeof(CMD)/sizeof(CMD[0]);

/* ════════════════════════════════════════════════════════════
   BALANCE HELPER
   ════════════════════════════════════════════════════════════ */
void recalc_balance() {
  x_default   = X_DEF + FRONT_XTRA;
  z_front     = Z_DEF - FRONT_COMP;   // more negative = front legs more extended = nose lifts
  z_back      = Z_DEF - LOAD_COMP;
  y_back_xtra = REAR_STANCE;
}
// Per-leg balance-aware values
inline float lz(uint8_t leg) { return (leg==LEG_BR||leg==LEG_BL) ? z_back  : z_front; }
inline float lx(uint8_t leg) { return (leg==LEG_FR||leg==LEG_FL) ? x_default : X_DEF; }
inline float ly(uint8_t leg) { return (leg==LEG_BR||leg==LEG_BL) ? y_back_xtra : 0.f; }

/* ════════════════════════════════════════════════════════════
   SETUP
   ════════════════════════════════════════════════════════════ */
void setup()
{
  Serial.begin(115200);
  recalc_balance();

  // Pre-calculate turn geometry
  float ta = sqrt(pow(2*x_default+LEN_SIDE,2)+pow(Y_STEP,2));
  float tb = 2*(Y_0+Y_STEP)+LEN_SIDE;
  float tc = sqrt(pow(2*x_default+LEN_SIDE,2)+pow(2*Y_0+Y_STEP+LEN_SIDE,2));
  float talpha = acos(constrain((ta*ta+tb*tb-tc*tc)/(2*ta*tb),-1.f,1.f));
  turn_x1 = (ta-LEN_SIDE)/2;
  turn_y1 = Y_0 + Y_STEP/2;
  turn_x0 = turn_x1 - tb*cos(talpha);
  turn_y0 = tb*sin(talpha) - turn_y1 - LEN_SIDE;

  pinMode(14, OUTPUT);
  for (uint8_t i=0;i<BATT_N;i++) batt_buf[i]=0;

  // Initial seated positions
  set_site(LEG_FR, lx(LEG_FR), Y_0+Y_STEP,         Z_ABS);
  set_site(LEG_BR, X_DEF,      Y_0+Y_STEP+ly(LEG_BR), Z_ABS);
  set_site(LEG_FL, lx(LEG_FL), Y_0,                 Z_ABS);
  set_site(LEG_BL, X_DEF,      Y_0+ly(LEG_BL),      Z_ABS);
  for(uint8_t i=0;i<4;i++) for(uint8_t j=0;j<3;j++) site_now[i][j]=site_expect[i][j];

  FlexiTimer2::set(20, servo_service);  // 50 Hz ISR
  FlexiTimer2::start();
  servo_attach();

  Serial.println(F("=== Spider Robot v4 Ready ==="));
  Serial.print(F("FC=")); Serial.print(FRONT_COMP); Serial.print(F(" LC=")); Serial.print(LOAD_COMP);
  Serial.print(F(" RS=")); Serial.print(REAR_STANCE);
  Serial.print(F(" zF=")); Serial.print(z_front);
  Serial.print(F(" zB=")); Serial.println(z_back);
  print_help();
}

/* ════════════════════════════════════════════════════════════
   MAIN LOOP
   ════════════════════════════════════════════════════════════ */
void loop()
{
  // ── Rolling battery average ──────────────────────────────
  batt_sum -= batt_buf[batt_idx];
  batt_buf[batt_idx] = analogRead(BATT_PIN);
  batt_sum += batt_buf[batt_idx];
  batt_idx = (batt_idx+1) & (BATT_N-1);     // fast modulo (power of 2)
  
 

  // ── Command dispatcher ───────────────────────────────────
  if (Serial.available()) {
    char c = Serial.read();
    while(Serial.available()) Serial.read();   // flush remainder
    Serial.print(F("> ")); Serial.println(c);
    for (uint8_t i=0;i<CMD_N;i++) {
      if (CMD[i].key == c) { CMD[i].fn(); return; }
    }
    Serial.println(F("Unknown. '?' for help."));
  }
}

/* ════════════════════════════════════════════════════════════
   SERVO ATTACH / DETACH
   ════════════════════════════════════════════════════════════ */
void servo_attach() {
  for(uint8_t i=0;i<4;i++) for(uint8_t j=0;j<3;j++){
    servo[i][j].attach(pgm_read_word(&SERVO_PIN[i][j]));
    delay(50);
  }
}
void servo_detach() {
  for(uint8_t i=0;i<4;i++) for(uint8_t j=0;j<3;j++){
    servo[i][j].detach(); delay(50);
  }
}

/* ════════════════════════════════════════════════════════════
   CORE MOVEMENT HELPERS
   ════════════════════════════════════════════════════════════ */

// Set all 4 legs to the same position
inline void set_all(float x, float y, float z) {
  for(uint8_t i=0;i<4;i++) set_site(i,x,y,z);
}
// Set front pair (FR + FL)
inline void set_front(float x, float y, float z) {
  set_site(LEG_FR,x,y,z); set_site(LEG_FL,x,y,z);
}
// Set back pair (BR + BL)
inline void set_back(float x, float y, float z) {
  set_site(LEG_BR,x,y,z); set_site(LEG_BL,x,y,z);
}
// Set all 4 legs individually in one call
inline void set_quad(float x0,float y0,float z0,
                     float x1,float y1,float z1,
                     float x2,float y2,float z2,
                     float x3,float y3,float z3) {
  set_site(0,x0,y0,z0); set_site(1,x1,y1,z1);
  set_site(2,x2,y2,z2); set_site(3,x3,y3,z3);
}

/* ════════════════════════════════════════════════════════════
   BASIC POSES
   ════════════════════════════════════════════════════════════ */
void stand() {
  move_speed = SPEED_SEAT;
  set_site(LEG_FR, lx(LEG_FR), Y_0,          z_front);
  set_site(LEG_FL, lx(LEG_FL), Y_0,          z_front);
  set_site(LEG_BR, X_DEF,      ly(LEG_BR),   z_back);
  set_site(LEG_BL, X_DEF,      ly(LEG_BL),   z_back);
  wait_all_reach();
}
void sit() {
  move_speed = SPEED_SEAT;
  set_all(K,K,Z_ABS);
  wait_all_reach();
}

/* ════════════════════════════════════════════════════════════
   SLEEP / WAKE
   ════════════════════════════════════════════════════════════ */
void robot_sleep() {
  Serial.println(F("Sleeping..."));
  move_speed=2; sit();
  set_all(X_DEF-20, Y_0, Z_ABS);
  wait_all_reach(); delay(500);
  servo_detach();
}
void wake_up() {
  Serial.println(F("Waking..."));
  servo_attach(); delay(200);
  move_speed=SPEED_SEAT;
  set_all(X_DEF, Y_0, Z_ABS);
  for(uint8_t i=0;i<4;i++) for(uint8_t j=0;j<3;j++) site_now[i][j]=site_expect[i][j];
  delay(300); stand();
}

/* ════════════════════════════════════════════════════════════
   BODY SHIFT HELPERS  (used by hand_wave / hand_shake)
   ════════════════════════════════════════════════════════════ */
void body_left(int d) {
  set_site(0,site_now[0][0]+d,K,K); set_site(1,site_now[1][0]+d,K,K);
  set_site(2,site_now[2][0]-d,K,K); set_site(3,site_now[3][0]-d,K,K);
  wait_all_reach();
}
void body_right(int d) { body_left(-d); }
void head_up(int d) {
  set_site(0,K,K,site_now[0][2]-d); set_site(1,K,K,site_now[1][2]+d);
  set_site(2,K,K,site_now[2][2]-d); set_site(3,K,K,site_now[3][2]+d);
  wait_all_reach();
}
void head_down(int d) { head_up(-d); }

/* ════════════════════════════════════════════════════════════
   LOCOMOTION
   ════════════════════════════════════════════════════════════ */
/* ── step_forward ─────────────────────────────────────────────
   Gait: diagonal trot  FL+BR  /  FR+BL
   FIX: LEG_BR (leg 2) was dragged Y_0→Y+52 then swung 52mm back
        in one shot → grinding noise + mechanical stress.
   Fix 1: stride reduced (Y_STEP 40→26) = 52mm max travel not 80mm
   Fix 2: swing broken into TWO hops via midpoint (Y+26) so servo
           never has to slew more than 26mm at once.
   Fix 3: plant FL/FR via midpoint too, same reason.
   ──────────────────────────────────────────────────────────── */
void step_forward() {
  move_speed = SPEED_LEG;
  if (site_now[LEG_FL][1] == Y_0) {
    // ── swing FL forward (two hops) ──
    set_site(LEG_FL,X_DEF,Y_0,          _ZU); wait_all_reach();
    set_site(LEG_FL,X_DEF,Y_0+Y_STEP,   _ZU); wait_all_reach();  // mid
    set_site(LEG_FL,X_DEF,Y_0+2*Y_STEP, _ZU); wait_all_reach();
    set_site(LEG_FL,X_DEF,Y_0+2*Y_STEP, _ZF); wait_all_reach();  // plant
    // ── body glides forward (all on ground) ──
    move_speed = SPEED_BODY;
    set_quad(X_DEF,Y_0,        _ZF,
             X_DEF,Y_0+2*Y_STEP,_ZB,
             X_DEF,Y_0+Y_STEP,  _ZF,
             X_DEF,Y_0+Y_STEP,  _ZB);
    wait_all_reach();
    // ── swing BR back home (two hops — KEY FIX FOR LEG 2) ──
    move_speed = SPEED_LEG;
    set_site(LEG_BR,X_DEF,Y_0+2*Y_STEP,_ZU); wait_all_reach();  // lift
    set_site(LEG_BR,X_DEF,Y_0+Y_STEP,  _ZU); wait_all_reach();  // mid ← relief
    set_site(LEG_BR,X_DEF,Y_0,          _ZU); wait_all_reach();  // home
    set_site(LEG_BR,X_DEF,Y_0,          _ZB); wait_all_reach();  // plant
  } else {
    // ── swing FR forward (two hops) ──
    set_site(LEG_FR,X_DEF,Y_0,          _ZU); wait_all_reach();
    set_site(LEG_FR,X_DEF,Y_0+Y_STEP,   _ZU); wait_all_reach();  // mid
    set_site(LEG_FR,X_DEF,Y_0+2*Y_STEP, _ZU); wait_all_reach();
    set_site(LEG_FR,X_DEF,Y_0+2*Y_STEP, _ZF); wait_all_reach();  // plant
    // ── body glides ──
    move_speed = SPEED_BODY;
    set_quad(X_DEF,Y_0+Y_STEP,  _ZF,
             X_DEF,Y_0+Y_STEP,  _ZB,
             X_DEF,Y_0,          _ZF,
             X_DEF,Y_0+2*Y_STEP,_ZB);
    wait_all_reach();
    // ── swing BL back home (two hops) ──
    move_speed = SPEED_LEG;
    set_site(LEG_BL,X_DEF,Y_0+2*Y_STEP,_ZU); wait_all_reach();  // lift
    set_site(LEG_BL,X_DEF,Y_0+Y_STEP,  _ZU); wait_all_reach();  // mid
    set_site(LEG_BL,X_DEF,Y_0,          _ZU); wait_all_reach();  // home
    set_site(LEG_BL,X_DEF,Y_0,          _ZB); wait_all_reach();  // plant
  }
}

void step_back() {
  move_speed = SPEED_LEG;
  if (site_now[LEG_BL][1] == Y_0) {
    // ── swing BL backward (two hops) ──
    set_site(LEG_BL,X_DEF,Y_0,          _ZU); wait_all_reach();
    set_site(LEG_BL,X_DEF,Y_0+Y_STEP,   _ZU); wait_all_reach();  // mid
    set_site(LEG_BL,X_DEF,Y_0+2*Y_STEP, _ZU); wait_all_reach();
    set_site(LEG_BL,X_DEF,Y_0+2*Y_STEP, _ZB); wait_all_reach();  // plant
    // ── body glides back ──
    move_speed = SPEED_BODY;
    set_quad(X_DEF,Y_0+2*Y_STEP,_ZF,
             X_DEF,Y_0,          _ZB,
             X_DEF,Y_0+Y_STEP,  _ZF,
             X_DEF,Y_0+Y_STEP,  _ZB);
    wait_all_reach();
    // ── swing FR home (two hops) ──
    move_speed = SPEED_LEG;
    set_site(LEG_FR,X_DEF,Y_0+2*Y_STEP,_ZU); wait_all_reach();
    set_site(LEG_FR,X_DEF,Y_0+Y_STEP,  _ZU); wait_all_reach();  // mid
    set_site(LEG_FR,X_DEF,Y_0,          _ZU); wait_all_reach();
    set_site(LEG_FR,X_DEF,Y_0,          _ZF); wait_all_reach();
  } else {
    // ── swing BR backward (two hops — same leg 2 relief) ──
    set_site(LEG_BR,X_DEF,Y_0,          _ZU); wait_all_reach();
    set_site(LEG_BR,X_DEF,Y_0+Y_STEP,   _ZU); wait_all_reach();  // mid
    set_site(LEG_BR,X_DEF,Y_0+2*Y_STEP, _ZU); wait_all_reach();
    set_site(LEG_BR,X_DEF,Y_0+2*Y_STEP, _ZB); wait_all_reach();  // plant
    // ── body glides back ──
    move_speed = SPEED_BODY;
    set_quad(X_DEF,Y_0+Y_STEP,  _ZF,
             X_DEF,Y_0+Y_STEP,  _ZB,
             X_DEF,Y_0+2*Y_STEP,_ZF,
             X_DEF,Y_0,          _ZB);
    wait_all_reach();
    // ── swing FL home (two hops) ──
    move_speed = SPEED_LEG;
    set_site(LEG_FL,X_DEF,Y_0+2*Y_STEP,_ZU); wait_all_reach();
    set_site(LEG_FL,X_DEF,Y_0+Y_STEP,  _ZU); wait_all_reach();  // mid
    set_site(LEG_FL,X_DEF,Y_0,          _ZU); wait_all_reach();
    set_site(LEG_FL,X_DEF,Y_0,          _ZF); wait_all_reach();
  }
}

void turn_left() {
  move_speed = SPEED_TURN;
  if (site_now[LEG_BL][1] == Y_0) {
    set_site(LEG_BL,X_DEF,Y_0,_ZU);                                      wait_all_reach();
    set_quad(turn_x1,turn_y1,_ZF, turn_x0,turn_y0,_ZB,
             turn_x1,turn_y1,_ZF, turn_x0,turn_y0,_ZU);                  wait_all_reach();
    set_site(LEG_BL,turn_x0,turn_y0,_ZB);                                wait_all_reach();
    set_quad(turn_x1,turn_y1,_ZF, turn_x0,turn_y0,_ZB,
             turn_x1,turn_y1,_ZF, turn_x0,turn_y0,_ZB);                  wait_all_reach();
    set_site(LEG_BR,turn_x0,turn_y0,_ZU);                                 wait_all_reach();
    set_quad(X_DEF,Y_0,_ZF, X_DEF,Y_0,_ZU, X_DEF,Y_0+Y_STEP,_ZF, X_DEF,Y_0+Y_STEP,_ZB);
    wait_all_reach();
    set_site(LEG_BR,X_DEF,Y_0,_ZB);                                       wait_all_reach();
  } else {
    set_site(LEG_FR,X_DEF,Y_0,_ZU);                                       wait_all_reach();
    set_quad(turn_x0,turn_y0,_ZU, turn_x1,turn_y1,_ZB,
             turn_x0,turn_y0,_ZF, turn_x1,turn_y1,_ZB);                  wait_all_reach();
    set_site(LEG_FR,turn_x0,turn_y0,_ZF);                                 wait_all_reach();
    set_quad(turn_x0,turn_y0,_ZF, turn_x1,turn_y1,_ZB,
             turn_x0,turn_y0,_ZF, turn_x1,turn_y1,_ZB);                  wait_all_reach();
    set_site(LEG_FL,turn_x0,turn_y0,_ZU);                                 wait_all_reach();
    set_quad(X_DEF,Y_0+Y_STEP,_ZF, X_DEF,Y_0+Y_STEP,_ZB,
             X_DEF,Y_0,_ZU,        X_DEF,Y_0,_ZB);                       wait_all_reach();
    set_site(LEG_FL,X_DEF,Y_0,_ZF);                                       wait_all_reach();
  }
}

void turn_right() {
  move_speed = SPEED_TURN;
  if (site_now[LEG_FL][1] == Y_0) {
    set_site(LEG_FL,X_DEF,Y_0,_ZU);                                       wait_all_reach();
    set_quad(turn_x0,turn_y0,_ZF, turn_x1,turn_y1,_ZB,
             turn_x0,turn_y0,_ZU, turn_x1,turn_y1,_ZB);                  wait_all_reach();
    set_site(LEG_FL,turn_x0,turn_y0,_ZF);                                 wait_all_reach();
    set_quad(turn_x0,turn_y0,_ZF, turn_x1,turn_y1,_ZB,
             turn_x0,turn_y0,_ZF, turn_x1,turn_y1,_ZB);                  wait_all_reach();
    set_site(LEG_FR,turn_x0,turn_y0,_ZU);                                 wait_all_reach();
    set_quad(X_DEF,Y_0,_ZU, X_DEF,Y_0,_ZB, X_DEF,Y_0+Y_STEP,_ZF, X_DEF,Y_0+Y_STEP,_ZB);
    wait_all_reach();
    set_site(LEG_FR,X_DEF,Y_0,_ZF);                                       wait_all_reach();
  } else {
    set_site(LEG_BR,X_DEF,Y_0,_ZU);                                       wait_all_reach();
    set_quad(turn_x1,turn_y1,_ZF, turn_x0,turn_y0,_ZU,
             turn_x1,turn_y1,_ZF, turn_x0,turn_y0,_ZB);                  wait_all_reach();
    set_site(LEG_BR,turn_x0,turn_y0,_ZB);                                 wait_all_reach();
    set_quad(turn_x1,turn_y1,_ZF, turn_x0,turn_y0,_ZB,
             turn_x1,turn_y1,_ZF, turn_x0,turn_y0,_ZB);                  wait_all_reach();
    set_site(LEG_BL,turn_x0,turn_y0,_ZU);                                 wait_all_reach();
    set_quad(X_DEF,Y_0+Y_STEP,_ZF, X_DEF,Y_0+Y_STEP,_ZB,
             X_DEF,Y_0,_ZF,        X_DEF,Y_0,_ZU);                       wait_all_reach();
    set_site(LEG_BL,X_DEF,Y_0,_ZB);                                       wait_all_reach();
  }
}

/* ════════════════════════════════════════════════════════════
   TRICKS  (compact form using set_front / set_back / set_quad)
   ════════════════════════════════════════════════════════════ */

void pushups(int n) {
  stand(); wait_all_reach(); delay(200);
  for(int i=0;i<n;i++){
    move_speed=1; set_all(K,K,-15); wait_all_reach(); delay(100);
    move_speed=3;
    set_front(K,K,_ZF); set_back(K,K,_ZB); wait_all_reach(); delay(200);
  }
}
void jump() {
  move_speed=2; set_all(X_DEF,Y_0,Z_ABS);      wait_all_reach(); delay(60);
  move_speed=8;
  set_front(X_DEF,Y_0,_ZF-18); set_back(X_DEF,Y_0,_ZB-18); wait_all_reach();
  move_speed=3; stand();
}
void happy_jump() { jump(); delay(100); jump(); }

void hand_shake(int n) {
  float xt,yt,zt; move_speed=1;
  if(site_now[LEG_BL][1]==Y_0){
    body_right(15); xt=site_now[LEG_FL][0]; yt=site_now[LEG_FL][1]; zt=site_now[LEG_FL][2];
    move_speed=SPEED_BODY;
    for(int i=0;i<n;i++){
      set_site(LEG_FL,X_DEF-30,Y_0+2*Y_STEP,55); wait_all_reach();
      set_site(LEG_FL,X_DEF-30,Y_0+2*Y_STEP,10); wait_all_reach();
    }
    set_site(LEG_FL,xt,yt,zt); wait_all_reach(); move_speed=1; body_left(15);
  } else {
    body_left(15); xt=site_now[LEG_FR][0]; yt=site_now[LEG_FR][1]; zt=site_now[LEG_FR][2];
    move_speed=SPEED_BODY;
    for(int i=0;i<n;i++){
      set_site(LEG_FR,X_DEF-30,Y_0+2*Y_STEP,55); wait_all_reach();
      set_site(LEG_FR,X_DEF-30,Y_0+2*Y_STEP,10); wait_all_reach();
    }
    set_site(LEG_FR,xt,yt,zt); wait_all_reach(); move_speed=1; body_right(15);
  }
}

void head_shake(int n) {
  move_speed=8;
  for(int i=0;i<n;i++){
    set_site(LEG_FR,K,Y_0+25,_ZF); set_site(LEG_FL,K,Y_0-25,_ZF);
    set_back(K,Y_0,K);             wait_all_reach();
    set_site(LEG_FR,K,Y_0-25,_ZF); set_site(LEG_FL,K,Y_0+25,_ZF);
    set_back(K,Y_0,K);             wait_all_reach();
  }
  stand();
}

void body_dance(int n) {
  sit(); move_speed=1;
  set_all(X_DEF,X_DEF,K); wait_all_reach();
  set_front(X_DEF,X_DEF,_ZF-20); set_back(X_DEF,X_DEF,_ZB-20); wait_all_reach();
  float spd=2; move_speed=spd; head_up(30);
  for(int i=0;i<n;i++){
    if(i>n/4) move_speed=spd*2;
    if(i>n/2) move_speed=spd*3;
    set_site(0,K,X_DEF-20,K); set_site(1,K,X_DEF+20,K);
    set_site(2,K,X_DEF-20,K); set_site(3,K,X_DEF+20,K); wait_all_reach();
    set_site(0,K,X_DEF+20,K); set_site(1,K,X_DEF-20,K);
    set_site(2,K,X_DEF+20,K); set_site(3,K,X_DEF-20,K); wait_all_reach();
  }
  move_speed=spd; head_down(30);
}

void body_twist() {
  move_speed=6;
  set_quad(_X,Y_0+25,_ZF, _X,Y_0-25,_ZB, _X,Y_0-25,_ZF, _X,Y_0+25,_ZB); wait_all_reach(); delay(400);
  set_quad(_X,Y_0-25,_ZF, _X,Y_0+25,_ZB, _X,Y_0+25,_ZF, _X,Y_0-25,_ZB); wait_all_reach(); delay(400);
  stand();
}

void body_roll() {
  move_speed=1; int r=15;
  set_quad(K,K,_ZF+r, K,K,_ZB+r, K,K,_ZF-r, K,K,_ZB-r); wait_all_reach();
  set_front(K,K,_ZF+r); set_back(K,K,_ZB-r);              wait_all_reach();
  set_quad(K,K,_ZF-r, K,K,_ZB-r, K,K,_ZF+r, K,K,_ZB+r); wait_all_reach();
  stand();
}

void breathing_idle(int n) {
  stand(); wait_all_reach(); move_speed=0.6f;
  for(int i=0;i<n;i++){
    set_front(K,K,_ZF-8); set_back(K,K,_ZB-8); wait_all_reach(); delay(600);
    set_front(K,K,_ZF+8); set_back(K,K,_ZB+8); wait_all_reach(); delay(800);
  }
  stand();
}

// ── Reaction poses ──────────────────────────────────────────
void scared() {
  move_speed=7;
  set_front(_X,Y_0,_ZF-30); set_back(_X,Y_0,_ZB+20); wait_all_reach();
}
void shiver() {
  move_speed=4;
  set_front(_X,Y_0,_ZF+20); set_back(_X,Y_0,_ZB-30); wait_all_reach();
  move_speed=8;
  for(int i=0;i<15;i++){
    set_all(_X-4,Y_0,K); wait_all_reach();
    set_all(_X+4,Y_0,K); wait_all_reach();
  }
  stand();
}
void fighting_mode() {
  move_speed=5;
  set_back(_X,Y_0,_ZB+20);
  set_front(_X+15,Y_0+15,_ZF-20); wait_all_reach(); delay(200);
  move_speed=9;
  for(int i=0;i<3;i++){
    set_front(_X+25,Y_0,_ZF+10); wait_all_reach();
    set_front(_X+15,Y_0+15,_ZF-20); wait_all_reach();
  }
  stand();
}
void play_dead() {
  move_speed=2; set_all(X_DEF,Y_0,Z_ABS); wait_all_reach();
  move_speed=1;
  set_quad(_X+30,Y_0+30,_ZBT, _X+30,Y_0+30,_ZBT-10,
           _X+30,Y_0+30,_ZBT, _X+30,Y_0+30,_ZBT-10); wait_all_reach();
}

// ── Locomotion tricks ───────────────────────────────────────
void moonwalk() {
  stand(); wait_all_reach();
  for(int i=0;i<4;i++){
    move_speed=8;  set_all(X_DEF,Y_0+30,Z_DEF); wait_all_reach();
    move_speed=10;
    uint8_t seq[4]={LEG_FR,LEG_BL,LEG_BR,LEG_FL};
    for(uint8_t j=0;j<4;j++){
      set_site(seq[j],X_DEF,Y_0+30,_ZU);  wait_all_reach();
      set_site(seq[j],X_DEF,Y_0,   lz(seq[j])); wait_all_reach();
    }
  }
  stand();
}

void dog_pee() {
  move_speed=3;
  set_site(LEG_FR,_X,Y_0+20,_ZF); set_site(LEG_BR,_X,Y_0+20,_ZB);
  set_site(LEG_FL,_X,Y_0+20,_ZF); set_site(LEG_BL,_X,Y_0,_ZB);
  wait_all_reach();
  move_speed=2; set_site(LEG_BL,_X,Y_0+70,-10); wait_all_reach(); delay(1000);
  for(int i=0;i<3;i++){
    set_site(LEG_BL,_X,Y_0+70,-5);  wait_all_reach();
    set_site(LEG_BL,_X,Y_0+70,-15); wait_all_reach();
  }
  move_speed=3; set_site(LEG_BL,_X,Y_0+20,_ZB); wait_all_reach(); stand();
}

void angry_stomp() {
  stand(); wait_all_reach();
  for(int i=0;i<4;i++){
    move_speed=10;
    set_site(LEG_FR,_X,Y_0,_ZF-50); wait_all_reach(); delay(40);
    set_site(LEG_FR,_X,Y_0,_ZF);    wait_all_reach();
    set_site(LEG_BR,_X,Y_0,_ZB-50); wait_all_reach(); delay(40);
    set_site(LEG_BR,_X,Y_0,_ZB);    wait_all_reach();
  }
}

// ── Expression poses ────────────────────────────────────────
void high_five() {
  move_speed=3;
  set_back(_X,Y_0,_ZBT); wait_all_reach();
  set_front(_X,Y_0,_ZF-50); wait_all_reach();
  for(int i=0;i<4;i++){
    move_speed=5;
    set_front(_X+30,Y_0,_ZF-50); wait_all_reach();
    set_front(_X-10,Y_0,_ZF-50); wait_all_reach();
  }
  delay(400); stand();
}
void bow() {
  move_speed=3;
  set_front(_X,Y_0,_ZF+30); set_back(_X,Y_0,_ZB-20);
  wait_all_reach(); delay(1500); stand();
}
void sneak() {
  move_speed=2;
  set_front(_X+30,Y_0,_ZF+30); set_back(_X+30,Y_0,_ZB+20); wait_all_reach();
}
void dizzy() {
  stand(); wait_all_reach(); move_speed=8;
  for(int i=0;i<3;i++){
    set_site(LEG_FR,_X,Y_0,_ZF-30); set_site(LEG_FL,_X,Y_0,_ZF+30); wait_all_reach();
    set_site(LEG_FR,_X,Y_0,_ZF+30); set_site(LEG_FL,_X,Y_0,_ZF-30); wait_all_reach();
  }
  move_speed=3; stand();
}
void happy_wiggle() {
  stand(); wait_all_reach(); move_speed=8;
  for(int i=0;i<4;i++){
    set_back(_X,Y_0-20,_ZB); wait_all_reach();
    set_back(_X,Y_0+20,_ZB); wait_all_reach();
  }
  stand();
}
void handstand() {
  move_speed=3;
  set_front(_X,Y_0,_ZF+20); set_back(_X+20,Y_0+30,_ZB-60);
  wait_all_reach(); delay(1500); stand();
}
void scratch() {
  move_speed=3;
  set_site(LEG_FL,_X,Y_0+10,_ZF); set_site(LEG_BL,_X,Y_0+10,_ZB);
  set_site(LEG_FR,_X,Y_0,_ZF);
  set_site(LEG_BR,_X,Y_0-30,_ZB+30); wait_all_reach();
  move_speed=10;
  for(int i=0;i<6;i++){
    set_site(LEG_BR,_X,Y_0-40,_ZB+20); wait_all_reach();
    set_site(LEG_BR,_X,Y_0-30,_ZB+35); wait_all_reach();
  }
  move_speed=3; stand();
}
void tiptoe() {
  move_speed=2;
  set_front(_X-30,Y_0,_ZF+40); set_back(_X-30,Y_0,_ZB+30); wait_all_reach();
}
void propose() {
  stand(); wait_all_reach(); move_speed=2;
  set_back(_X,Y_0,_ZB+20); wait_all_reach();
  set_site(LEG_FR,_X,Y_0+40,_ZF-40); wait_all_reach(); delay(3000); stand();
}
void tap_foot() {
  stand(); wait_all_reach();
  set_site(LEG_FL,_X,Y_0+10,_ZF); set_site(LEG_BL,_X,Y_0+10,_ZB); wait_all_reach();
  for(int i=0;i<4;i++){
    move_speed=8;  set_site(LEG_FR,_X,Y_0,_ZF+20); wait_all_reach();
    move_speed=12; set_site(LEG_FR,_X,Y_0,_ZF);    wait_all_reach(); delay(100);
  }
  stand();
}
void blow_kiss() {
  stand(); wait_all_reach(); move_speed=3;
  set_site(LEG_FR,_X-20,Y_0,_ZF-30); wait_all_reach(); delay(500);
  move_speed=2; set_site(LEG_FR,_X,Y_0+40,_ZF-10); wait_all_reach(); delay(800); stand();
}
void digging() {
  stand(); wait_all_reach(); move_speed=3;
  set_front(_X,Y_0,_ZF+20); set_back(_X,Y_0,_ZB-10); wait_all_reach();
  move_speed=10;
  for(int i=0;i<6;i++){
    set_site(LEG_FR,_X,Y_0+30,_ZF-10); wait_all_reach();
    set_site(LEG_FR,_X,Y_0,   _ZF+20); wait_all_reach();
    set_site(LEG_FL,_X,Y_0+30,_ZF-10); wait_all_reach();
    set_site(LEG_FL,_X,Y_0,   _ZF+20); wait_all_reach();
  }
  stand();
}
void salute() {
  move_speed=3;
  set_back(_X,Y_0,_ZB+20); wait_all_reach();
  set_site(LEG_FR,_X-20,Y_0+10,_ZF-40); wait_all_reach(); delay(2000);
  move_speed=6; set_site(LEG_FR,_X,Y_0,_ZF); wait_all_reach(); stand();
}
void shy() {
  move_speed=3;
  set_back(_X,Y_0,_ZBT); set_front(_X,Y_0,_ZF+10); wait_all_reach();
  move_speed=2;
  set_front(_X-30,Y_0+20,_ZF-20); wait_all_reach(); delay(2500); stand();
}
void radar_scan() {
  stand(); wait_all_reach(); move_speed=4;
  for(int i=0;  i<=30;  i+=2){ set_quad(_X+i,Y_0,_ZF,_X-i,Y_0,_ZB,_X-i,Y_0,_ZF,_X+i,Y_0,_ZB); wait_all_reach(); delay(10); }
  delay(200);
  for(int i=30; i>=-30; i-=2){ set_quad(_X+i,Y_0,_ZF,_X-i,Y_0,_ZB,_X-i,Y_0,_ZF,_X+i,Y_0,_ZB); wait_all_reach(); delay(10); }
  delay(200);
  for(int i=-30;i<=0;   i+=2){ set_quad(_X+i,Y_0,_ZF,_X-i,Y_0,_ZB,_X-i,Y_0,_ZF,_X+i,Y_0,_ZB); wait_all_reach(); delay(10); }
  stand();
}
void caterpillar() {
  stand(); wait_all_reach(); move_speed=2;
  set_front(_X+30,Y_0+30,_ZF); set_back(_X-15,Y_0,_ZB); wait_all_reach(); delay(500);
  set_front(_X-15,Y_0,_ZF);    set_back(_X+30,Y_0+30,_ZB); wait_all_reach(); delay(500);
  stand();
}
void surrender() {
  move_speed=3; set_back(_X,Y_0,_ZBT); wait_all_reach();
  move_speed=2;
  set_site(LEG_FR,_X+20,Y_0+20,_ZF-50); set_site(LEG_FL,_X+20,Y_0-20,_ZF-50);
  wait_all_reach(); delay(2500); stand();
}

/* ════════════════════════════════════════════════════════════
   UTILITY
   ════════════════════════════════════════════════════════════ */
void print_battery() {
  Serial.print(F("Batt: "));
  
  Serial.println(F(" mV"));
}
void print_help() {
  Serial.println(F(
    "F/B/L/R  move+turn\n"
    "X/x      stand/sit\n"
    "W/w      sleep/wake\n"
    "n/J/q    push-ups/jump/happy-jump\n"
    "V/U      hand-shake/dance\n"
    "c/o/y    twist/roll/head-no\n"
    "b/f/a/d  scared/shiver/fight/dead\n"
    "m/p/e    moon/pee/stomp\n"
    "//k/t    wave/bow/sneak\n"
    "z/h/j/s  dizzy/wiggle/handstand/scratch\n"
    "T/P/I/K  tiptoe/propose/tap/kiss\n"
    "D/G/Y/M  dig/salute/shy/radar\n"
    "C/Z/i    caterpillar/surrender/breathe\n"
    "r/?      battery/help\n"
    "+/-  LOAD_COMP\n"
    "(/)" "  REAR_STANCE\n"
    "1/2/3    speed x0.5/1/2"
  ));
}

/* ════════════════════════════════════════════════════════════
   CORE IK ENGINE
   ════════════════════════════════════════════════════════════ */

// Timer ISR — 50 Hz
void servo_service() {
  sei();
  static float alpha, beta, gamma;
  for (uint8_t i=0;i<4;i++) {
    for (uint8_t j=0;j<3;j++) {
      float diff = site_expect[i][j] - site_now[i][j];
      float spd  = temp_speed[i][j];
      site_now[i][j] = (fabsf(diff) >= fabsf(spd)) ? site_now[i][j]+spd
                                                     : site_expect[i][j];
    }
    cartesian_to_polar(alpha, beta, gamma,
                       site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }
  rest_counter++;
}

// Division-by-zero guarded set_site
void set_site(int leg, float x, float y, float z) {
  float dx = (x!=K) ? x-site_now[leg][0] : 0.f;
  float dy = (y!=K) ? y-site_now[leg][1] : 0.f;
  float dz = (z!=K) ? z-site_now[leg][2] : 0.f;
  float len = sqrt(dx*dx + dy*dy + dz*dz);
  if (len < 0.001f) {
    temp_speed[leg][0] = temp_speed[leg][1] = temp_speed[leg][2] = 0.f;
  } else {
    float inv = move_speed * speed_mul / len;
    temp_speed[leg][0] = dx*inv;
    temp_speed[leg][1] = dy*inv;
    temp_speed[leg][2] = dz*inv;
  }
  if (x!=K) site_expect[leg][0] = x;
  if (y!=K) site_expect[leg][1] = y;
  if (z!=K) site_expect[leg][2] = z;
}

// Blocking wait with 5-second timeout guard (prevents infinite freeze)
void wait_reach(int leg) {
  uint32_t t0 = millis();
  while (site_now[leg][0] != site_expect[leg][0] ||
         site_now[leg][1] != site_expect[leg][1] ||
         site_now[leg][2] != site_expect[leg][2]) {
    if (millis()-t0 > 5000) { // 5 s safety timeout
      // force-snap to target
      for(uint8_t j=0;j<3;j++) site_now[leg][j]=site_expect[leg][j];
      break;
    }
  }
}
void wait_all_reach() { for(uint8_t i=0;i<4;i++) wait_reach(i); }

// IK: Cartesian → polar angles  (clamped acos to prevent NaN)
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma,
                         volatile float x, volatile float y, volatile float z) {
  float w = (x>=0?1.f:-1.f) * sqrt(x*x + y*y);
  float v = w - LEN_C;
  float d = sqrt(v*v + z*z);
  // precomputed denominators
  float inv_2ad  = (d < 0.001f) ? 0.f : 1.f/(2.f*LEN_A*d);
  float inv_2ab  = 1.f/(2.f*LEN_A*LEN_B);
  float arg_a = constrain((LEN_A*LEN_A - LEN_B*LEN_B + v*v + z*z)*inv_2ad, -1.f, 1.f);
  float arg_b = constrain((LEN_A*LEN_A + LEN_B*LEN_B - v*v - z*z)*inv_2ab, -1.f, 1.f);
  alpha = (atan2(z,v) + acos(arg_a)) * (180.f/PI_F);
  beta  =  acos(arg_b)               * (180.f/PI_F);
  gamma = ((w>=0) ? atan2(y,x) : atan2(-y,-x)) * (180.f/PI_F);
}

// Polar → servo write  (FR/BL share one mapping, BR/FL share the mirrored one)
void polar_to_servo(int leg, float alpha, float beta, float gamma) {
  float a, b, g;
  if (leg==LEG_FR || leg==LEG_BL) { a=90-alpha; b=beta;     g=gamma+90; }
  else                             { a=alpha+90; b=180-beta; g=90-gamma; }
  servo[leg][0].write((int)constrain(a, 0.f, 180.f));
  servo[leg][1].write((int)constrain(b, 0.f, 180.f));
  servo[leg][2].write((int)constrain(g, 0.f, 180.f));
}
