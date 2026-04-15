// Add this to your existing pigpio motor template:
// Quadrature encoder reader (A/B channels) using pigpio callbacks.
// - Counts edges reliably
// - Provides position count + velocity estimate
//
// IMPORTANT ELECTRICAL NOTE:
// If your encoder is powered from 5V, its A/B outputs are likely 5V.
// Raspberry Pi GPIO is 3.3V only → use a level shifter/divider before connecting.

#include <pigpio.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>

class QuadratureEncoder {
public:
  // If you know your encoder resolution, set these for velocity conversion:
  // counts_per_wheel rev_output = 333 counts/rev
  // If you count 4 edges (x4 decoding), effective counts/rev = 4 * 333 = 1332 
  QuadratureEncoder(int gpioA, int gpioB,
                    double counts_per_rev_effective = 1332,
                    unsigned glitch_us = 50)
      : gpioA_(gpioA),
        gpioB_(gpioB),
        cpr_eff_(counts_per_rev_effective),
        glitch_us_(glitch_us) {}

  void init() {
    gpioSetMode(gpioA_, PI_INPUT);
    gpioSetMode(gpioB_, PI_INPUT);

    // Enable pull-ups if you have open-drain outputs or long wires (often helpful).
    // If your encoder outputs are push-pull, pull-ups still usually OK.
    gpioSetPullUpDown(gpioA_, PI_PUD_UP);
    gpioSetPullUpDown(gpioB_, PI_PUD_UP);

    // Debounce / noise filtering in pigpio (very useful near motors):
    gpioGlitchFilter(gpioA_, glitch_us_);
    gpioGlitchFilter(gpioB_, glitch_us_);

    // Initialize last state
    int a = gpioRead(gpioA_);
    int b = gpioRead(gpioB_);
    last_state_.store((a << 1) | b);

    // Register callbacks on BOTH edges for both channels (x4 decoding)
    cbA_ = gpioSetAlertFuncEx(gpioA_, &QuadratureEncoder::alertTrampoline, this);
    cbB_ = gpioSetAlertFuncEx(gpioB_, &QuadratureEncoder::alertTrampoline, this);

    // For velocity estimation:
    last_vel_time_us_.store(gpioTick());
    last_vel_count_.store(count_.load());
  }

  void shutdown() {
    // Disable callbacks
    gpioSetAlertFunc(gpioA_, nullptr);
    gpioSetAlertFunc(gpioB_, nullptr);
    // Optionally remove glitch filters
    gpioGlitchFilter(gpioA_, 0);
    gpioGlitchFilter(gpioB_, 0);
  }

  int32_t getCount() const { return count_.load(); }

  void resetCount(int32_t v = 0) { count_.store(v); }

  // Returns wheel angular velocity (rad/s) using a time window since last call.
  // Call this at a fixed interval in your control loop (e.g., every 10 ms).
  double getOmegaRadPerSec() {
    const uint32_t now_us = gpioTick();
    const int32_t c_now = count_.load();

    const uint32_t prev_us = last_vel_time_us_.load();
    const int32_t  c_prev  = last_vel_count_.load();

    uint32_t dt_us = now_us - prev_us;
    if (dt_us == 0) dt_us = 1;

    const int32_t dc = c_now - c_prev;

    last_vel_time_us_.store(now_us);
    last_vel_count_.store(c_now);

    const double dt = static_cast<double>(dt_us) * 1e-6;
    const double rev_per_sec = (static_cast<double>(dc) / cpr_eff_) / dt;
    return rev_per_sec * 2.0 * 3.141592653589793;
  }

private:
  int gpioA_;
  int gpioB_;
  double cpr_eff_;
  unsigned glitch_us_;

  std::atomic<int32_t> count_{0};
  std::atomic<int> last_state_{0};

  // velocity state
  std::atomic<uint32_t> last_vel_time_us_{0};
  std::atomic<int32_t>  last_vel_count_{0};

  // pigpio requires static trampoline
  static void alertTrampoline(int gpio, int level, uint32_t tick, void* user) {
    (void)gpio; (void)tick;
    if (level == PI_TIMEOUT) return; // ignore timeouts
    auto* self = static_cast<QuadratureEncoder*>(user);
    self->onEdge();
  }

  void onEdge() {
    // Read both pins to get current AB state
    const int a = gpioRead(gpioA_);
    const int b = gpioRead(gpioB_);
    const int new_state = (a << 1) | b;

    const int old_state = last_state_.exchange(new_state);

    // State transition table for quadrature (x4)
    // Valid transitions: 00->01->11->10->00 (one direction) and reverse.
    // We compute delta based on old/new combination.
    const int idx = (old_state << 2) | new_state;
    // Map idx -> +1/-1/0
    // This table is standard for Gray-code quadrature.
    static const int8_t delta[16] = {
      0, +1, -1,  0,
     -1,  0,  0, +1,
     +1,  0,  0, -1,
      0, -1, +1,  0
    };

    const int8_t d = delta[idx & 0x0F];
    if (d != 0) {
      count_.fetch_add(d);
    }
  }

  int cbA_{0};
  int cbB_{0};
};

///////////////////////
//Example snippet to include to the motor_control.cpp inside main() after gpioInitialise())
///////////////////////

// Example encoder pins (choose your GPIOs)
constexpr int ENC_A = 17;
constexpr int ENC_B = 25;

// For Pololu 25D 20.4:1, 48 CPR:
// counts per output rev (1x) ≈ 48 * 20.4 = 979.2
// x4 decoding => 3916.8
QuadratureEncoder enc(ENC_A, ENC_B, 3916.8 /*effective CPR*/, 50 /*glitch us*/);
enc.init();

// In your control loop (e.g., 100 Hz or 200 Hz):
for (;;) {
  int32_t pos = enc.getCount();
  double omega = enc.getOmegaRadPerSec();   // rad/s
  // ... use omega for Kv * v_wheel damping, logging, etc.
  // std::cout << "pos=" << pos << " omega=" << omega << " rad/s\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}