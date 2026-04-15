// Dagu Rover 5 — Task 3: Virtual joystick via WebSocket
// RPi runs Crow HTTP + WebSocket server
// Phone browser opens the page, nipple.js joystick sends {x,y} at 20 Hz
// Differential drive: left = y + x, right = y - x, clamped to [-1, 1]
// Watchdog: motors stop if no message received for 500 ms
//
// Requires pigpiod running:
//   sudo systemctl start pigpiod
//
// Compile:
//   g++ joystick.cpp -o joystick -I. -lpigpiod_if2 -lpthread -std=c++17 -O2
//
// Run:
//   ./joystick
// Then open http://<RPi-IP>:8080 on your phone

#include "crow_all.h"
#include <pigpiod_if2.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>

// ── MotorL298N ───────────────────────────────────────────────────────────────

class MotorL298N {
public:
    MotorL298N(int pi, int enaPin, int in1Pin, int in2Pin)
        : pi_(pi), ena_(enaPin), in1_(in1Pin), in2_(in2Pin) {}

    void setup() {
        set_mode(pi_, in1_, PI_OUTPUT);
        set_mode(pi_, in2_, PI_OUTPUT);
        gpio_write(pi_, in1_, 0);
        gpio_write(pi_, in2_, 0);
        set_PWM_frequency(pi_, ena_, 1000);
        set_PWM_range(pi_, ena_, 255);
        set_PWM_dutycycle(pi_, ena_, 0);
    }

    // speed: -1.0 (full reverse) to +1.0 (full forward)
    void setSpeed(double speed) {
        speed = std::clamp(speed, -1.0, 1.0);
        int pwm = static_cast<int>(std::abs(speed) * 255);
        if (speed > 0.01) {
            gpio_write(pi_, in1_, 1);
            gpio_write(pi_, in2_, 0);
        } else if (speed < -0.01) {
            gpio_write(pi_, in1_, 0);
            gpio_write(pi_, in2_, 1);
        } else {
            gpio_write(pi_, in1_, 0);
            gpio_write(pi_, in2_, 0);
            pwm = 0;
        }
        set_PWM_dutycycle(pi_, ena_, pwm);
    }

    void stop() { setSpeed(0.0); }

private:
    int pi_, ena_, in1_, in2_;
};

// ── Globals ──────────────────────────────────────────────────────────────────

static std::mutex              motor_mutex;
static std::atomic<double>     cmd_left{0.0}, cmd_right{0.0};
static std::atomic<bool>       running{true};
static std::atomic<int64_t>    last_msg_ms{0};  // epoch ms of last WS message

static int64_t now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

// ── Watchdog thread ──────────────────────────────────────────────────────────
// Stops motors if no joystick message received within 500 ms

static void watchdog(MotorL298N& left, MotorL298N& right) {
    while (running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        int64_t elapsed = now_ms() - last_msg_ms.load();
        if (elapsed > 500) {
            std::lock_guard<std::mutex> lock(motor_mutex);
            left.stop();
            right.stop();
            cmd_left.store(0.0);
            cmd_right.store(0.0);
        }
    }
}

// ── HTML page (served to phone browser) ──────────────────────────────────────

static const char* HTML = R"html(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>Rover 5 Joystick</title>
  <style>
    * { margin:0; padding:0; box-sizing:border-box; }
    body {
      background:#1a1a2e;
      color:#eee;
      font-family: monospace;
      display:flex;
      flex-direction:column;
      align-items:center;
      height:100vh;
      overflow:hidden;
    }
    #status {
      margin-top:20px;
      font-size:1.1em;
      padding:6px 18px;
      border-radius:20px;
      background:#16213e;
    }
    #status.ok  { color:#4ade80; }
    #status.err { color:#f87171; }
    #info {
      margin-top:12px;
      font-size:0.9em;
      color:#aaa;
      text-align:center;
    }
    #zone {
      flex:1;
      width:100%;
      display:flex;
      align-items:center;
      justify-content:center;
    }
    #estop {
      margin-bottom:30px;
      padding:14px 40px;
      font-size:1.1em;
      background:#dc2626;
      color:#fff;
      border:none;
      border-radius:12px;
      cursor:pointer;
    }
    #estop:active { background:#991b1b; }
  </style>
</head>
<body>
  <div id="status" class="err">Disconnected</div>
  <div id="info">L: 0.00 &nbsp; R: 0.00</div>
  <div id="zone"></div>
  <button id="estop" ontouchstart="eStop()" onclick="eStop()">STOP</button>

  <script src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.10.1/nipplejs.min.js"></script>
  <script>
    var ws, joystick;
    var x = 0, y = 0;
    var statusEl = document.getElementById('status');
    var infoEl   = document.getElementById('info');

    function connect() {
      ws = new WebSocket('ws://' + location.host + '/ws');
      ws.onopen  = function() {
        statusEl.textContent = 'Connected';
        statusEl.className   = 'ok';
      };
      ws.onclose = function() {
        statusEl.textContent = 'Disconnected';
        statusEl.className   = 'err';
        setTimeout(connect, 1000);
      };
    }

    function eStop() {
      x = 0; y = 0;
      if (ws && ws.readyState === 1)
        ws.send(JSON.stringify({x:0, y:0}));
    }

    // Send joystick at 20 Hz
    setInterval(function() {
      if (ws && ws.readyState === 1) {
        ws.send(JSON.stringify({x: x, y: y}));
        var L = Math.max(-1, Math.min(1,  y + x));
        var R = Math.max(-1, Math.min(1,  y - x));
        infoEl.innerHTML = 'L: ' + L.toFixed(2) + ' &nbsp; R: ' + R.toFixed(2);
      }
    }, 50);

    joystick = nipplejs.create({
      zone: document.getElementById('zone'),
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: '#4ade80',
      size: 150
    });

    joystick.on('move', function(evt, data) {
      // data.vector: {x, y} normalised -1..1; nipple y is inverted
      x =  data.vector.x;
      y = -data.vector.y;
    });
    joystick.on('end', function() { x = 0; y = 0; });

    connect();
  </script>
</body>
</html>
)html";

// ── main ─────────────────────────────────────────────────────────────────────

int main() {
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        std::cerr << "pigpiod connection failed — run: sudo systemctl start pigpiod" << std::endl;
        return 1;
    }
    std::cout << "Connected to pigpiod OK" << std::endl;

    // Left:  ENA=GPIO12, IN1=GPIO23, IN2=GPIO24
    // Right: ENB=GPIO13, IN3=GPIO27, IN4=GPIO22
    MotorL298N left (pi, 12, 23, 24);
    MotorL298N right(pi, 13, 27, 22);
    left.setup();
    right.setup();

    // Start watchdog thread
    last_msg_ms.store(now_ms());
    std::thread wd(watchdog, std::ref(left), std::ref(right));

    crow::SimpleApp app;

    // Serve joystick HTML
    CROW_ROUTE(app, "/")([]() {
        crow::response res(HTML);
        res.set_header("Content-Type", "text/html");
        return res;
    });

    // WebSocket endpoint
    CROW_ROUTE(app, "/ws")
        .websocket()
        .onopen([](crow::websocket::connection& /*conn*/) {
            std::cout << "Phone connected" << std::endl;
        })
        .onclose([&](crow::websocket::connection& /*conn*/, const std::string& /*reason*/) {
            std::cout << "Phone disconnected — stopping motors" << std::endl;
            std::lock_guard<std::mutex> lock(motor_mutex);
            left.stop();
            right.stop();
        })
        .onmessage([&](crow::websocket::connection& /*conn*/,
                       const std::string& data, bool /*binary*/) {
            last_msg_ms.store(now_ms());

            auto json = crow::json::load(data);
            if (!json) return;

            double jx = json["x"].d();
            double jy = json["y"].d();

            // Differential drive mixer
            double l = std::clamp(jy + jx, -1.0, 1.0);
            double r = std::clamp(jy - jx, -1.0, 1.0);

            std::lock_guard<std::mutex> lock(motor_mutex);
            left.setSpeed(l);
            right.setSpeed(r);
        });

    std::cout << "Server running on http://0.0.0.0:8080" << std::endl;
    std::cout << "Open http://<RPi-IP>:8080 on your phone" << std::endl;

    app.port(8080).multithreaded().run();

    running.store(false);
    wd.join();
    left.stop();
    right.stop();
    pigpio_stop(pi);
    return 0;
}
