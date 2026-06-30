#include "main_core.h"


// --- Sensor Data ---
// main_core.cpp
USData usData = {
  .pos_x_f          = 0.0f,
  .pos_y_f          = 0.0f,
  .dist_cm          = { US_INVALID_DIST, US_INVALID_DIST, US_INVALID_DIST, US_INVALID_DIST },
  .invalid_count    = {0},
  .coord_x          = US_INVALID_DIST,
  .coord_y          = US_INVALID_DIST,
  .current          = US_COUNT - 1,
  .last_trigger_time = 0,
  .last_display_time = 0,
};
TopMaixPosData topmaixPosData;
RobotMonitor robotMonitor;
RobotMonitor teammateMonitor;

// main_core.cpp — 這裡才是唯一定義
volatile uint32_t echo_start[US_COUNT]    = {0};
volatile uint32_t echo_duration[US_COUNT] = {0};
volatile bool     echo_done[US_COUNT]     = {false};

// --- OLED OBJECT ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void main_core_init() {
    // Initialize all Hardware Serials
    Serial.begin(115200);
    Serial2.begin(115200); 
    Serial3.begin(115200);
    Serial4.begin(115200); 
    Serial5.begin(921600);
    Serial6.begin(115200); 
    Serial7.begin(115200);
    Serial8.begin(921600);

    Wire.begin();
    Wire.setClock(400000); // Fast I2C for OLED

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for(;;); // Lock if OLED fails
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    
    // Pin Setups
    pinMode(Charge_Pin, OUTPUT);
    pinMode(Kicker_Pin, OUTPUT);
    
    // Ensure kicker starts safe
    digitalWrite(Charge_Pin, LOW); // Start charging
    digitalWrite(Kicker_Pin, LOW);

    // Button
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_ENTER, INPUT_PULLUP);
    pinMode(BTN_ESC, INPUT_PULLUP);

    pinMode(COM_O1_PIN, INPUT);
    pinMode(COM_O2_PIN, INPUT);

    for (uint8_t i = 0; i < US_COUNT; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
        digitalWrite(trigPins[i], LOW);
    }

    attachInterrupt(digitalPinToInterrupt(ECHO_F), echoISR<US_FRONT>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_R), echoISR<US_RIGHT>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_B), echoISR<US_BACK>,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_L), echoISR<US_LEFT>,  CHANGE);

/*    pinMode(TRIG_F, OUTPUT);
    pinMode(TRIG_R, OUTPUT);
    pinMode(TRIG_B, OUTPUT);
    pinMode(TRIG_L, OUTPUT);
    pinMode(ECHO_F, INPUT);
    pinMode(ECHO_R, INPUT);
    pinMode(ECHO_B, INPUT);
    pinMode(ECHO_L, INPUT);*/
}

void drawMessage(const char* msg) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println(msg);
    display.display();
}
void ballsensor() {
  uint8_t buffer[6];
  Serial6.write(0xDD);
  while(!Serial6.available());
  Serial6.readBytes(buffer,6);
  if(buffer[0] != 0xCC) return;
  if (buffer[5] != 0xEE) return;
  uint8_t found = buffer[1];
  uint16_t angle = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
  uint8_t dist = buffer[4];
  if (!found) {
    robotMonitor.ball_valid = false;
    robotMonitor.ball_angle = 65535; // Invalid angle
    robotMonitor.ball_dist = 255; // Max distance
    return;
  }
  robotMonitor.ball_valid = true;
  robotMonitor.ball_angle = angle;
  robotMonitor.ball_dist = dist;
}
void kicker_control(bool kick) {
    static uint32_t charge_start_time = 0;
    static bool is_charged = false;
    static bool is_charging = false;

    const uint32_t CHARGE_MS = 5000; 
    uint32_t now = millis();

    // 1. Kick Logic (Priority)
    if (kick && is_charged) {
        digitalWrite(Kicker_Pin, HIGH);
        delay(15); // Solenoid pulse
        digitalWrite(Kicker_Pin, LOW);
        
        is_charged = false; 
        is_charging = false;
        Serial.println("KICKED");
        return; // Exit to prevent immediate re-charge trigger in same frame
    }

    // 2. Charging State Machine
    if (!is_charged && !is_charging) {
        // Start a new charge cycle
        charge_start_time = now;
        is_charging = true;
        digitalWrite(Charge_Pin, HIGH);
        Serial.println("CHARGING...");
    }

    if (is_charging) {
        if (now - charge_start_time >= CHARGE_MS) {
            // Charge complete
            digitalWrite(Charge_Pin, LOW);
            is_charging = false;
            is_charged = true;
            Serial.println("READY TO KICK");
        }
    }
}

void readTopMaix() {
  uint8_t buffer[10];
  Serial3.write(0xDD);
  //while(!Serial3.available()){Serial.println("cam");};
  Serial3.readBytes(buffer,10);
  topmaixPosData.valid = 0;

  if(buffer[0] == 0xCC && buffer[9] == 0xEE){
    uint8_t checksum = (buffer[1] + buffer[2] + buffer[3] + buffer[4]+ buffer[5]+ buffer[6]+ buffer[7]) & 0xFF;
    if(checksum == buffer[8]){
      //Serial.printf("duration: %ld\n", micros() - start);
      topmaixPosData.valid = true;
      topmaixPosData.x = (int8_t)(uint8_t)buffer[1];
      topmaixPosData.y = (int8_t)(uint8_t)buffer[2];
      topmaixPosData.status = buffer[3];

      topmaixPosData.ball_found = buffer[4];
      topmaixPosData.ball_angle = (uint16_t)buffer[5] | ((uint16_t)buffer[6] << 8);
      topmaixPosData.ball_dist = buffer[7];
    }
  }
  else{
    topmaixPosData.valid = false;  // checksum error
  }
}

void roleSwitchControl() {
    if (robotMonitor.role == 1) {
        // Defense-specific logic
    } else if (robotMonitor.role == 2) {
        // Offense-specific logic
    }
}

void update_all_sensor(){
    ballsensor();
    readTopMaix();
    updateUS();
}
/*
void sensor_fusion() {
    // Ball Sensor Fusion
    // Position Sensor Fusion(Ultrasonic + TopMaix)
    robotMonitor.pos_x = (int8_t)round(usData.coord_x);
    robotMonitor.pos_y = (int8_t)round(usData.coord_y);
    
    Serial.printf("US distances: L=%.2f, R=%.2f, F=%.2f, B=%.2f\n", usData.dist_cm[US_LEFT], usData.dist_cm[US_RIGHT], usData.dist_cm[US_FRONT], usData.dist_cm[US_BACK]);
    Serial.printf("US coord_x: %.2f, coord_y: %.2f\n", usData.coord_x, usData.coord_y);
    Serial.printf("TopMaix x: %d, y: %d\n", topmaixPosData.x, topmaixPosData.y);
    Serial.printf("finalpos_x: %d, finalpos_y: %d\n", robotMonitor.pos_x, robotMonitor.pos_y);
    
}

// Kalman filter state — persists between calls
static float estimated_x    = 0.0f, estimated_y    = 0.0f;
static float uncertainty_x  = 1.0f, uncertainty_y  = 1.0f, uncertainty_xy = 0.0f;
static uint32_t last_update_time = 0;

// Noise tuning
static const float PROCESS_NOISE        = 0.01f;  // how much position drifts per step
static const float ULTRASONIC_NOISE     = 0.20f;  // how noisy is the ultrasonic
static const float CAMERA_NOISE         = 0.05f;  // how noisy is the camera
static const float OUTLIER_THRESHOLD    = 3.0f;   // reject reading if too far off (sigma)

void sensor_fusion() {

    // ------------------------------------------------------------------
    // PREDICT — time has passed, so we're slightly less certain now
    // ------------------------------------------------------------------
    uint32_t current_time = micros();
    float dt = (last_update_time == 0) ? 0.005f : (current_time - last_update_time) * 1e-6f;
    last_update_time = current_time;
    dt = constrain(dt, 0.001f, 0.1f);

    float drift = PROCESS_NOISE * dt * dt;
    uncertainty_x  += drift;
    uncertainty_y  += drift;

    // ------------------------------------------------------------------
    // FUSE ONE SENSOR — called once for ultrasonic, once for camera
    // returns true if the reading was accepted (not an outlier)
    // ------------------------------------------------------------------
    auto fuse = [&](float sensor_x, float sensor_y, float sensor_noise) -> bool {

        float gap_x = sensor_x - estimated_x;   // how far off is this sensor?
        float gap_y = sensor_y - estimated_y;

        // Innovation covariance S = uncertainty + sensor_noise
        float S_xx = uncertainty_x  + sensor_noise;
        float S_xy = uncertainty_xy;
        float S_yy = uncertainty_y  + sensor_noise;

        float det = S_xx*S_yy - S_xy*S_xy;
        if (fabsf(det) < 1e-9f) return false;   // degenerate, skip

        float inv    = 1.0f / det;
        float Si_xx  =  S_yy * inv;
        float Si_xy  = -S_xy * inv;
        float Si_yy  =  S_xx * inv;

        // Outlier gate — reject if sensor reading is suspiciously far from estimate
        float mahalanobis_dist_sq = gap_x*(Si_xx*gap_x + Si_xy*gap_y)
                                  + gap_y*(Si_xy*gap_x + Si_yy*gap_y);
        if (mahalanobis_dist_sq > OUTLIER_THRESHOLD * OUTLIER_THRESHOLD) return false;

        // Kalman gain — how much to trust this sensor right now
        // (high uncertainty OR low sensor noise → trust sensor more)
        float gain_xx = uncertainty_x*Si_xx + uncertainty_xy*Si_xy;
        float gain_xy = uncertainty_x*Si_xy + uncertainty_xy*Si_yy;
        float gain_yx = uncertainty_xy*Si_xx + uncertainty_y*Si_xy;
        float gain_yy = uncertainty_xy*Si_xy + uncertainty_y*Si_yy;

        // Nudge estimate toward the sensor reading
        estimated_x += gain_xx*gap_x + gain_xy*gap_y;
        estimated_y += gain_yx*gap_x + gain_yy*gap_y;

        // We learned something — uncertainty shrinks
        float new_unc_x  = (1.0f-gain_xx)*uncertainty_x  + (-gain_xy)*uncertainty_xy;
        float new_unc_xy = (1.0f-gain_xx)*uncertainty_xy + (-gain_xy)*uncertainty_y;
        float new_unc_yx = (-gain_yx)*uncertainty_x      + (1.0f-gain_yy)*uncertainty_xy;
        float new_unc_y  = (-gain_yx)*uncertainty_xy     + (1.0f-gain_yy)*uncertainty_y;

        uncertainty_x  = new_unc_x;
        uncertainty_xy = 0.5f*(new_unc_xy + new_unc_yx);   // keep symmetric
        uncertainty_y  = new_unc_y;
        return true;
    };

    // ------------------------------------------------------------------
    // FUSE — ultrasonic first, then camera
    // ------------------------------------------------------------------
    bool ultrasonic_accepted = fuse(usData.coord_x, usData.coord_y, ULTRASONIC_NOISE);

    bool camera_accepted = false;
    if (topmaixPosData.valid) {
        camera_accepted = fuse((float)topmaixPosData.x, (float)topmaixPosData.y, CAMERA_NOISE);
    }

    // ------------------------------------------------------------------
    // Write fused result to robotMonitor
    // ------------------------------------------------------------------
    robotMonitor.pos_x = (int8_t)round(estimated_x);
    robotMonitor.pos_y = (int8_t)round(estimated_y);

    // ------------------------------------------------------------------
    // Ball Sensor Fusion
    // ------------------------------------------------------------------
    if (topmaixPosData.valid && topmaixPosData.ball_found) {
        robotMonitor.ball_valid = true;
        robotMonitor.ball_angle = topmaixPosData.ball_angle;
        robotMonitor.ball_dist  = topmaixPosData.ball_dist;
    } else {
        robotMonitor.ball_valid = false;
        robotMonitor.ball_angle = 65535;
        robotMonitor.ball_dist  = 255;
    }

    // ------------------------------------------------------------------
    // Debug
    // ------------------------------------------------------------------
    Serial.printf("US distances: L=%.2f, R=%.2f, F=%.2f, B=%.2f\n",
                  usData.dist_cm[US_LEFT], usData.dist_cm[US_RIGHT],
                  usData.dist_cm[US_FRONT], usData.dist_cm[US_BACK]);
    Serial.printf("US coord_x: %.2f, coord_y: %.2f  [%s]\n",
                  usData.coord_x, usData.coord_y, ultrasonic_accepted ? "OK" : "GATE");
    Serial.printf("TopMaix x: %d, y: %d  [%s]\n",
                  topmaixPosData.x, topmaixPosData.y, camera_accepted ? "OK" : "GATE/SKIP");
    Serial.printf("finalpos_x: %d, finalpos_y: %d\n",
                  robotMonitor.pos_x, robotMonitor.pos_y);
}
*/
/*
void sensor_fusion() {

    // --- Kalman state (persists between calls) ---
    static bool  first_run     = true;
    static float est_x         = 0.0f;
    static float est_y         = 0.0f;
    static float unc_x         = 999.0f;
    static float unc_y         = 999.0f;
    static uint32_t last_time  = 0;

    // --- Noise config ---
    const float CAMERA_NOISE      = 0.05f;
    const float ULTRASONIC_NOISE  = 0.20f;
    const float PROCESS_NOISE     = 0.01f;

    // --- Seed on first call ---
    if (first_run) {
        est_x      = usData.coord_x;
        est_y      = usData.coord_y;
        first_run  = false;
    }

    // --- Predict: time passed, uncertainty grows ---
    uint32_t now = micros();
    float dt = (last_time == 0) ? 0.005f : (now - last_time) * 1e-6f;
    last_time = now;
    dt = constrain(dt, 0.001f, 0.1f);

    unc_x += PROCESS_NOISE * dt * dt;
    unc_y += PROCESS_NOISE * dt * dt;

    // --- Update: fuse one sensor into the estimate ---
    // gain = how much to trust the sensor vs our current estimate
    // high uncertainty OR low sensor noise → trust sensor more
    auto fuse = [&](float sensor_x, float sensor_y, float sensor_noise) {
        float gain_x = unc_x / (unc_x + sensor_noise);
        float gain_y = unc_y / (unc_y + sensor_noise);
        est_x  = est_x + gain_x * (sensor_x - est_x);
        est_y  = est_y + gain_y * (sensor_y - est_y);
        unc_x  = (1.0f - gain_x) * unc_x;
        unc_y  = (1.0f - gain_y) * unc_y;
    };

    // --- Fuse ultrasonic ---
    fuse(usData.coord_x, usData.coord_y, ULTRASONIC_NOISE);

    // --- Fuse camera (only when valid) ---
    if (topmaixPosData.valid) {
        fuse((float)topmaixPosData.x, (float)topmaixPosData.y, CAMERA_NOISE);
    }

    // --- Write result ---
    robotMonitor.pos_x = (int8_t)round(est_x);
    robotMonitor.pos_y = (int8_t)round(est_y);

    // --- Ball ---
    if (topmaixPosData.valid && topmaixPosData.ball_found) {
        robotMonitor.ball_valid = true;
        robotMonitor.ball_angle = topmaixPosData.ball_angle;
        robotMonitor.ball_dist  = topmaixPosData.ball_dist;
    } 
    //robotMonitor.ball_valid = false;
    //robotMonitor.ball_angle = 65535;
    //robotMonitor.ball_dist  = 255;
    

    // --- Debug ---
    Serial.printf("US coord_x: %.2f, coord_y: %.2f\n", usData.coord_x, usData.coord_y);
    Serial.printf("TopMaix x: %d, y: %d\n", topmaixPosData.x, topmaixPosData.y);
    Serial.printf("finalpos_x: %d, finalpos_y: %d\n", robotMonitor.pos_x, robotMonitor.pos_y);
}
*/

void sensor_fusion() {

    // =========================================================
    // KALMAN FILTER STATE
    // static = these variables survive between calls,
    //          the filter "remembers" its last estimate
    // =========================================================
    static bool     first_run  = true;
    static float    est_x      = 0.0f;   // current best estimate of robot X
    static float    est_y      = 0.0f;   // current best estimate of robot Y
    static float    unc_x      = 999.0f; // how uncertain we are about X (starts very high)
    static float    unc_y      = 999.0f; // how uncertain we are about Y (starts very high)
    static uint32_t last_time  = 0;

    // =========================================================
    // NOISE CONSTANTS — the main knobs you tune
    //
    // CAMERA_NOISE:
    //   How much you distrust the camera reading each update.
    //   LOWER  → trust camera more → final position pulls strongly toward camera
    //   HIGHER → trust camera less → camera has little effect
    //   Start low because camera gives accurate absolute position.
    //
    // ULTRASONIC_NOISE:
    //   How much you distrust the ultrasonic reading each update.
    //   LOWER  → trust ultrasonic more → final position pulls strongly toward US
    //   HIGHER → trust ultrasonic less → US has little effect
    //   Set higher than camera because ultrasonic is noisier and narrower FOV.
    //
    // PROCESS_NOISE:
    //   How much the robot's true position can drift between loop iterations.
    //   LOWER  → filter assumes robot barely moves → estimate changes slowly, very smooth
    //            but lags behind if robot moves fast
    //   HIGHER → filter assumes robot moves a lot → estimate reacts faster to sensors
    //            but becomes noisier
    //   Increase this if your robot moves fast or estimate feels sluggish.
    // =========================================================
    //const float CAMERA_NOISE     = 0.05f;
    const float CAMERA_NOISE     = 0.075f;
    const float ULTRASONIC_NOISE = 0.20f;
    const float PROCESS_NOISE    = 0.01f;

    // =========================================================
    // SEED — runs only on the very first call
    // Without this, est_x/y starts at (0,0) which is probably
    // not where your robot is, causing the filter to take many
    // loops to converge to the real position.
    // We seed from ultrasonic since it is always available.
    // =========================================================
    if (first_run) {
        est_x     = usData.coord_x;
        est_y     = usData.coord_y;
        first_run = false;
    }

    // =========================================================
    // PREDICT — called every loop
    //
    // We don't know where the robot moved since last loop,
    // so we leave est_x/y unchanged (best guess: it stayed put).
    // But we admit our confidence dropped a little — unc grows.
    //
    // dt = time since last loop in seconds
    // The longer the gap, the more uncertainty we add.
    // =========================================================
    uint32_t now = micros();
    float dt = (last_time == 0) ? 0.005f : (now - last_time) * 1e-6f;
    last_time = now;
    dt = constrain(dt, 0.001f, 0.1f);  // clamp: ignore gaps <1ms or >100ms

    unc_x += PROCESS_NOISE * dt * dt;  // uncertainty grows with time
    unc_y += PROCESS_NOISE * dt * dt;

    // =========================================================
    // FUSE — blends one sensor reading into the current estimate
    //
    // gain = how much to trust THIS sensor RIGHT NOW
    //
    //         unc
    // gain = ─────────────────
    //         unc + sensor_noise
    //
    // Case 1: unc is large (we are lost) → gain → 1.0 → trust sensor fully
    // Case 2: unc is small (we are confident) → gain → 0.0 → ignore sensor
    // Case 3: sensor_noise is large (noisy sensor) → gain → 0.0 → ignore it
    // Case 4: sensor_noise is small (accurate sensor) → gain → 1.0 → trust it
    //
    // After the update:
    //   est moves toward the sensor reading by (gain × gap)
    //   unc shrinks because we just learned something new
    // =========================================================
    auto fuse = [&](float sensor_x, float sensor_y, float sensor_noise) {
        float gain_x = unc_x / (unc_x + sensor_noise);  // 0.0 to 1.0
        float gain_y = unc_y / (unc_y + sensor_noise);

        est_x = est_x + gain_x * (sensor_x - est_x);   // nudge estimate toward sensor
        est_y = est_y + gain_y * (sensor_y - est_y);

        unc_x = (1.0f - gain_x) * unc_x;   // we learned something → less uncertain
        unc_y = (1.0f - gain_y) * unc_y;
    };

    // --- Fuse ultrasonic (always available) ---
    fuse(usData.coord_x, usData.coord_y, ULTRASONIC_NOISE);

    // --- Fuse camera (only when TopMaix has a valid fix) ---
    if (topmaixPosData.valid) {
        fuse((float)topmaixPosData.x, (float)topmaixPosData.y, CAMERA_NOISE);
    }

    // =========================================================
    // WRITE RESULT
    // est_x/y is the fused position — not raw US, not raw camera,
    // but a weighted blend of both based on their reliability.
    // =========================================================
    robotMonitor.pos_x = (int8_t)round(est_x);
    robotMonitor.pos_y = (int8_t)round(est_y);

    // =========================================================
    // BALL SENSOR FUSION
    // Camera is the only source for ball info — use it directly
    // when valid, otherwise mark ball as not found.
    // =========================================================
    if (topmaixPosData.valid && topmaixPosData.ball_found) {
        robotMonitor.ball_valid = true;
        robotMonitor.ball_angle = topmaixPosData.ball_angle;
        robotMonitor.ball_dist  = topmaixPosData.ball_dist;
    } else {
        ;
        //robotMonitor.ball_valid = false;
        //robotMonitor.ball_angle = 65535;  // sentinel: no angle
        //robotMonitor.ball_dist  = 255;    // sentinel: max distance
    }

    // --- Debug ---
    Serial.printf("US coord_x: %.2f, coord_y: %.2f\n", usData.coord_x, usData.coord_y);
    Serial.printf("TopMaix x: %d, y: %d\n", topmaixPosData.x, topmaixPosData.y);
    Serial.printf("finalpos_x: %d, finalpos_y: %d\n", robotMonitor.pos_x, robotMonitor.pos_y);
}
void sendMotor(float vx, float vy, float rot_v, int target_heading) {
    uint8_t data[6];
    data[0] = PROTOCAL_HEADER;
    data[1] = (int8_t)vx;
    data[2] = (int8_t)vy;
    // Scale up by 100 so we don't lose decimals (e.g., 0.5 rad/s becomes 50)
    data[3] = (int8_t)(rot_v * 100.0f);
    // Scale down by 10 to fit 0-360 into a single byte (0-36)
    data[4] = (int8_t)(target_heading / 10);
    data[5] = PROTOCAL_END;
    Serial8.write(data, sizeof(data));
}

void sendMaincoreData() {
    uint8_t data[10];
    data[0] = PROTOCAL_HEADER;
    data[1] = robotMonitor.pos_x;
    data[2] = robotMonitor.pos_y;
    data[3] = robotMonitor.ball_valid ? 1 : 0;
    data[4] = robotMonitor.ball_angle;
    data[5] = robotMonitor.ball_angle>>8; // High byte
    data[6] = robotMonitor.ball_dist;
    data[7] = robotMonitor.ball_dist>>8; // High byte
    data[8] = robotMonitor.role; // Reserved
    data[9] = PROTOCAL_END;
    Serial8.write(data, sizeof(data));
}

void stopMotors() {
    sendMotor(0.0f, 0.0f, 0.0f, 90); // Send zero velocities to stop
}


bool UI_Interface(){
    update_all_sensor();
    static uint32_t lastDisplayTime = 0;
    switch (robotMonitor.currentState) {
        case RobotState::STATE_READY:
            if (digitalRead(BTN_ENTER) == LOW) {
                Serial8.write(LS_CAL_START); // Command to Sensor Board
                drawMessage("SCANNING");
                delay(500);
                robotMonitor.currentState = RobotState::STATE_CALIBRATING;
                break;
            }

            if (millis() - lastDisplayTime > 100) { // 每 0.1 秒更新一次螢幕
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(SSD1306_WHITE);
                display.setCursor(0, 0);
                display.printf("ball dist: %d\n", robotMonitor.ball_dist);
                display.printf("ball angle: %d\n",robotMonitor.ball_angle);
                display.display();
                lastDisplayTime = millis();
            }
            if(digitalRead(BTN_UP) == LOW){
                display.clearDisplay();
                display.display();
                return false;
            }
            //offense

            //defense

            break;
        case RobotState::STATE_CALIBRATING:
            if (digitalRead(BTN_ESC) == LOW) {
                Serial8.write(LS_CAL_END); // Command to Save
                drawMessage("SAVING...");
                delay(500);
                robotMonitor.currentState = RobotState::STATE_SAVING;
            }
            break;
        case RobotState::STATE_SAVING:
            if(Serial8.available()){
                uint8_t c = Serial8.read();
                if(c == LS_CAL_END){
                    drawMessage("SAVED!");
                    delay(1000); // 讓 SAVED 停一下
                }
                // 回到初始狀態
                drawMessage("READY");
                delay(1000);
                display.clearDisplay();
                robotMonitor.currentState = RobotState::STATE_READY;
            }
            break;
    }
    return true;
}

void triggerUS(uint8_t i) {
  digitalWrite(trigPins[i], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[i], LOW);
}


// 合併 updateFilteredUS + markInvalidUS
void updateReading(uint8_t i, uint32_t duration) {
  if (duration > 100 && duration < 15000) {
    float raw = duration * 0.0343f / 2.0f;
    usData.dist_cm[i] = isValidUS(usData.dist_cm[i])
                      ? usData.dist_cm[i] * (1.0f - US_FILTER_ALPHA) + raw * US_FILTER_ALPHA
                      : raw;
    usData.invalid_count[i] = 0;
  } else if (++usData.invalid_count[i] >= US_INVALID_LIMIT) {
    usData.dist_cm[i] = US_INVALID_DIST;
  }
}

void updateUSCoordinate() {
    // ==================== X 軸邏輯 (右正, 左負) ====================
    if (isValidUS(usData.dist_cm[US_LEFT]) && usData.dist_cm[US_LEFT] > 5.0f && usData.dist_cm[US_LEFT] < 30.0f && 
        (usData.dist_cm[US_LEFT] < usData.dist_cm[US_RIGHT] || !isValidUS(usData.dist_cm[US_RIGHT]))) {
        // 左邊在 5~30 且比右邊小 -> 採信左邊
        usData.coord_x = -(91.0f - usData.dist_cm[US_LEFT]); 
    } 
    else if (isValidUS(usData.dist_cm[US_RIGHT]) && usData.dist_cm[US_RIGHT] > 5.0f && usData.dist_cm[US_RIGHT] < 30.0f && 
            (usData.dist_cm[US_RIGHT] < usData.dist_cm[US_LEFT] || !isValidUS(usData.dist_cm[US_LEFT]))) {
        // 右邊在 5~30 且比左邊小 -> 採信右邊
        usData.coord_x = 91.0f - usData.dist_cm[US_RIGHT]; 
    } 
    else {
        // 兩邊都大於 30，走原本的相減
        usData.coord_x = (isValidUS(usData.dist_cm[US_LEFT]) && isValidUS(usData.dist_cm[US_RIGHT]))
                        ? (usData.dist_cm[US_LEFT] - usData.dist_cm[US_RIGHT]) / 2.0f : US_INVALID_DIST;
    }
    // ==================== Y 軸邏輯 (前後總長 243, 半長 121.5) ====================
    if (isValidUS(usData.dist_cm[US_BACK]) && usData.dist_cm[US_BACK] > 5.0f && usData.dist_cm[US_BACK] < 30.0f && 
        (!isValidUS(usData.dist_cm[US_FRONT]) || usData.dist_cm[US_FRONT] > 150.0f)) {
        // 靠近後方，前方很大 -> 車在後半場 (負半場)
        usData.coord_y = -(121.5f - usData.dist_cm[US_BACK]);
    } 
    else if (isValidUS(usData.dist_cm[US_FRONT]) && usData.dist_cm[US_FRONT] > 5.0f && usData.dist_cm[US_FRONT] < 30.0f && 
            (!isValidUS(usData.dist_cm[US_BACK]) || usData.dist_cm[US_BACK] > 150.0f)) {
        // 靠近前方，後方很大 -> 車在前半場 (正半場)
        usData.coord_y = 121.5f - usData.dist_cm[US_FRONT]; 
    } 
    else {
        // 兩邊都正常，維持原來的相減
        usData.coord_y = (isValidUS(usData.dist_cm[US_BACK]) && isValidUS(usData.dist_cm[US_FRONT]))
                        ? (usData.dist_cm[US_BACK] - usData.dist_cm[US_FRONT]) / 2.0f : US_INVALID_DIST;
    }

    // ==================== 球門厚度補償 ====================
    if (usData.coord_x < 30.0f && usData.coord_x > -30.0f) {
        if (usData.coord_y > 0.0f) {
            usData.coord_y -= 6.0f;  // y > 0 時，往 0 修正 (減 6)
        } 
        else if (usData.coord_y < 0.0f) {
            usData.coord_y += 6.0f;  // y < 0 時，往 0 修正 (加 6)
        }
    }
}

void updateUS() {
    static uint8_t  current_us        = US_COUNT - 1;
    static uint32_t last_trigger_time = 0;
    if (millis() - last_trigger_time >= 30) {
        last_trigger_time = millis();
        current_us = (current_us + 1) % US_COUNT;
        echo_done[current_us] = false;
        triggerUS(current_us);
    }

    for (uint8_t i = 0; i < US_COUNT; i++) {
        if (!echo_done[i]) continue;

        noInterrupts();
        uint32_t duration = echo_duration[i];
        echo_done[i] = false;
        interrupts();

        updateReading(i, duration);
        updateUSCoordinate();
    }
}

/*
void updateUS() {
  static uint8_t  current_us        = US_COUNT - 1;
  static uint32_t last_trigger_time = 0;

  if (millis() - last_trigger_time >= 20) {
    last_trigger_time = millis();
    current_us = (current_us + 1) % US_COUNT;

    digitalWrite(trigPins[current_us], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[current_us], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[current_us], LOW);

    uint32_t duration = pulseIn(echoPins[current_us], HIGH, 30000);

    updateReading(current_us, duration);
    updateUSCoordinate();
  }
}*/