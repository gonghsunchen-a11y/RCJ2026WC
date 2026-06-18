#define IR_Port_Count 10

unsigned long lastNoBallSend = 0;   // For 10 Hz "no ball" message

uint8_t distance;
uint8_t max_port;

TaskHandle_t Task1;
TaskHandle_t Task2;

uint8_t ir_pins[IR_Port_Count]  = {1,3,5,6,8,10,12,14,15,17};
uint16_t ir_weight[IR_Port_Count];

volatile bool read_done_flag = false;

void rgbLEDWrite(uint8_t red_val, uint8_t green_val, uint8_t blue_val) {
  rmt_data_t led_data[24];
  // default WS2812B color order is G, R, B
  int color[3] = {red_val, green_val, blue_val};
  int i = 0;
  for (int col = 0; col < 3; col++) {
    for (int bit = 0; bit < 8; bit++) {
      if ((color[col] & (1 << (7 - bit)))) {
        // HIGH bit
        led_data[i].level0 = 1;     // T1H
        led_data[i].duration0 = 8;  // 0.8us
        led_data[i].level1 = 0;     // T1L
        led_data[i].duration1 = 4;  // 0.4us
      } else {
        // LOW bit
        led_data[i].level0 = 1;     // T0H
        led_data[i].duration0 = 4;  // 0.4us
        led_data[i].level1 = 0;     // T0L
        led_data[i].duration1 = 8;  // 0.8us
      }
      i++;
    }
  }
  rmtWrite(38, led_data, RMT_SYMBOLS_OF(led_data), RMT_WAIT_FOR_EVER);
}

// ============ IR Reading ============
void IR_reading() {
  dist = 0x00;
  max_port = 0xff;
  
  for (uint8_t i = 0; i < IR_Port_Count; i++) {
    ir_weight[i] = 0;
  }
  
  uint64_t start = micros();
  while (micros() - start < 833) {
    for (uint8_t i = 0; i < IR_Port_Count; i++) {
      if (!digitalRead(ir_pins[i])) {
        ir_weight[i]++;
      }
    }
  }
  
  for (uint8_t i = 0; i < IR_Port_Count; i++) {
    if(ir_weight[i] > 0){
      dist = dist + 1;
    }
  }
  
  if(dist){
    uint16_t max_value = 0;
    for (uint8_t i = 0; i < IR_Port_Count; i++) {
      if(ir_weight[i] > max_value) {
        max_index = i;
        max_value = ir_weight[i];
      }
    }
    max_port = ir_port[max_index];
    dist = IR_Port_Count + 1 - dist;
  }
  else{
    dis = 0;
    max_index = 0xff;
  }
  read_done_flag = true;
}

// ============ Setup ============
void setup() {
  Serial.begin(115200);
  Serial0.begin(115200);
  rmtInit(38, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
  rgbLEDWrite(125,0,0);

  for (uint8_t i = 0; i < IR_Port_Count; i++) {
    pinMode(ir_pins[i], INPUT_PULLUP);
  }

  BLEDevice::init("POD");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pDataCharacteristic = pService->createCharacteristic(
    DATA_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pDataCharacteristic->setCallbacks(new MyDataCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
  
  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
}


// ============ Task1: IR Reading ============
void Task1code(void *parameter) {
  while (1) {
    if(!read_done_flag){
      IR_reading();
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ============ Task2: Max + Serial0 ============
void Task2code(void *parameter) {
  while (1) {
    while (!read_done_flag);
    if(Serial0.read() == 0xbb){
      if(dis!=0){
        //dis = 16 - (uint8_t)(16.0 * ir_weight[max_index] / (float)max_value);
        uint8_t send_data = ((dis & 0x0F) << 4) | (max_port & 0x0F);
          Serial0.write(0xAA);
          Serial0.write(send_data);
          Serial0.write(ball_possession);
          Serial0.write(0xEE);
        }
      else{
        Serial0.write(0xAA);
        Serial0.write(0xFF);
        Serial0.write(0xFF);
        Serial0.write(0xEE);
      }
    }
    read_done_flag = false;
    // Step 2: Debug print
    if(Serial.available()){
      Serial.println("");
      for (uint8_t i = 0; i < IR_Port_Count; i++) {
        Serial.print(ir_port[i]);
        Serial.print("=");
        Serial.println(ir_weight[i]);
      }
      if(max_port == 3 || max_port == 4){
        rgbLEDWrite(0,0,0);
        rgbLEDWrite(0,125,0);
      }
      else{
        rgbLEDWrite(0,0,0);
        rgbLEDWrite(125,0,0);
      }
      Serial.print("Max = ");
      Serial.print(max_port);
      Serial.print(" @ distance ");
      Serial.println(dis);
      delay(100);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void loop() {
  // Empty
}
