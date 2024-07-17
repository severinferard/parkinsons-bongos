#include <Arduino_FreeRTOS.h>
#include <USB-MIDI.h>
#include <CapacitiveSensor.h>
#include <Adafruit_NeoPixel.h>

#define SERIAL_MONITOR_BAUDRATE 115200
#define MAX_NOTE_DURATION_MS 5000

#define MIDI_RECV_CHANNEL 1
#define MIDI_RECV_NOTE_1 67
#define MIDI_RECV_NOTE_2 68
#define MIDI_RECV_NOTE_3 69

#define TOM_1_PIXEL_PIN 5
#define TOM_1_PIXEL_COUNT 30
#define TOM_1_TOUCH_SEND_PIN 4
#define TOM_1_TOUCH_READ_PIN 2

#define CS_TOUCH_THRESHOLD 1000




USBMIDI_CREATE_DEFAULT_INSTANCE();
using namespace MIDI_NAMESPACE;

struct tom_task_param_s {
  uint16_t id;
  uint16_t touch_pin;
  Adafruit_NeoPixel *strip;
};

enum tom_task_event_e {
  TOM_TASK_EVENT_NOTE_ON = 1,
  TOM_TASK_EVENT_NOTE_OFF,
  TOM_TASK_EVENT_TOUCH,
};

enum tom_state_e {
  TOM_STATE_IDLE,
  TOM_STATE_WAITING,
  TOM_STATE_SUCCESS,
  TOM_STATE_FAILED
};

CapacitiveSensor touchSensor1 = CapacitiveSensor(TOM_1_TOUCH_SEND_PIN, TOM_1_TOUCH_READ_PIN);

TaskHandle_t taskTom1Handler;

Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(TOM_1_PIXEL_COUNT, TOM_1_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void xprintf(const char *format, ...) {
  static char buffer[512]; 
  va_list args;
  va_start(args, format);
  vsprintf(buffer, format, args);
  va_end(args);
  Serial.print(buffer);
}

static void setStripColor(Adafruit_NeoPixel *strip, uint8_t r, uint8_t g, uint8_t b) {
  uint32_t numPixel = strip->numPixels();

  for (uint32_t i = 0; i < numPixel; i++) {
    strip->setPixelColor(i, r, g, b);
  }
  strip->show();
}

static void OnNoteOn(byte channel, byte note, byte velocity) {
  xprintf("Note ON from channel %u, note: %u, velocity: %u\n", channel, note, velocity);

  if (channel == MIDI_RECV_CHANNEL) {
    switch (note) {
      case MIDI_RECV_NOTE_1:
        Serial.println("Sending to Tom 1");
        xTaskNotify(taskTom1Handler, TOM_TASK_EVENT_NOTE_ON, eSetValueWithOverwrite);
        break;

      default:
        break;
    }
  }
}

static void OnNoteOff(byte channel, byte note, byte velocity) {
  xprintf("Note OFF from channel %u, note: %u, velocity: %u\n", channel, note, velocity);

  if (channel == MIDI_RECV_CHANNEL) {
    switch (note) {
      case MIDI_RECV_NOTE_1:
        Serial.println("Sending to Tom 1");
        xTaskNotify(taskTom1Handler, TOM_TASK_EVENT_NOTE_OFF, eSetValueWithOverwrite);
        break;

      default:
        break;
    }
  }
}

struct tom_task_param_s tom_1_params = { .id = 1, .touch_pin = A0, .strip = &strip1 };

void vTaskTom(void *pvParameters) {
  uint32_t event;
  uint32_t t0;
  uint8_t state = TOM_STATE_IDLE;
  const struct tom_task_param_s *params = (struct tom_task_param_s *)pvParameters;

  pinMode(params->touch_pin, INPUT);
  delay(1000);

  for (;;) {

    int ret = xTaskNotifyWait(0, 0, &event, portMAX_DELAY);

    if (ret == pdTRUE) {
      switch (event) {
        case TOM_TASK_EVENT_NOTE_ON:
          t0 = millis();
          state = TOM_STATE_WAITING;
          Serial.println("blue");
          setStripColor(params->strip, 0, 0, 50);
          break;

        case TOM_TASK_EVENT_NOTE_OFF:
          setStripColor(params->strip, 0, 0, 0);
          if (state == TOM_STATE_WAITING) {
            setStripColor(params->strip, 50, 0, 0);
          }
          state = TOM_STATE_IDLE;
          break;

        case TOM_TASK_EVENT_TOUCH:
          if (state == TOM_STATE_WAITING) {
            setStripColor(params->strip, 0, 50, 0);
            uint32_t delta_t = millis() - t0;

            // Write params->id and delta_t to the file on the sd card

            state = TOM_STATE_SUCCESS;
          }
          break;
      }
    }
  }
}






void setup() {
  Serial.begin(SERIAL_MONITOR_BAUDRATE);

  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.setHandleNoteOn(OnNoteOn);
  MIDI.setHandleNoteOff(OnNoteOff);

  strip1.begin();
  strip1.show();
  strip1.setPixelColor(0, 50, 0, 0);
  strip1.show();

  touchSensor1.set_CS_AutocaL_Millis(0xFFFFFFFF);

  // put your setup code here, to run once:
  xTaskCreate(vTaskTom,  // Task function
              "Tom1",    // Task name
              128,       // Stack size
              (void *)&tom_1_params,
              3,                  // Priority
              &taskTom1Handler);  // TaskHandle
}

void loop() {
  MIDI.read();

  long ret1 = touchSensor1.capacitiveSensor(30);

  if (ret1 > CS_TOUCH_THRESHOLD) {
    xTaskNotify(taskTom1Handler, TOM_TASK_EVENT_TOUCH, eSetValueWithOverwrite);
  }
}
