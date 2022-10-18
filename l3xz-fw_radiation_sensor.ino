/*
 * Software for the leg controller for the L3X-Z Hexapod
 *
 * Hardware:
 *   - Arduino Nano 33 IoT
 *   - MCP2515
 * https://github.com/107-systems/l3xz-hw_leg-controller
 *
 * Used Subject-IDs
 * 3000 - pub - Integer16 - radiation value
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <SPI.h>

#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>

/**************************************************************************************
 * DEFINES
 **************************************************************************************/

#define LED1_PIN 2
#define LED2_PIN A7
#define LED3_PIN A6
#define RADIATION_PIN 10

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int          const MKRCAN_MCP2515_CS_PIN  = 3;
static int          const MKRCAN_MCP2515_INT_PIN = 9;

static CanardPortID const ID_RADIATION_VALUE = 3000U;
static CanardNodeID const RADIATION_SENSOR_NODE_ID = 98;

static SPISettings  const MCP2515x_SPI_SETTING{1000000, MSBFIRST, SPI_MODE0};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void radiation_count();
void onReceiveBufferFull(CanardFrame const &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515([]()
                       {
                         noInterrupts();
                         SPI.beginTransaction(MCP2515x_SPI_SETTING);
                         digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
                       },
                       []()
                       {
                         digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
                         SPI.endTransaction();
                         interrupts();
                       },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

Node node_hdl([](CanardFrame const & frame) -> bool { return mcp2515.transmit(frame); });

Heartbeat_1_0<> hb;
volatile int radiation_ticks = 0;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  //while(!Serial) { Watchdog.reset(); } /* only for debug */

  /* Setup LED pins and initialize */
  pinMode(LED1_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED2_PIN, LOW);
  pinMode(LED3_PIN, OUTPUT);
  digitalWrite(LED3_PIN, LOW);
  pinMode(RADIATION_PIN, INPUT_PULLUP);

  /* Configure OpenCyphal node. */
  node_hdl.setNodeId(RADIATION_SENSOR_NODE_ID);

  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), []() { mcp2515.onExternalEventHandler(); }, FALLING);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  /* Configure initial heartbeat */
  hb.data.uptime = 0;
  hb = Heartbeat_1_0<>::Health::NOMINAL;
  hb = Heartbeat_1_0<>::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  /* set up radiation measurement */
  attachInterrupt(digitalPinToInterrupt(RADIATION_PIN), radiation_count, RISING);
  radiation_ticks = 0;
}

void loop()
{
  /* toggle LEDS */
  static bool flag_led=0;
  if((millis()%200)==0)
  {
    if(flag_led==0) // execute only once
    {
      if(digitalRead(LED2_PIN)==LOW)
      {
        digitalWrite(LED2_PIN, HIGH);
        digitalWrite(LED3_PIN, LOW);
      }
      else
      {
        digitalWrite(LED2_PIN, LOW);
        digitalWrite(LED3_PIN, HIGH);
      }
    }
    flag_led=1;
  }
  else flag_led=0;

  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_heartbeat = 0;
  static unsigned long prev_radiation = 0;

  unsigned long const now = millis();

  if((now - prev_heartbeat) > 1000)
  {
     hb.data.uptime = millis() / 1000;
     hb = Heartbeat_1_0<>::Mode::OPERATIONAL;
     node_hdl.publish(hb);
     prev_heartbeat = now;
   }

  if((now - prev_radiation) > 10000)
  {
    noInterrupts();
    Integer16_1_0<ID_RADIATION_VALUE> uavcan_radiation_value;
    uavcan_radiation_value.data.value = radiation_ticks;
    radiation_ticks = 0;
    interrupts();
    node_hdl.publish(uavcan_radiation_value);

    if (Serial) {
      Serial.print("Radiation Value: ");
      Serial.println(uavcan_radiation_value.data.value);
    }

    prev_radiation = now;
  }

  /* Transmit all enqeued CAN frames */
  while(node_hdl.transmitCanFrame()) { }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame, micros());
}

void radiation_count()
{
  radiation_ticks++;
  if (Serial) Serial.println(radiation_ticks);
}
