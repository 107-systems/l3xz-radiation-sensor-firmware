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
static CanardNodeID const DEFAULT_RADIATION_SENSOR_NODE_ID = 98;

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

CyphalHeap<Node::DEFAULT_O1HEAP_SIZE> node_heap;
Node node_hdl(node_heap.data(), node_heap.size(), DEFAULT_RADIATION_SENSOR_NODE_ID);

static uint16_t update_period_radiation_cpm_ms = 10000;

/* REGISTER ***************************************************************************/

static RegisterNatural8  reg_rw_uavcan_node_id                    ("uavcan.node.id",                     Register::Access::ReadWrite, Register::Persistent::No, DEFAULT_RADIATION_SENSOR_NODE_ID, [&node_hdl](uint8_t const reg_val) { node_hdl.setNodeId(reg_val); });
static RegisterString    reg_ro_uavcan_node_description           ("uavcan.node.description",            Register::Access::ReadWrite, Register::Persistent::No, "L3X-Z Radiation Sensor");
static RegisterNatural16 reg_ro_uavcan_pub_radiation_cpm_id       ("uavcan.pub.radiation_cpm.id",        Register::Access::ReadOnly,  Register::Persistent::No, ID_RADIATION_VALUE);
static RegisterString    reg_ro_uavcan_pub_radiation_cpm_type     ("uavcan.pub.radiation_cpm.type",      Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Integer16.1.0");
static RegisterNatural16 reg_rw_rad_update_period_radiation_cpm_ms("rad.update_period_ms.radiation_cpm", Register::Access::ReadWrite, Register::Persistent::No, update_period_radiation_cpm_ms, nullptr, nullptr, [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterList      reg_list;

/* NODE INFO **************************************************************************/

static NodeInfo node_info
(
  /* uavcan.node.Version.1.0 protocol_version */
  1, 0,
  /* uavcan.node.Version.1.0 hardware_version */
  1, 0,
  /* uavcan.node.Version.1.0 software_version */
  0, 1,
  /* saturated uint64 software_vcs_revision_id */
  NULL,
  /* saturated uint8[16] unique_id */
  OpenCyphalUniqueId(),
  /* saturated uint8[<=50] name */
  "107-systems.l3xz-fw_radiation_sensor"
);

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
  node_hdl.setNodeId(DEFAULT_RADIATION_SENSOR_NODE_ID);

  node_info.subscribe(node_hdl);

  reg_list.add(reg_rw_uavcan_node_id);
  reg_list.add(reg_ro_uavcan_node_description);
  reg_list.add(reg_ro_uavcan_pub_radiation_cpm_id);
  reg_list.add(reg_ro_uavcan_pub_radiation_cpm_type);
  reg_list.add(reg_rw_rad_update_period_radiation_cpm_ms);
  reg_list.subscribe(node_hdl);


  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), []() { mcp2515.onExternalEventHandler(); }, LOW);

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
  /* Process all pending OpenCyphal actions.
   */
  node_hdl.spinSome([](CanardFrame const & frame) -> bool { return mcp2515.transmit(frame); });

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

  if((now - prev_radiation) > update_period_radiation_cpm_ms)
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
