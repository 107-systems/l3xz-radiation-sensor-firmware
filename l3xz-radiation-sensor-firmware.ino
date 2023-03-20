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

#include <pico/stdlib.h>
#include <hardware/watchdog.h>

#include <SPI.h>

#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
#include <107-Arduino-UniqueId.h>
#include <107-Arduino-CriticalSection.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::_register;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_CS_PIN  = 3;
static int const MKRCAN_MCP2515_INT_PIN = 9;
static int const RADIATION_PIN          = 10;

static CanardPortID const ID_RADIATION_VALUE = 3000U;
static CanardNodeID const DEFAULT_RADIATION_SENSOR_NODE_ID = 21;

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
                         SPI.beginTransaction(MCP2515x_SPI_SETTING);
                         digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
                       },
                       []()
                       {
                         digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
                         SPI.endTransaction();
                       },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

Node::Heap<Node::DEFAULT_O1HEAP_SIZE> node_heap;
Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); }, DEFAULT_RADIATION_SENSOR_NODE_ID);

Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>
  (Heartbeat_1_0::_traits_::FixedPortId, 1*1000*1000UL /* = 1 sec in usecs. */);

Publisher<Integer16_1_0> radiation_tick_pub = node_hdl.create_publisher<Integer16_1_0>
  (ID_RADIATION_VALUE, 1*1000*1000UL /* = 1 sec in usecs. */);

/* REGISTER ***************************************************************************/

static CanardNodeID node_id = DEFAULT_RADIATION_SENSOR_NODE_ID;

#if __GNUC__ >= 11

const auto node_registry = node_hdl.create_registry();

const auto reg_rw_uavcan_node_id                = node_registry->expose("cyphal.node.id", {}, node_id);
const auto reg_ro_uavcan_node_description       = node_registry->route ("cyphal.node.description", {true}, []() { return "L3X-Z Radiation Sensor"; });
const auto reg_ro_uavcan_pub_radiation_cpm_id   = node_registry->route ("cyphal.pub.radiation_cpm.id", {true}, []() { return ID_RADIATION_VALUE; });
const auto reg_ro_uavcan_pub_radiation_cpm_type = node_registry->route ("cyphal.pub.radiation_cpm.type", {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; } );

#endif /* __GNUC__ >= 11 */

static volatile int16_t rad_tick_cnt = 0;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  //while(!Serial) { Watchdog.reset(); } /* only for debug */

  /* NODE INFO **************************************************************************/
  static const auto node_info = node_hdl.create_node_info
  (
    /* uavcan.node.Version.1.0 protocol_version */
    1, 0,
    /* uavcan.node.Version.1.0 hardware_version */
    1, 0,
    /* uavcan.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    OpenCyphalUniqueId(),
    /* saturated uint8[<=50] name */
    "107-systems.l3xz-radiation-sensor"
  );

  /* Setup LED pins and initialize */
  pinMode(RADIATION_PIN, INPUT_PULLUP);

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

  /* set up radiation measurement */
  attachInterrupt(digitalPinToInterrupt(RADIATION_PIN),
                  []() { rad_tick_cnt++; },
                  RISING);
}

void loop()
{
  /* Process all pending OpenCyphal actions.
   */
  {
    CriticalSection crit_sec;
    node_hdl.spinSome();
  }

  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_heartbeat = 0;
  static unsigned long prev_radiation = 0;

  unsigned long const now = millis();

  if((now - prev_heartbeat) > 1000)
  {
    prev_heartbeat = now;

    Heartbeat_1_0 msg;

    msg.uptime = millis() / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);
   }

  if((now - prev_radiation) > 10000)
  {
    prev_radiation = now;

    Integer16_1_0 msg;

    {
      CriticalSection crit_sec;
      msg.value = rad_tick_cnt;
      rad_tick_cnt = 0;
    }

    radiation_tick_pub->publish(msg);

    if (Serial) {
      Serial.print("Radiation Value: ");
      Serial.println(msg.value);
    }
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame);
}
