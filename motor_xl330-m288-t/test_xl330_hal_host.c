#include "dxl/xl330_hal.h"

#include <stdio.h>
#include <string.h>

#ifndef STM32F103xB
#error "Host compile check must define the STM32F103xB device macro before including STM32 HAL headers."
#endif

typedef struct {
    uint8_t writes[8][DXL_MAX_PACKET_SIZE];
    uint16_t write_len[8];
    uint8_t write_count;
    uint8_t tx_mode_count;
    uint8_t rx_mode_count;
} FakeTransport;

static DXL2_Status fake_write(void *user, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    (void)timeout_ms;
    FakeTransport *fake = (FakeTransport *)user;
    if (fake == 0 || fake->write_count >= 8u || len > DXL_MAX_PACKET_SIZE) {
        return DXL2_ERR_BAD_PARAM;
    }
    memcpy(fake->writes[fake->write_count], data, len);
    fake->write_len[fake->write_count] = len;
    fake->write_count++;
    return DXL2_OK;
}

static DXL2_Status fake_read_byte(void *user, uint8_t *byte, uint32_t timeout_ms)
{
    (void)user;
    (void)byte;
    (void)timeout_ms;
    return DXL2_ERR_TIMEOUT;
}

static void fake_set_tx_mode(void *user)
{
    ((FakeTransport *)user)->tx_mode_count++;
}

static void fake_set_rx_mode(void *user)
{
    ((FakeTransport *)user)->rx_mode_count++;
}

static uint32_t fake_millis(void *user)
{
    (void)user;
    return 0u;
}

DXL2_Status dxl2_stm32hal_transport_init(DXL2_Transport *transport,
                                          DXL2_STM32HAL_Context *context,
                                          UART_HandleTypeDef *huart)
{
    (void)transport;
    (void)context;
    (void)huart;
    return DXL2_ERR_UNSUPPORTED;
}

DXL2_Status dxl2_stm32hal_transport_init_half_duplex(DXL2_Transport *transport,
                                                     DXL2_STM32HAL_Context *context,
                                                     UART_HandleTypeDef *huart,
                                                     GPIO_TypeDef *dir_port,
                                                     uint16_t dir_pin,
                                                     GPIO_PinState tx_state)
{
    (void)transport;
    (void)context;
    (void)huart;
    (void)dir_port;
    (void)dir_pin;
    (void)tx_state;
    return DXL2_ERR_UNSUPPORTED;
}

static int expect(int condition, const char *message)
{
    if (!condition) {
        printf("FAIL %s\n", message);
        return 1;
    }
    printf("PASS %s\n", message);
    return 0;
}

static uint8_t packet_inst(const FakeTransport *fake, uint8_t index)
{
    return fake->writes[index][7];
}

static uint16_t packet_param_u16(const FakeTransport *fake, uint8_t index, uint8_t offset)
{
    return (uint16_t)fake->writes[index][8u + offset] |
           ((uint16_t)fake->writes[index][9u + offset] << 8);
}

static int test_public_header_surface(void)
{
    XL330_HandleTypeDef handle;
    XL330_InitTypeDef init;
    XL330_HalfDuplexInitTypeDef hd_init;
    XL330_StorageConfig storage;
    memset(&handle, 0, sizeof(handle));
    memset(&init, 0, sizeof(init));
    memset(&hd_init, 0, sizeof(hd_init));
    memset(&storage, 0, sizeof(storage));

    init.storage = &storage;
    hd_init.storage = &storage;
    return expect(sizeof(handle) > 0u && sizeof(init) > 0u && sizeof(hd_init) > 0u,
                  "public HAL-style header compiles and exposes handle/init/storage types");
}

static int test_controller_stages_until_commit(void)
{
    int fails = 0;
    FakeTransport fake;
    DXL2_Transport transport;
    XL330_Controller ctl;
    memset(&fake, 0, sizeof(fake));
    memset(&transport, 0, sizeof(transport));
    memset(&ctl, 0, sizeof(ctl));

    transport.user = &fake;
    transport.write = fake_write;
    transport.read_byte = fake_read_byte;
    transport.set_tx_mode = fake_set_tx_mode;
    transport.set_rx_mode = fake_set_rx_mode;
    transport.millis = fake_millis;

    fails += expect(dxl2_bus_init(&ctl.group.bus, &transport) == DXL2_OK, "fake bus init succeeds");
    ctl.group.ids = ctl.ids;
    ctl.group.count = 0u;

    fails += expect(xl330_add(&ctl, 1u) == DXL2_OK, "first motor added");
    fails += expect(xl330_add(&ctl, 2u) == DXL2_OK, "second motor added");
    fails += expect(xl330_mode(&ctl, XL330_MODE_POSITION) == DXL2_OK, "mode is staged");
    fails += expect(xl330_torque(&ctl, XL330_TORQUE_ENABLE) == DXL2_OK, "torque is staged");
    fails += expect(xl330_goal_position(&ctl, 1u, 0x01020304) == DXL2_OK, "goal for id 1 is staged");
    fails += expect(xl330_goal_position(&ctl, 2u, 0x05060708) == DXL2_OK, "goal for id 2 is staged");
    fails += expect(fake.write_count == 0u, "staging performs no transport writes before commit");

    fails += expect(xl330_commit(&ctl) == DXL2_OK, "commit succeeds with fake tx-only transport");
    fails += expect(fake.write_count == 4u, "commit emits mode, restored torque, and goal sync writes");
    fails += expect(packet_inst(&fake, 0) == DXL_INST_SYNC_WRITE && packet_param_u16(&fake, 0, 0) == XL330_ADDR_TORQUE_ENABLE,
                    "mode change first disables torque through sync-write");
    fails += expect(packet_inst(&fake, 1) == DXL_INST_SYNC_WRITE && packet_param_u16(&fake, 1, 0) == XL330_ADDR_OPERATING_MODE,
                    "mode change sync-writes operating mode");
    fails += expect(packet_inst(&fake, 2) == DXL_INST_SYNC_WRITE && packet_param_u16(&fake, 2, 0) == XL330_ADDR_TORQUE_ENABLE,
                    "commit restores staged torque state");
    fails += expect(packet_inst(&fake, 3) == DXL_INST_SYNC_WRITE && packet_param_u16(&fake, 3, 0) == XL330_ADDR_GOAL_POSITION,
                    "commit flushes staged position goals");
    fails += expect(ctl.mode_dirty == 0u && ctl.torque_dirty == 0u && ctl.goal_kind == XL330_GOAL_NONE,
                    "commit clears staged dirty state");
    return fails;
}

int main(void)
{
    int fails = 0;
    fails += test_public_header_surface();
    fails += test_controller_stages_until_commit();
    return fails ? 1 : 0;
}
