#include "dxl/dxl_protocol2.h"
#include <stdio.h>
#include <string.h>

static int expect_packet(const char *name, uint8_t id, uint8_t inst, const uint8_t *params, uint16_t plen,
                         const uint8_t *expected, uint16_t expected_len)
{
    uint8_t out[DXL_MAX_PACKET_SIZE];
    uint16_t out_len = 0;
    DXL2_Status st = dxl2_build_packet(out, sizeof(out), id, inst, params, plen, &out_len);
    if (st != DXL2_OK || out_len != expected_len || memcmp(out, expected, expected_len) != 0) {
        printf("FAIL %s st=%d len=%u expected=%u\n", name, st, out_len, expected_len);
        return 1;
    }
    printf("PASS %s\n", name);
    return 0;
}

static int test_parser(void)
{
    const uint8_t raw[] = {0xFF,0xFF,0xFD,0x00,0x01,0x07,0x00,0x55,0x00,0x06,0x04,0x26,0x65,0x5D};
    DXL2_StatusPacket pkt;
    DXL2_Status st = dxl2_parse_status_packet(raw, sizeof(raw), 1, true, &pkt);
    if (st != DXL2_OK || pkt.id != 1 || pkt.error != 0 || pkt.param_len != 3 || pkt.params[0] != 0x06 || pkt.params[1] != 0x04 || pkt.params[2] != 0x26) {
        printf("FAIL parser st=%d id=%u plen=%u\n", st, pkt.id, pkt.param_len);
        return 1;
    }
    printf("PASS parser\n");
    return 0;
}

static int test_stuffing(void)
{
    const uint8_t src[] = {0x01,0xFF,0xFF,0xFD,0x02};
    uint8_t stuffed[16], clean[16];
    uint16_t slen = 0, clen = 0;
    if (dxl2_byte_stuff(src, sizeof(src), stuffed, sizeof(stuffed), &slen) != DXL2_OK) return 1;
    if (slen != 6 || stuffed[4] != 0xFD) { printf("FAIL stuffing\n"); return 1; }
    if (dxl2_destuff(stuffed, slen, clean, sizeof(clean), &clen) != DXL2_OK) return 1;
    if (clen != sizeof(src) || memcmp(src, clean, sizeof(src)) != 0) { printf("FAIL destuffing\n"); return 1; }
    printf("PASS stuffing\n");
    return 0;
}

int main(void)
{
    int fails = 0;
    const uint8_t ping[] = {0xFF,0xFF,0xFD,0x00,0x01,0x03,0x00,0x01,0x19,0x4E};
    fails += expect_packet("ping", 1, DXL_INST_PING, NULL, 0, ping, sizeof(ping));

    const uint8_t read_params[] = {0x84,0x00,0x04,0x00};
    const uint8_t read_pkt[] = {0xFF,0xFF,0xFD,0x00,0x01,0x07,0x00,0x02,0x84,0x00,0x04,0x00,0x1D,0x15};
    fails += expect_packet("read", 1, DXL_INST_READ, read_params, sizeof(read_params), read_pkt, sizeof(read_pkt));

    const uint8_t write_params[] = {0x74,0x00,0x00,0x02,0x00,0x00};
    const uint8_t write_pkt[] = {0xFF,0xFF,0xFD,0x00,0x01,0x09,0x00,0x03,0x74,0x00,0x00,0x02,0x00,0x00,0xCA,0x89};
    fails += expect_packet("write", 1, DXL_INST_WRITE, write_params, sizeof(write_params), write_pkt, sizeof(write_pkt));

    const uint8_t sync_read_params[] = {0x84,0x00,0x04,0x00,0x01,0x02};
    const uint8_t sync_read_pkt[] = {0xFF,0xFF,0xFD,0x00,0xFE,0x09,0x00,0x82,0x84,0x00,0x04,0x00,0x01,0x02,0xCE,0xFA};
    fails += expect_packet("sync_read", DXL_BROADCAST_ID, DXL_INST_SYNC_READ, sync_read_params, sizeof(sync_read_params), sync_read_pkt, sizeof(sync_read_pkt));

    const uint8_t sync_write_params[] = {0x74,0x00,0x04,0x00,0x01,0x96,0x00,0x00,0x00,0x02,0xAA,0x00,0x00,0x00};
    const uint8_t sync_write_pkt[] = {0xFF,0xFF,0xFD,0x00,0xFE,0x11,0x00,0x83,0x74,0x00,0x04,0x00,0x01,0x96,0x00,0x00,0x00,0x02,0xAA,0x00,0x00,0x00,0x82,0x87};
    fails += expect_packet("sync_write", DXL_BROADCAST_ID, DXL_INST_SYNC_WRITE, sync_write_params, sizeof(sync_write_params), sync_write_pkt, sizeof(sync_write_pkt));

    fails += test_parser();
    fails += test_stuffing();
    return fails ? 1 : 0;
}
