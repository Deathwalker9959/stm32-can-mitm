#ifndef __TESTS_H
#define __TESTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Number of frames transmitted per bus during the bus TX burst test. */
#define TEST_BUS_TX_FRAMES     10U

/** Interval between burst frames in milliseconds. */
#define TEST_BUS_TX_PERIOD_MS  20U

typedef enum
{
    TEST_NOT_RUN = 0,
    TEST_PASS    = 1,
    TEST_FAIL    = 2,
} TestResult;

typedef struct
{
    /* ---- Phase 1: loopback self-tests ---- */
    TestResult can1_loopback;
    TestResult can2_loopback;
    TestResult mcp2515_spi_regs;
    TestResult mcp2515_loopback;

    /* ---- Phase 2: live bus TX burst ---- */
    TestResult can1_bus_tx;
    TestResult can2_bus_tx;
    TestResult mcp2515_bus_tx;

    uint8_t all_passed;

    /* ---- MCP2515 SPI diagnostics ----
     * Inspect these when mcp2515_spi_regs == TEST_FAIL.
     * 0xFF on canstat/canctrl usually means MISO is floating (not wired).
     * 0x00 on canstat usually means CS is stuck high or chip not powered. */
    struct {
        uint8_t reset_ok;       /* 1 if mcp2515_reset() returned true              */
        uint8_t canstat;        /* CANSTAT read after reset  – expect 0x80         */
        uint8_t canctrl;        /* CANCTRL read after reset  – expect 0x87         */
        uint8_t cnf1_readback;  /* CNF1 read after writing 0xA5 – expect 0xA5      */
    } spi;

    /* ---- CAN bus TX diagnostics ----
     * If can2_start_ok == 0, CAN2 never started → nothing transmitted on it.
     * can1_frames / can2_frames count how many frames were actually queued. */
    struct {
        uint8_t can1_start_ok;   /* 1 if HAL_CAN_Start(&hcan1) returned HAL_OK  */
        uint8_t can2_start_ok;   /* 1 if HAL_CAN_Start(&hcan2) returned HAL_OK  */
        uint8_t can1_frames;     /* frames successfully queued on CAN1           */
        uint8_t can2_frames;     /* frames successfully queued on CAN2           */
        uint8_t mcp2515_frames;  /* frames where mcp2515_send_message() returned true */
    } bus_tx;

    /* ---- MCP2515 TX diagnostics (captured on first frame of bus_tx test) ----
     *
     * txb0ctrl_immed  — TXB0CTRL read immediately after mcp2515_send_message().
     *   bit 3 (TXREQ) = 1 → chip accepted the frame and is actively transmitting.
     *   bit 3 (TXREQ) = 0 → RTS command had no effect (SPI or CS issue).
     *   bits 6:5 (TXP)    → priority (0 = lowest).
     *   bit 4 (MLOA)      → 1 = lost arbitration (another node won).
     *   bit 1 (TXERR)     → 1 = bus error during TX (no ACK from any node).
     *
     * eflg_after_delay — EFLG read after TEST_BUS_TX_PERIOD_MS ms.
     *   bit 5 (TXBO)  = 1 → TX bus-off  (TEC > 255). Transceiver not driving bus.
     *   bit 4 (TXEP)  = 1 → TX error-passive (TEC >= 128). Frames still attempted.
     *   bit 3 (RXEP)  = 1 → RX error-passive.
     *   bit 2 (TXWAR) = 1 → TX warning (TEC >= 96).
     *   0x00 means no errors at all — chip received ACK from another node.
     *
     * tec_after_delay — Transmit Error Counter after the delay.
     *   0   → every frame was ACK'd (another node is on the bus).
     *   > 0 → counts failed TX attempts; 8 per failed frame.
     *
     * canstat_after_send — CANSTAT read immediately after send.
     *   bits 7:5 should be 0x00 (NORMAL mode).
     *   0x60 = listen-only, 0x40 = loopback — wrong mode, init failed.  */
    struct {
        uint8_t txb0ctrl_immed;    /* TXB0CTRL right after send — TXREQ must be 1  */
        uint8_t eflg_after_delay;  /* EFLG after delay   — 0x00 = no bus errors    */
        uint8_t tec_after_delay;   /* TEC  after delay   — 0x00 = all ACK'd        */
        uint8_t canstat_after_send;/* CANSTAT after send — bits 7:5 must be 0x00   */
    } mcp2515_tx_diag;

} TestSuite;

/**
 * Inspect this in the debugger Watch / Live Expressions window.
 * Set a breakpoint on the closing brace of Test_RunAll() to read results.
 */
extern TestSuite g_tests;

/**
 * @brief Run all hardware self-tests (loopback + bus TX burst).
 * Call after MX peripheral inits and before Gateway_CAN_Init().
 */
void Test_RunAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __TESTS_H */
