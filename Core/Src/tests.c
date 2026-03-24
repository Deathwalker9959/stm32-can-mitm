/**
 * @file tests.c
 * @brief Manual hardware-in-the-loop tests for CAN1, CAN2, and MCP2515 SPI.
 */

#include "tests.h"
#include "main.h"
#include "mcp2515.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

TestSuite g_tests = {0};

/* ------------------------------------------------------------------
 * Drain both RX FIFOs before stopping a CAN controller.
 * Prevents stale loopback frames from firing as interrupts once the
 * gateway enables notifications via Gateway_CAN_Start().
 * ------------------------------------------------------------------ */
static void can_drain_rx(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxh;
    uint8_t rxd[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        (void)HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxh, rxd);
    }
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0U)
    {
        (void)HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxh, rxd);
    }
}

/* ------------------------------------------------------------------
 * Pass-all mask filter.  hcan must be in READY state.
 * ------------------------------------------------------------------ */
static void can_filter_passall(CAN_HandleTypeDef *hcan,
                               uint32_t bank,
                               uint32_t fifo)
{
    CAN_FilterTypeDef f;
    f.FilterBank           = bank;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000U;
    f.FilterIdLow          = 0x0000U;
    f.FilterMaskIdHigh     = 0x0000U;
    f.FilterMaskIdLow      = 0x0000U;
    f.FilterFIFOAssignment = fifo;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14U;
    HAL_CAN_ConfigFilter(hcan, &f);
}

/* ------------------------------------------------------------------
 * Phase 1 – CAN loopback test
 * ------------------------------------------------------------------ */
static __attribute__((unused)) TestResult can_loopback_test(CAN_HandleTypeDef *hcan,
                                                            uint32_t filter_bank,
                                                            uint32_t test_id)
{
    CAN_TxHeaderTypeDef txh;
    CAN_RxHeaderTypeDef rxh;
    uint8_t    tx_data[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t    rx_data[8] = {0U};
    uint32_t   mailbox    = 0U;
    uint32_t   start_tick;
    TestResult result     = TEST_FAIL;

    can_filter_passall(hcan, filter_bank, CAN_RX_FIFO0);

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return TEST_FAIL;
    }

    txh.StdId              = test_id;
    txh.ExtId              = 0U;
    txh.IDE                = CAN_ID_STD;
    txh.RTR                = CAN_RTR_DATA;
    txh.DLC                = sizeof(tx_data);
    txh.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(hcan, &txh, tx_data, &mailbox) != HAL_OK)
    {
        can_drain_rx(hcan);
        HAL_CAN_Stop(hcan);
        return TEST_FAIL;
    }

    start_tick = HAL_GetTick();
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0U)
    {
        if ((HAL_GetTick() - start_tick) > 50U)
        {
            can_drain_rx(hcan);
            HAL_CAN_Stop(hcan);
            return TEST_FAIL;
        }
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxh, rx_data) == HAL_OK)
    {
        if ((rxh.StdId  == test_id)         &&
            (rxh.IDE    == CAN_ID_STD)       &&
            (rxh.RTR    == CAN_RTR_DATA)     &&
            (rxh.DLC    == sizeof(tx_data))  &&
            (rx_data[0] == tx_data[0])       &&
            (rx_data[1] == tx_data[1])       &&
            (rx_data[2] == tx_data[2])       &&
            (rx_data[3] == tx_data[3]))
        {
            result = TEST_PASS;
        }
    }

    can_drain_rx(hcan);
    HAL_CAN_Stop(hcan);
    return result;
}

/* ------------------------------------------------------------------
 * Phase 1 – MCP2515 SPI register sanity check
 *
 * Fills g_tests.spi with the actual register values so you can see
 * exactly what the chip (or nothing) returned.
 *
 * Typical failure signatures:
 *   canstat=0xFF, canctrl=0xFF  → MISO floating, SPI not connected
 *   canstat=0x00, canctrl=0x00  → CS stuck high or chip not powered
 *   canstat=0x80, canctrl!=0x87 → partial SPI issue
 * ------------------------------------------------------------------ */
static TestResult mcp2515_spi_register_test(void)
{
    g_tests.spi.reset_ok      = 0U;
    g_tests.spi.canstat       = 0xFFU;
    g_tests.spi.canctrl       = 0xFFU;
    g_tests.spi.cnf1_readback = 0xFFU;

    if (!mcp2515_reset())
    {
        return TEST_FAIL;
    }
    g_tests.spi.reset_ok = 1U;

    /* Actively poll until chip acknowledges config mode (OPMOD[2:0]=100)
     * or timeout — avoids relying solely on the reset delay being long enough */
    if (!mcp2515_set_mode(MCP2515_MODE_CONFIG))
    {
        g_tests.spi.canstat = mcp2515_read_register(0x0EU);
        return TEST_FAIL;
    }

    g_tests.spi.canstat = mcp2515_read_register(0x0EU);
    if ((g_tests.spi.canstat & 0xE0U) != 0x80U)
    {
        return TEST_FAIL;
    }

    /* CANCTRL[7:5]=100 (config mode) is required; lower bits vary by crystal
     * config so only verify the mode field, not the full 0x87 value */
    g_tests.spi.canctrl = mcp2515_read_register(0x0FU);
    if ((g_tests.spi.canctrl & 0xE0U) != 0x80U)
    {
        return TEST_FAIL;
    }

    if (!mcp2515_write_register(0x2AU, 0xA5U))
    {
        return TEST_FAIL;
    }
    g_tests.spi.cnf1_readback = mcp2515_read_register(0x2AU);
    if (g_tests.spi.cnf1_readback != 0xA5U)
    {
        return TEST_FAIL;
    }

    return TEST_PASS;
}

/* ------------------------------------------------------------------
 * Phase 1 – MCP2515 internal loopback test
 * ------------------------------------------------------------------ */
static TestResult mcp2515_loopback_test(void)
{
    mcp2515_frame_t tx_frame;
    mcp2515_frame_t rx_frame;
    uint32_t start_tick;

    if (!mcp2515_init(CAN_250KBPS))
    {
        return TEST_FAIL;
    }

    if (!mcp2515_set_mode(MCP2515_MODE_LOOPBACK))
    {
        return TEST_FAIL;
    }

    tx_frame.id       = 0x333U;
    tx_frame.dlc      = 4U;
    tx_frame.extended = 0U;
    tx_frame.rtr      = 0U;
    tx_frame.data[0]  = 0x11U;
    tx_frame.data[1]  = 0x22U;
    tx_frame.data[2]  = 0x33U;
    tx_frame.data[3]  = 0x44U;

    if (!mcp2515_send_message(&tx_frame))
    {
        return TEST_FAIL;
    }

    start_tick = HAL_GetTick();
    while (!mcp2515_irq_pending())
    {
        if ((HAL_GetTick() - start_tick) > 50U)
        {
            return TEST_FAIL;
        }
    }

    if (!mcp2515_receive_message(&rx_frame))
    {
        return TEST_FAIL;
    }

    if ((rx_frame.id      == tx_frame.id)      &&
        (rx_frame.dlc     == tx_frame.dlc)      &&
        (rx_frame.data[0] == tx_frame.data[0])  &&
        (rx_frame.data[1] == tx_frame.data[1])  &&
        (rx_frame.data[2] == tx_frame.data[2])  &&
        (rx_frame.data[3] == tx_frame.data[3]))
    {
        return TEST_PASS;
    }

    return TEST_FAIL;
}

/* ------------------------------------------------------------------
 * Phase 2 – CAN bus TX burst
 *
 * start_ok and frames_sent are written directly to g_tests.bus_tx
 * so the debugger can see if CAN failed to start or sent 0 frames.
 * ------------------------------------------------------------------ */
static __attribute__((unused)) TestResult can_bus_tx_test(CAN_HandleTypeDef *hcan,
                                                          uint32_t filter_bank,
                                                          uint32_t tx_id,
                                                          uint8_t *start_ok_out,
                                                          uint8_t *frames_out)
{
    CAN_TxHeaderTypeDef txh;
    uint8_t    tx_data[8] = {0x00U, 0x11U, 0x22U, 0x33U,
                              0x44U, 0x55U, 0x66U, 0x77U};
    uint32_t   mailbox    = 0U;
    uint32_t   start_tick;
    uint8_t    i;
    TestResult result     = TEST_PASS;

    *start_ok_out = 0U;
    *frames_out   = 0U;

    can_filter_passall(hcan, filter_bank, CAN_RX_FIFO0);

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return TEST_FAIL;
    }
    *start_ok_out = 1U;

    txh.StdId              = tx_id;
    txh.ExtId              = 0U;
    txh.IDE                = CAN_ID_STD;
    txh.RTR                = CAN_RTR_DATA;
    txh.DLC                = sizeof(tx_data);
    txh.TransmitGlobalTime = DISABLE;

    for (i = 0U; i < TEST_BUS_TX_FRAMES; ++i)
    {
        /* Drain any loopback copies from the RX FIFO before each send so the
         * FIFO never fills up.  A full FIFO can set the FOVR overrun flag which
         * on some silicon revisions stalls the TX state machine. */
        can_drain_rx(hcan);

        /* Wait until at least one TX mailbox is free (allow up to 3 in flight). */
        start_tick = HAL_GetTick();
        while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
        {
            if ((HAL_GetTick() - start_tick) > 100U)
            {
                result = TEST_FAIL;
                goto stop;
            }
        }

        tx_data[0] = i;

        if (HAL_CAN_AddTxMessage(hcan, &txh, tx_data, &mailbox) != HAL_OK)
        {
            result = TEST_FAIL;
            break;
        }
        (*frames_out)++;

        HAL_Delay(TEST_BUS_TX_PERIOD_MS);
    }

stop:
    can_drain_rx(hcan);
    HAL_CAN_Stop(hcan);
    return result;
}

/* ------------------------------------------------------------------
 * Phase 2 – MCP2515 bus TX burst (normal mode, logger provides ACK)
 *
 * mcp2515_init() is called before every frame.  This guarantees a
 * completely clean chip state (TEC=0, no bus-off, all TX buffers free)
 * for each send regardless of whether the previous frame received ACK.
 * Without a re-init, three failed transmissions fill all TX buffers and
 * subsequent sends return false.
 * ------------------------------------------------------------------ */
static __attribute__((unused)) TestResult mcp2515_bus_tx_test(void)
{
    mcp2515_frame_t frame;
    uint8_t i;

    frame.id       = 0x333U;
    frame.dlc      = 8U;
    frame.extended = 0U;
    frame.rtr      = 0U;
    frame.data[1]  = 0x11U;
    frame.data[2]  = 0x22U;
    frame.data[3]  = 0x33U;
    frame.data[4]  = 0x44U;
    frame.data[5]  = 0x55U;
    frame.data[6]  = 0x66U;
    frame.data[7]  = 0x77U;

    g_tests.bus_tx.mcp2515_frames = 0U;

    for (i = 0U; i < TEST_BUS_TX_FRAMES; ++i)
    {
        /* Re-init before every frame: clears TEC/REC, exits bus-off,
         * frees all TX buffers — no ABAT polling needed. */
        if (!mcp2515_init(CAN_250KBPS))
        {
            return TEST_FAIL;
        }

        frame.data[0] = i;

        if (!mcp2515_send_message(&frame))
        {
            return TEST_FAIL;
        }

        g_tests.bus_tx.mcp2515_frames++;

        /* Capture diagnostics on the first frame (chip freshly initialised —
         * cleanest state for evaluating the TX path). */
        if (i == 0U)
        {
            /* TXB0CTRL immediately after send: TXREQ (bit 3) must be 1.
             * If it is 0 the RTS command had no effect (SPI or CS problem). */
            g_tests.mcp2515_tx_diag.txb0ctrl_immed     = mcp2515_read_register(0x30U);
            g_tests.mcp2515_tx_diag.canstat_after_send  = mcp2515_read_register(0x0EU);
        }

        /* Inter-frame gap: frame is on the bus during this window.
         * The re-init at the top of the next iteration resets the chip,
         * aborting any pending retry automatically. */
        HAL_Delay(TEST_BUS_TX_PERIOD_MS);

        if (i == 0U)
        {
            /* Error state after the delay — chip has had time to attempt TX
             * and accumulate errors if no ACK was received.
             * eflg == 0x00 means another node provided ACK (healthy bus). */
            g_tests.mcp2515_tx_diag.eflg_after_delay = mcp2515_read_register(0x2DU);
            g_tests.mcp2515_tx_diag.tec_after_delay  = mcp2515_read_register(0x1CU);
        }
    }

    return TEST_PASS;
}

/* ------------------------------------------------------------------
 * Public entry point
 * ------------------------------------------------------------------ */
void Test_RunAll(void)
{
    /* Phase 1 – CAN loopback tests disabled: these start/stop CAN1 and CAN2
     * which creates a ~200ms gap where the gateway is not running.  Any
     * external CAN tool connected during that window will see TX-fail because
     * nobody is ACKing.  The tests also always return TEST_FAIL in CAN_MODE_NORMAL
     * (no internal loopback), so they provide no value during normal operation.
     * Re-enable for standalone hardware bring-up without external tools attached. */
    g_tests.can1_loopback = TEST_NOT_RUN;
    g_tests.can2_loopback = TEST_NOT_RUN;

    g_tests.mcp2515_spi_regs = mcp2515_spi_register_test();
    g_tests.mcp2515_loopback = mcp2515_loopback_test();

    /* Phase 2 – bus TX burst disabled: these tests stop/start CAN1 and CAN2
     * which causes TX failures on any external CAN debugger connected during
     * boot.  Re-enable when doing standalone hardware bring-up without a
     * live debugger attached. */
    g_tests.can1_bus_tx    = TEST_NOT_RUN;
    g_tests.can2_bus_tx    = TEST_NOT_RUN;
    g_tests.mcp2515_bus_tx = TEST_NOT_RUN;

    g_tests.all_passed = (
        (g_tests.can1_loopback    == TEST_PASS) &&
        (g_tests.can2_loopback    == TEST_PASS) &&
        (g_tests.mcp2515_spi_regs == TEST_PASS) &&
        (g_tests.mcp2515_loopback == TEST_PASS)
    ) ? 1U : 0U;

    /* <<< SET DEBUGGER BREAKPOINT HERE, inspect g_tests >>> */
      }
