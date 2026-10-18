/* Copyright 2025 muzkr https://github.com/muzkr
 * Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include <string.h>

#if !defined(ENABLE_OVERLAY)
#include "py32f0xx.h"
#endif
#ifdef ENABLE_FMRADIO_EMBEDDED
#include "app/fm.h"
#endif
#if defined(ENABLE_DTRAC)
#include "app/app.h"        //for DTrac
#include "driver/systick.h" //for DTrac
#endif
#include "app/uart.h"
#ifdef ENABLE_AIRCOPY_UART
#include "app/aircopy.h"
#endif
#include "board.h"
#include "py32f071_ll_dma.h"
#include "driver/backlight.h"
#include "driver/bk4819.h"
#include "driver/crc.h"
#include "driver/eeprom.h"
#include "driver/gpio.h"

#if defined(ENABLE_UART)
#include "driver/uart.h"
#endif

#if defined(ENABLE_USB)
#include "driver/vcp.h"
#endif

#include "functions.h"
#include "misc.h"
#include "settings.h"
#include "version.h"

#ifdef ENABLE_FEAT_F4HWN_MULTIBOOT
#include "driver/mb_flash.h"
#ifdef ENABLE_FEAT_F4HWN_EXT_FLASH_RW
#include "driver/py25q16.h"
#endif
#endif

#ifdef ENABLE_FEAT_F4HWN_OVERLAY_APPS
#include "apps/app_overlay.h"
#endif

#if defined(ENABLE_OVERLAY)
#include "sram-overlay.h"
#endif

#define UNUSED(x) (void)(x)

#define DMA_INDEX(x, y, z) (((x) + (y)) % (z))

#if defined(ENABLE_UART)
#define DMA_CHANNEL LL_DMA_CHANNEL_2
#endif

// !! Make sure this is correct!
#define MAX_REPLY_SIZE 144

#ifdef ENABLE_AIRCOPY_UART
#define UART_CMD_AIRCOPY 0x0740u
#endif

typedef struct
{
    uint16_t ID;
    uint16_t Size;
} Header_t;

typedef struct
{
    uint8_t Padding[2];
    uint16_t ID;
} Footer_t;

#if defined(ENABLE_DTRAC)

// for DTrac app RX CTCSS/DCS
typedef struct
{
    Header_t Header;
    uint32_t CodeValue; // little-endian:
                        // 99    = off
                        // 0..49 = CTCSS index
                        // else  = DCS Golay code word
} CMD_9999_t;

// for DTrac app TX CTCSS/DCS
typedef struct
{
    Header_t Header;
    uint32_t CodeValue; // little-endian:
                        // 99    = off
                        // 0..49 = CTCSS index
                        // else  = DCS Golay code word
} CMD_9998_t;

// for DTrac app RX Freq
typedef struct
{
    Header_t Header;
    uint32_t DownFrequency;
} CMD_9997_t;

// for DTrac app TX Freq
typedef struct
{
    Header_t Header;
    uint32_t UpFrequency;
} CMD_9996_t;

// for DTrac app mode
typedef struct
{
    Header_t Header;
    char Mode;
} CMD_9995_t;

// for DTrac app MonitorStatus
typedef struct
{
    Header_t Header;
    char MonitorStatus;
} CMD_9993_t;

#endif

typedef struct
{
    Header_t Header;
    uint32_t Timestamp;
} CMD_0514_t;

typedef struct
{
    Header_t Header;
    struct
    {
        char Version[16];
        bool bHasCustomAesKey;
        bool bIsInLockScreen;
        uint8_t Padding[2];
        uint32_t Challenge[4];
    } Data;
} REPLY_0514_t;

typedef struct
{
    Header_t Header;
    uint16_t Offset;
    uint8_t Size;
    uint8_t Padding;
    uint32_t Timestamp;
} CMD_051B_t;

typedef struct
{
    Header_t Header;
    struct
    {
        uint16_t Offset;
        uint8_t Size;
        uint8_t Padding;
        uint8_t Data[128];
    } Data;
} REPLY_051B_t;

typedef struct
{
    Header_t Header;
    uint16_t Offset;
    uint8_t Size;
    bool bAllowPassword;
    uint32_t Timestamp;
    uint8_t Data[0];
} CMD_051D_t;

typedef struct
{
    Header_t Header;
    struct
    {
        uint16_t Offset;
    } Data;
} REPLY_051D_t;

#ifdef ENABLE_EXTRA_UART_CMD
typedef struct
{
    Header_t Header;
    struct
    {
        uint16_t RSSI;
        uint8_t ExNoiseIndicator;
        uint8_t GlitchIndicator;
    } Data;
} REPLY_0527_t;

typedef struct
{
    Header_t Header;
    struct
    {
        uint16_t Voltage;
        uint16_t Current;
    } Data;
} REPLY_0529_t;

typedef struct
{
    Header_t Header;
    uint32_t Response[4];
} CMD_052D_t;
#endif

typedef struct
{
    Header_t Header;
    struct
    {
        bool bIsLocked;
        uint8_t Padding[3];
    } Data;
} REPLY_052D_t;

#ifdef ENABLE_EXTRA_UART_CMD
typedef struct
{
    Header_t Header;
    uint32_t Timestamp;
} CMD_052F_t;
#endif

static const uint8_t Obfuscation[16] =
    {
        0x16, 0x6C, 0x14, 0xE6, 0x2E, 0x91, 0x0D, 0x40, 0x21, 0x35, 0xD5, 0x40, 0x13, 0x03, 0xE9, 0x80};

typedef union
{
    uint8_t Buffer[256];
    struct
    {
        Header_t Header;
        uint8_t Data[252];
    };
} UART_Command_t __attribute__((aligned(4)));

#if defined(ENABLE_UART)
static uint32_t UART_Timestamp;
static UART_Command_t UART_Command;
static uint16_t gUART_WriteIndex;
#endif
#if defined(ENABLE_USB)
static uint32_t VCP_Timestamp;
static UART_Command_t VCP_Command;
static uint16_t VCP_ReadIndex;
#endif

// static bool     bIsEncrypted = true;
#define bIsEncrypted true

#ifdef ENABLE_USB
static void SendReply_VCP(void *pReply, uint16_t Size)
{
    static uint8_t VCP_ReplyBuf[MAX_REPLY_SIZE + sizeof(Header_t) + sizeof(Footer_t)]
        __attribute__((aligned(4)));

    // !!
    if (Size > MAX_REPLY_SIZE)
    {
        return;
    }

    uint8_t *pBody = VCP_ReplyBuf + sizeof(Header_t);
    uint8_t *pFooter = pBody + Size;

    memcpy(pBody, pReply, Size);
    pReply = pBody;

    if (bIsEncrypted)
    {
        uint8_t *pBytes = (uint8_t *)pReply;
        unsigned int i;
        for (i = 0; i < Size; i++)
            pBytes[i] ^= Obfuscation[i % 16];
    }

    /* Build the transport header/footer byte by byte. The reply body may have
     * an odd size, so pFooter is not necessarily half-word aligned; casting it
     * to Footer_t and storing ID as uint16_t can HardFault on Cortex-M0+. */
    VCP_ReplyBuf[0] = 0xAB;
    VCP_ReplyBuf[1] = 0xCD;
    VCP_ReplyBuf[2] = (uint8_t)(Size & 0xFFu);
    VCP_ReplyBuf[3] = (uint8_t)(Size >> 8);

    // VCP_Send((uint8_t *)&Header, sizeof(Header));
    // VCP_Send(pReply, Size);

    if (bIsEncrypted)
    {
        pFooter[0] = Obfuscation[(Size + 0) % 16] ^ 0xFF;
        pFooter[1] = Obfuscation[(Size + 1) % 16] ^ 0xFF;
    }
    else
    {
        pFooter[0] = 0xFF;
        pFooter[1] = 0xFF;
    }
    pFooter[2] = 0xDC;
    pFooter[3] = 0xBA;

    // VCP_Send((uint8_t *)&Footer, sizeof(Footer));

    VCP_SendAsync(VCP_ReplyBuf, sizeof(Header_t) + Size + sizeof(Footer_t));
}
#endif // ENABLE_USB

static void SendReply(uint32_t Port, void *pReply, uint16_t Size)
{
#if defined(ENABLE_USB)
    if (Port == UART_PORT_VCP)
    {
        SendReply_VCP(pReply, Size);
        return;
    }
#endif

#if defined(ENABLE_UART)
    Header_t Header;
    Footer_t Footer;

    if (bIsEncrypted)
    {
        uint8_t *pBytes = (uint8_t *)pReply;
        unsigned int i;
        for (i = 0; i < Size; i++)
            pBytes[i] ^= Obfuscation[i % 16];
    }

    Header.ID = 0xCDAB;
    Header.Size = Size;

    UART_Send(&Header, sizeof(Header));
    UART_Send(pReply, Size);

    if (bIsEncrypted)
    {
        Footer.Padding[0] = Obfuscation[(Size + 0) % 16] ^ 0xFF;
        Footer.Padding[1] = Obfuscation[(Size + 1) % 16] ^ 0xFF;
    }
    else
    {
        Footer.Padding[0] = 0xFF;
        Footer.Padding[1] = 0xFF;
    }
    Footer.ID = 0xBADC;

    UART_Send(&Footer, sizeof(Footer));
#endif
}

#ifdef ENABLE_AIRCOPY_UART
void UART_SendAircopy(const uint16_t *data, uint8_t words)
{
    static union
    {
        uint8_t Bytes[sizeof(Header_t) + 2u +
                      AIRCOPY_FRAME_WORDS_MAX * sizeof(uint16_t) + sizeof(uint16_t)];
        struct __attribute__((packed, aligned(4)))
        {
            Header_t Header;
            uint8_t Words;
            uint8_t Reserved;
            uint16_t Data[AIRCOPY_FRAME_WORDS_MAX];
        } Packet;
    } Frame;
    Header_t transportHeader;
    const uint16_t transportFooter = 0xBADCu;

    if (words == 0u || words > AIRCOPY_FRAME_WORDS_MAX)
        return;

    Frame.Packet.Header.ID = UART_CMD_AIRCOPY;
    Frame.Packet.Header.Size = (uint16_t)(2u + words * sizeof(Frame.Packet.Data[0]));
    Frame.Packet.Words = words;
    Frame.Packet.Reserved = 0;
    memcpy(Frame.Packet.Data, data, words * sizeof(Frame.Packet.Data[0]));

    const uint16_t bodySize = (uint16_t)(sizeof(Frame.Packet.Header) +
                                         Frame.Packet.Header.Size);
    const uint16_t crc = CRC_Calculate(Frame.Bytes, bodySize);
    Frame.Bytes[bodySize] = (uint8_t)crc;
    Frame.Bytes[bodySize + 1u] = (uint8_t)(crc >> 8);

    // Peer radios feed this packet back through UART_IsCommandAvailable(), so
    // unlike a PC reply it must carry a real CRC instead of the 0xFFFF marker.
    for (uint16_t i = 0; i < bodySize + sizeof(crc); i++)
        Frame.Bytes[i] ^= Obfuscation[i % 16u];

    transportHeader.ID = 0xCDABu;
    transportHeader.Size = bodySize;
    UART_Send(&transportHeader, sizeof(transportHeader));
    UART_Send(Frame.Bytes, bodySize + sizeof(crc));
    UART_Send(&transportFooter, sizeof(transportFooter));
}
#endif

static void SendVersion(uint32_t Port)
{
    REPLY_0514_t Reply;

    Reply.Data.Padding[0] = Reply.Data.Padding[1] = 0;
    Reply.Header.ID = 0x0515;
    Reply.Header.Size = sizeof(Reply.Data);
    strncpy(Reply.Data.Version, Version, sizeof(Reply.Data.Version));
    Reply.Data.bHasCustomAesKey = bHasCustomAesKey;
    Reply.Data.bIsInLockScreen = bIsInLockScreen;
    Reply.Data.Challenge[0] = gChallenge[0];
    Reply.Data.Challenge[1] = gChallenge[1];
    Reply.Data.Challenge[2] = gChallenge[2];
    Reply.Data.Challenge[3] = gChallenge[3];

    SendReply(Port, &Reply, sizeof(Reply));
}

#ifndef ENABLE_FEAT_F4HWN
static bool IsBadChallenge(const uint32_t *pKey, const uint32_t *pIn, const uint32_t *pResponse)
{
    // PY32 has no AES hardware
    /*
    unsigned int i;
    uint32_t     IV[4];

    IV[0] = 0;
    IV[1] = 0;
    IV[2] = 0;
    IV[3] = 0;

    AES_Encrypt(pKey, IV, pIn, IV, true);

    for (i = 0; i < 4; i++)
        if (IV[i] != pResponse[i])
            return true;
    */

    return false;
}
#endif

// session init, sends back version info and state
// timestamp is a session id really
static void CMD_0514(uint32_t Port, const uint8_t *pBuffer)
{
    const CMD_0514_t *pCmd = (const CMD_0514_t *)pBuffer;

    if (0)
    {
    }
#if defined(ENABLE_UART)
    else if (Port == UART_PORT_UART)
    {
        UART_Timestamp = pCmd->Timestamp;
    }
#endif
#if defined(ENABLE_USB)
    else if (Port == UART_PORT_VCP)
    {
        VCP_Timestamp = pCmd->Timestamp;
    }
#endif

#ifdef ENABLE_FMRADIO_EMBEDDED
    gFmRadioCountdown_500ms = fm_radio_countdown_500ms;
#endif

    gSerialConfigCountDown_500ms = 12; // 6 sec

    // Backlight left untouched: a serial session is neutral, so the normal BLTime
    // inactivity countdown keeps running from the last keypress (no forced turn-off).

    SendVersion(Port);
}

// read eeprom
static void CMD_051B(uint32_t Port, const uint8_t *pBuffer)
{
    const CMD_051B_t *pCmd = (const CMD_051B_t *)pBuffer;
    REPLY_051B_t Reply;
    bool bLocked = false;

    uint32_t Timestamp = 0;

    if (0)
    {
    }
#if defined(ENABLE_UART)
    else if (Port == UART_PORT_UART)
    {
        Timestamp = UART_Timestamp;
    }
#endif
#if defined(ENABLE_USB)
    else if (Port == UART_PORT_VCP)
    {
        Timestamp = VCP_Timestamp;
    }
#endif
    else
    {
        return;
    }

    if (pCmd->Timestamp != Timestamp)
        return;

    gSerialConfigCountDown_500ms = 12; // 6 sec

#ifdef ENABLE_FMRADIO_EMBEDDED
    gFmRadioCountdown_500ms = fm_radio_countdown_500ms;
#endif

    // Reject reads that do not fit in the fixed-size reply buffer.
    if (pCmd->Size > sizeof(Reply.Data.Data))
        return;

    memset(&Reply, 0, sizeof(Reply));
    Reply.Header.ID = 0x051C;
    Reply.Header.Size = pCmd->Size + 4;
    Reply.Data.Offset = pCmd->Offset;
    Reply.Data.Size = pCmd->Size;

    if (bHasCustomAesKey)
        bLocked = gIsLocked;

    if (!bLocked)
    {
        EEPROM_ReadBuffer(pCmd->Offset, Reply.Data.Data, pCmd->Size);
    }

    SendReply(Port, &Reply, pCmd->Size + 8);
}

// write eeprom
static void CMD_051D(uint32_t Port, const uint8_t *pBuffer)
{
    const CMD_051D_t *pCmd = (const CMD_051D_t *)pBuffer;
    REPLY_051D_t Reply;
    bool bReloadEeprom;
    bool bIsLocked;

    uint32_t Timestamp = 0;

    /* Bound the write against the received frame: Data[] must hold pCmd->Size
     * bytes, otherwise the loop below would read adjacent RAM and persist it to
     * EEPROM (memory disclosure). A non-multiple-of-8 Size is NOT rejected: the
     * Size/8 loop simply truncates the sub-page tail, matching the historical
     * behavior CHIRP relies on for its final (unaligned) config block. */
    if (pCmd->Header.Size < 8u + pCmd->Size)
        return;

    if (0)
    {
    }
#if defined(ENABLE_UART)
    else if (Port == UART_PORT_UART)
    {
        Timestamp = UART_Timestamp;
    }
#endif
#if defined(ENABLE_USB)
    else if (Port == UART_PORT_VCP)
    {
        Timestamp = VCP_Timestamp;
    }
#endif
    else
    {
        return;
    }

    if (pCmd->Timestamp != Timestamp)
        return;

    gSerialConfigCountDown_500ms = 12; // 6 sec

    bReloadEeprom = false;

#ifdef ENABLE_FMRADIO_EMBEDDED
    gFmRadioCountdown_500ms = fm_radio_countdown_500ms;
#endif

    Reply.Header.ID = 0x051E;
    Reply.Header.Size = sizeof(Reply.Data);
    Reply.Data.Offset = pCmd->Offset;

    bIsLocked = bHasCustomAesKey ? gIsLocked : false;

    if (!bIsLocked)
    {
        unsigned int i;
        for (i = 0; i < (pCmd->Size / 8); i++)
        {
            const uint16_t Offset = pCmd->Offset + (i * 8U);

            if (Offset >= 0x0F30 && Offset < 0x0F40)
                if (!gIsLocked)
                    bReloadEeprom = true;

            if ((Offset < 0x0E98 || Offset >= 0x0EA0) || !bIsInLockScreen || pCmd->bAllowPassword)
            {
                EEPROM_WriteBuffer(Offset, &pCmd->Data[i * 8U], 8);
            }
        }

        if (bReloadEeprom)
#ifdef ENABLE_FEAT_F4HWN_MULTIBOOT_HOT_CFG
            SETTINGS_InitEEPROM(false);
#else
            SETTINGS_InitEEPROM();
#endif
    }

    SendReply(Port, &Reply, sizeof(Reply));
}

#if defined(ENABLE_DTRAC)

// for DTrac app
// Reverse-lookup a DCS Golay code word to its option index and direction.
// Returns 0xFF if the code word does not match any known DCS option.
static uint8_t DCS_GetOptionFromGolay(uint32_t code, DCS_CodeType_t *pCodeType)
{
    for (uint8_t i = 0; i < DCS_OPTION_COUNT; i++)
    {
        if (DCS_GetGolayCodeWord(CODE_TYPE_DIGITAL, i) == code)
        {
            *pCodeType = CODE_TYPE_DIGITAL;
            return i;
        }

        if (DCS_GetGolayCodeWord(CODE_TYPE_REVERSE_DIGITAL, i) == code)
        {
            *pCodeType = CODE_TYPE_REVERSE_DIGITAL;
            return i;
        }
    }

    return 0xFF;
}

// for DTrac app RX CTCSS/DCS
static void CMD_9999(const uint8_t *pBuffer)
{
    const CMD_9999_t *pCmd = (const CMD_9999_t *)pBuffer;
    uint32_t codeValue = pCmd->CodeValue;
    bool changed = false;

    if (pCmd->Header.Size < sizeof(uint32_t))
        return;

    // 关闭接收亚音
    if (codeValue == 99)
    {
        if (gRxVfo->pRX->CodeType != CODE_TYPE_OFF ||
            gRxVfo->pRX->Code != 99)
        {
            gRxVfo->pRX->CodeType = CODE_TYPE_OFF;
            gRxVfo->pRX->Code = 99;
            changed = true;
        }
    }
    // CTCSS：0..49
    else if (codeValue < ARRAY_SIZE(CTCSS_Options))
    {
        uint8_t ctcssCode = (uint8_t)codeValue;

        if (gRxVfo->pRX->CodeType != CODE_TYPE_CONTINUOUS_TONE ||
            gRxVfo->pRX->Code != ctcssCode)
        {
            gRxVfo->pRX->CodeType = CODE_TYPE_CONTINUOUS_TONE;
            gRxVfo->pRX->Code = ctcssCode;
            changed = true;
        }
    }
    // DCS Golay 码字
    else
    {
        DCS_CodeType_t codeType = CODE_TYPE_DIGITAL;
        uint8_t dcsOption = DCS_GetOptionFromGolay(codeValue, &codeType);

        if (dcsOption != 0xFF)
        {
            if (gRxVfo->pRX->CodeType != codeType ||
                gRxVfo->pRX->Code != dcsOption)
            {
                gRxVfo->pRX->CodeType = codeType;
                gRxVfo->pRX->Code = dcsOption;
                changed = true;
            }
        }
    }

    if (changed)
    {
        /*
         * 不要调用 BK4819_ExitSubAu()。
         * 该函数可能同时关掉 TX 亚音，导致 0x9999 设置的发射亚音失效。
         *
         * 让固件根据 pRX 重新配置 BK4819 的接收亚音检测使能/参数。
         * 如果当前正在 PTT，建议不要重配，等释放后由正常流程应用。
         */
        if (!gPttIsPressed)
        {
            RADIO_SetupRegisters(true);
            // 如果当前处于 MONITOR，可能需要恢复：
            // if (gMonitor) APP_StartListening(FUNCTION_MONITOR);
            // else          APP_StartListening(FUNCTION_RECEIVE);
        }

        gUpdateDisplay = true;
    }
}

// for DTrac app TX CTCSS/DCS
static void CMD_9998(const uint8_t *pBuffer)
{
    const CMD_9998_t *pCmd = (const CMD_9998_t *)pBuffer;
    uint32_t codeValue = pCmd->CodeValue;

    // 关闭亚音：APP 发送 99
    if (codeValue == 99)
    {
        if (gRxVfo->pTX->CodeType != CODE_TYPE_OFF ||
            gRxVfo->pTX->Code != 99)
        {
            gRxVfo->pTX->CodeType = CODE_TYPE_OFF;
            gRxVfo->pTX->Code = 99;
            BK4819_ExitSubAu();
        }
        return;
    }

    // CTCSS：APP 发送的是 CTCSS_Options 的索引 0..49
    if (codeValue < ARRAY_SIZE(CTCSS_Options))
    {
        uint8_t ctcssCode = (uint8_t)codeValue;

        if (gRxVfo->pTX->CodeType != CODE_TYPE_CONTINUOUS_TONE ||
            gRxVfo->pTX->Code != ctcssCode)
        {
            gRxVfo->pTX->CodeType = CODE_TYPE_CONTINUOUS_TONE;
            gRxVfo->pTX->Code = ctcssCode;
            BK4819_SetCTCSSFrequency(CTCSS_Options[ctcssCode]);
        }
        return;
    }

    // DCS：APP 发送的是已经算好的 Golay 码字
    {
        DCS_CodeType_t codeType = CODE_TYPE_DIGITAL;
        uint8_t dcsOption = DCS_GetOptionFromGolay(codeValue, &codeType);

        if (dcsOption != 0xFF)
        {
            if (gRxVfo->pTX->CodeType != codeType ||
                gRxVfo->pTX->Code != dcsOption)
            {
                gRxVfo->pTX->CodeType = codeType;
                gRxVfo->pTX->Code = dcsOption;
                BK4819_SetCDCSSCodeWord(
                    DCS_GetGolayCodeWord(codeType, dcsOption));
            }
        }
    }
}

// for DTrac app RX Freq
static void CMD_9997(uint32_t Port, const uint8_t *pBuffer)
{
    const CMD_9997_t *pCmd = (const CMD_9997_t *)pBuffer;
    uint32_t downFrequency = pCmd->DownFrequency / 10;

    if (gRxVfo->pRX->Frequency != downFrequency)
    {
        gRxVfo->pRX->Frequency = downFrequency;

        if (!gPttIsPressed)
        {
            // BK4819_SetAGC(false);
            // SYSTICK_DelayUs(10);

            BK4819_SetFrequency(downFrequency);
            BK4819_PickRXFilterPathBasedOnFrequency(downFrequency);

            uint16_t reg30 = BK4819_ReadRegister(BK4819_REG_30);
            if (gMonitor)
            {
                BK4819_WriteRegister(BK4819_REG_30, reg30 & ~0x8000);
                SYSTICK_DelayUs(100);
                BK4819_WriteRegister(BK4819_REG_30, reg30 | 0x8000);
                SYSTICK_DelayUs(100);
                BK4819_WriteRegister(BK4819_REG_30, reg30);
            }
            else
            {
                // VCO Calibration bit 15 0x8000
                BK4819_WriteRegister(BK4819_REG_30, reg30 | 0x8000);
                SYSTICK_DelayUs(100);
                BK4819_WriteRegister(BK4819_REG_30, reg30);
            }

            BK4819_SetAGC(true);

            SYSTICK_DelayUs(1500);

            SendVersion(Port);
        }

        gUpdateDisplay = true;
    }
}

// for DTrac app TX Freq
static void CMD_9996(const uint8_t *pBuffer)
{
    const CMD_9996_t *pCmd = (const CMD_9996_t *)pBuffer;
    uint32_t upFrequency = pCmd->UpFrequency / 10;
    if (gRxVfo->pTX->Frequency != upFrequency)
    {
        gRxVfo->pTX->Frequency = upFrequency;

        if (gPttIsPressed)
        {
            BK4819_PickRXFilterPathBasedOnFrequency(upFrequency);
            BK4819_SetFrequency(upFrequency);

            uint16_t reg30 = BK4819_ReadRegister(BK4819_REG_30);

            // VCO Calibration bit 15 0x8000
            BK4819_WriteRegister(BK4819_REG_30, reg30 | 0x8000);
            SYSTICK_DelayUs(100);
            BK4819_WriteRegister(BK4819_REG_30, reg30);

            SYSTICK_DelayUs(1500);
        }

        gUpdateDisplay = true;
    }
}

// for DTrac app mode
static void CMD_9995(const uint8_t *pBuffer)
{
    const CMD_9995_t *pCmd = (const CMD_9995_t *)pBuffer;
    // char mode = pCmd->Mode;
    uint8_t mode;
    switch (pCmd->Mode)
    {
    case 'F':
        mode = MODULATION_FM;
        break;
    case 'A':
        mode = MODULATION_AM;
        break;
    case 'U':
        mode = MODULATION_USB;
        break;
#ifdef ENABLE_BYP_RAW_DEMODULATORS
    case 'B':
        mode = MODULATION_BYP;
        break;
    case 'R':
        mode = MODULATION_RAW;
        break;
#endif
    default:
        mode = MODULATION_FM;
        break;
    }
    if (gRxVfo->Modulation != mode)
    {
        gRxVfo->Modulation = mode;
        RADIO_SetModulation(mode);
    }

    gUpdateDisplay = true;
}

// for DTrac app MONITOR
char lastMonitorStatus = 'U';
static void CMD_9993(const uint8_t *pBuffer)
{
    const CMD_9993_t *pCmd = (const CMD_9993_t *)pBuffer;

    if (lastMonitorStatus != pCmd->MonitorStatus)
    {
        switch (pCmd->MonitorStatus)
        {
        case 'Y':
            RADIO_SetupRegisters(true);
            APP_StartListening(FUNCTION_MONITOR);
            break;
        case 'N':
            RADIO_SetupRegisters(true);
            APP_StartListening(FUNCTION_RECEIVE);
            break;
        }
        lastMonitorStatus = pCmd->MonitorStatus;
    }
}

#endif

#ifdef ENABLE_EXTRA_UART_CMD
// read RSSI
static void CMD_0527(uint32_t Port)
{
    REPLY_0527_t Reply;

    Reply.Header.ID = 0x0528;
    Reply.Header.Size = sizeof(Reply.Data);
    Reply.Data.RSSI = BK4819_ReadRegister(BK4819_REG_67) & 0x01FF;
    Reply.Data.ExNoiseIndicator = BK4819_ReadRegister(BK4819_REG_65) & 0x007F;
    Reply.Data.GlitchIndicator = BK4819_ReadRegister(BK4819_REG_63);

    SendReply(Port, &Reply, sizeof(Reply));
}

// read ADC
static void CMD_0529(uint32_t Port)
{
    REPLY_0529_t Reply;

    Reply.Header.ID = 0x52A;
    Reply.Header.Size = sizeof(Reply.Data);

    // Original doesn't actually send current!
    BOARD_ADC_GetBatteryInfo(&Reply.Data.Voltage, &Reply.Data.Current);

    SendReply(Port, &Reply, sizeof(Reply));
}

#ifndef ENABLE_FEAT_F4HWN
static void CMD_052D(uint32_t Port, const uint8_t *pBuffer)
{
    const CMD_052D_t *pCmd = (const CMD_052D_t *)pBuffer;
    REPLY_052D_t Reply;
    bool bIsLocked;

#ifdef ENABLE_FMRADIO_EMBEDDED
    gFmRadioCountdown_500ms = fm_radio_countdown_500ms;
#endif
    Reply.Header.ID = 0x052E;
    Reply.Header.Size = sizeof(Reply.Data);

    bIsLocked = bHasCustomAesKey;

    if (!bIsLocked)
        bIsLocked = IsBadChallenge(gCustomAesKey, gChallenge, pCmd->Response);

    if (!bIsLocked)
    {
        bIsLocked = IsBadChallenge(gDefaultAesKey, gChallenge, pCmd->Response);
        if (bIsLocked)
            gTryCount++;
    }

    if (gTryCount < 3)
    {
        if (!bIsLocked)
            gTryCount = 0;
    }
    else
    {
        gTryCount = 3;
        bIsLocked = true;
    }

    gIsLocked = bIsLocked;
    Reply.Data.bIsLocked = bIsLocked;
    Reply.Data.Padding[0] = Reply.Data.Padding[1] = Reply.Data.Padding[2] = 0;

    SendReply(Port, &Reply, sizeof(Reply));
}
#endif

// session init, sends back version info and state
// timestamp is a session id really
// this command also disables dual watch, crossband,
// DTMF side tones, freq reverse, PTT ID, DTMF decoding, frequency offset
// exits power save, sets main VFO to upper,
static void CMD_052F(uint32_t Port, const uint8_t *pBuffer)
{
    const CMD_052F_t *pCmd = (const CMD_052F_t *)pBuffer;

    gEeprom.DUAL_WATCH = DUAL_WATCH_OFF;
    gEeprom.CROSS_BAND_RX_TX = CROSS_BAND_OFF;
    gEeprom.RX_VFO = 0;
    gEeprom.DTMF_SIDE_TONE = false;
    gEeprom.VfoInfo[0].FrequencyReverse = false;
    gEeprom.VfoInfo[0].pRX = &gEeprom.VfoInfo[0].freq_config_RX;
    gEeprom.VfoInfo[0].pTX = &gEeprom.VfoInfo[0].freq_config_TX;
    gEeprom.VfoInfo[0].TX_OFFSET_FREQUENCY_DIRECTION = TX_OFFSET_FREQUENCY_DIRECTION_OFF;
    gEeprom.VfoInfo[0].DTMF_PTT_ID_TX_MODE = PTT_ID_OFF;
#ifdef ENABLE_DTMF_CALLING
    gEeprom.VfoInfo[0].DTMF_DECODING_ENABLE = false;
#endif

#ifdef ENABLE_NOAA
    gIsNoaaMode = false;
#endif

    if (gCurrentFunction == FUNCTION_POWER_SAVE)
        FUNCTION_Select(FUNCTION_FOREGROUND);

    gSerialConfigCountDown_500ms = 12; // 6 sec

    if (0)
    {
    }
#if defined(ENABLE_UART)
    else if (Port == UART_PORT_UART)
    {
        UART_Timestamp = pCmd->Timestamp;
    }
#endif
#if defined(ENABLE_USB)
    else if (Port == UART_PORT_VCP)
    {
        VCP_Timestamp = pCmd->Timestamp;
    }
#endif

    // Backlight left untouched: a serial session is neutral, so the normal BLTime
    // inactivity countdown keeps running from the last keypress (no forced turn-off).

    SendVersion(Port);
}
#endif

#ifdef ENABLE_UART_RW_BK_REGS
static void CMD_0601_ReadBK4819Reg(uint32_t Port, const uint8_t *pBuffer)
{
    typedef struct __attribute__((__packed__))
    {
        Header_t header;
        uint8_t reg;
    } CMD_0601_t;

    CMD_0601_t *cmd = (CMD_0601_t *)pBuffer;

    struct __attribute__((__packed__))
    {
        Header_t header;
        struct __attribute__((__packed__))
        {
            uint8_t reg;
            uint16_t value;
        } data;
    } reply;

    reply.header.ID = 0x0601;
    reply.header.Size = sizeof(reply.data);
    reply.data.reg = cmd->reg;
    reply.data.value = BK4819_ReadRegister(cmd->reg);
    SendReply(Port, &reply, sizeof(reply));
}

static void CMD_0602_WriteBK4819Reg(const uint8_t *pBuffer)
{
    typedef struct __attribute__((__packed__))
    {
        Header_t header;
        uint8_t reg;
        uint16_t value;
    } CMD_0602_t;

    CMD_0602_t *cmd = (CMD_0602_t *)pBuffer;
    BK4819_WriteRegister(cmd->reg, cmd->value);
}
#endif

bool UART_IsCommandAvailable(uint32_t Port)
{
    uint16_t Index;
    uint16_t TailIndex;
    uint16_t Size;
    uint16_t Crc;
    uint16_t CommandLength;
    uint16_t DmaLength;
    uint8_t *ReadBuf;
    uint16_t ReadBufSize;
    uint16_t *pReadPointer;
    UART_Command_t *pUART_Command;

    if (0)
    {
    }
#if defined(ENABLE_UART)
    else if (Port == UART_PORT_UART)
    {
        DmaLength = sizeof(UART_DMA_Buffer) - LL_DMA_GetDataLength(DMA1, DMA_CHANNEL);
        ReadBuf = UART_DMA_Buffer;
        ReadBufSize = sizeof(UART_DMA_Buffer);
        pReadPointer = &gUART_WriteIndex;
        pUART_Command = &UART_Command;
    }
#endif
#if defined(ENABLE_USB)
    else if (Port == UART_PORT_VCP)
    {
        DmaLength = VCP_RxBufPointer;
        ReadBuf = VCP_RxBuf;
        ReadBufSize = sizeof(VCP_RxBuf);
        pReadPointer = &VCP_ReadIndex;
        pUART_Command = &VCP_Command;
    }
#endif
    else
    {
        return false;
    }

    // Limit iterations to prevent long loops when buffer is full of non-command data
    uint16_t maxIterations = ReadBufSize + 1;

    while (maxIterations--)
    {
        if ((*pReadPointer) == DmaLength)
            return false;

        // Find 0xAB with iteration limit
        uint16_t searchLimit = ReadBufSize;
        while ((*pReadPointer) != DmaLength && ReadBuf[*pReadPointer] != 0xABU && searchLimit--)
            *pReadPointer = DMA_INDEX((*pReadPointer), 1, ReadBufSize);

        if (searchLimit == 0)
        {
            // Too many bytes without finding 0xAB - sync to current position and exit
            *pReadPointer = DmaLength;
            return false;
        }

        if ((*pReadPointer) == DmaLength)
            return false;

        if ((*pReadPointer) < DmaLength)
            CommandLength = DmaLength - (*pReadPointer);
        else
            CommandLength = (DmaLength + ReadBufSize) - (*pReadPointer);

        if (CommandLength < 8)
            return 0;

        if (ReadBuf[DMA_INDEX(*pReadPointer, 1, ReadBufSize)] == 0xCD)
            break;

        *pReadPointer = DMA_INDEX(*pReadPointer, 1, ReadBufSize);
    }

    if (maxIterations == 0)
    {
        // Safety: too many outer loop iterations
        *pReadPointer = DmaLength;
        return false;
    }

    Index = DMA_INDEX(*pReadPointer, 2, ReadBufSize);
    Size = (ReadBuf[DMA_INDEX(Index, 1, ReadBufSize)] << 8) | ReadBuf[Index];

    if ((Size + 8u) > ReadBufSize)
    {
        *pReadPointer = DmaLength;
        return false;
    }

    if (CommandLength < (Size + 8))
        return false;

    Index = DMA_INDEX(Index, 2, ReadBufSize);
    TailIndex = DMA_INDEX(Index, Size + 2, ReadBufSize);

    if (ReadBuf[TailIndex] != 0xDC || ReadBuf[DMA_INDEX(TailIndex, 1, ReadBufSize)] != 0xBA)
    {
        *pReadPointer = DmaLength;
        return false;
    }

    if (TailIndex < Index)
    {
        const uint16_t ChunkSize = ReadBufSize - Index;
        memcpy(pUART_Command->Buffer, ReadBuf + Index, ChunkSize);
        memcpy(pUART_Command->Buffer + ChunkSize, ReadBuf, TailIndex);
    }
    else
        memcpy(pUART_Command->Buffer, ReadBuf + Index, TailIndex - Index);

    TailIndex = DMA_INDEX(TailIndex, 2, ReadBufSize);
    if (TailIndex < (*pReadPointer))
    {
        memset(ReadBuf + (*pReadPointer), 0, ReadBufSize - (*pReadPointer));
        memset(ReadBuf, 0, TailIndex);
    }
    else
        memset(ReadBuf + (*pReadPointer), 0, TailIndex - (*pReadPointer));

    *pReadPointer = TailIndex;

    /* --
    if (pUART_Command->Header.ID == 0x0514)
        bIsEncrypted = false;

    if (pUART_Command->Header.ID == 0x6902)
        bIsEncrypted = true;
    -- */

    if (bIsEncrypted)
    {
        unsigned int i;
        for (i = 0; i < (Size + 2u); i++)
            pUART_Command->Buffer[i] ^= Obfuscation[i % 16];
    }

    Crc = pUART_Command->Buffer[Size] | (pUART_Command->Buffer[Size + 1] << 8);

    return Size >= sizeof(Header_t) &&
           pUART_Command->Header.Size <= Size - sizeof(Header_t) &&
           CRC_Calculate(pUART_Command->Buffer, Size) == Crc;
}

#ifdef ENABLE_FEAT_F4HWN_MULTIBOOT
/* Timestamp latched by the device-info handshake (0x0514) for this port. Slot
 * writes/erases require it to match, like the EEPROM write command (CMD_051D). */
static uint32_t mb_port_timestamp(uint32_t Port)
{
#if defined(ENABLE_UART)
    if (Port == UART_PORT_UART)
        return UART_Timestamp;
#endif
#if defined(ENABLE_USB)
    if (Port == UART_PORT_VCP)
        return VCP_Timestamp;
#endif
    (void)Port;
    return 0;
}
#endif

void UART_HandleCommand(uint32_t Port)
{
    UART_Command_t *pUART_Command;

    if (0)
    {
    }
#if defined(ENABLE_UART)
    else if (Port == UART_PORT_UART)
    {
        pUART_Command = &UART_Command;
    }
#endif
#if defined(ENABLE_USB)
    else if (Port == UART_PORT_VCP)
    {
        pUART_Command = &VCP_Command;
    }
#endif
    else
    {
        return;
    }

    switch (pUART_Command->Header.ID)
    {
#if defined(ENABLE_DTRAC)
    case 0x9999:
        CMD_9999(pUART_Command->Buffer); // for DTrac app TX CTCSS/DCS
        break;

    case 0x9998:
        CMD_9998(pUART_Command->Buffer); // for DTrac app RX CTCSS/DCS
        break;

    case 0x9997:
        CMD_9997(Port, pUART_Command->Buffer); // for DTrac app RX Freq
        break;

    case 0x9996:
        CMD_9996(pUART_Command->Buffer); // for DTrac app TX Freq
        break;

    case 0x9995:
        CMD_9995(pUART_Command->Buffer); // for DTrac app Mode
        break;

    case 0x9993:
        CMD_9993(pUART_Command->Buffer); // for DTrac app MONITOR
        break;
#endif
#ifdef ENABLE_AIRCOPY_UART
    case UART_CMD_AIRCOPY:
    {
        const uint8_t words = pUART_Command->Data[0];
        const uint16_t payloadSize = (uint16_t)(2u + words * sizeof(uint16_t));

        if (Port == UART_PORT_UART &&
            words > 0u && words <= AIRCOPY_FRAME_WORDS_MAX &&
            pUART_Command->Header.Size == payloadSize)
        {
            AIRCOPY_StoreUartPacket(&pUART_Command->Data[2], words);
        }
        break;
    }
#endif

    case 0x0514:
        CMD_0514(Port, pUART_Command->Buffer);
        break;

    case 0x051B:
        CMD_051B(Port, pUART_Command->Buffer);
        break;

    case 0x051D:
        CMD_051D(Port, pUART_Command->Buffer);
        break;

    case 0x051F: // Not implementing non-authentic command
        break;

    case 0x0521: // Not implementing non-authentic command
        break;

#ifdef ENABLE_EXTRA_UART_CMD
    case 0x0527:
        CMD_0527(Port);
        break;

    case 0x0529:
        CMD_0529(Port);
        break;

#ifndef ENABLE_FEAT_F4HWN
    case 0x052D:
        CMD_052D(Port, pUART_Command->Buffer);
        break;
#endif

    case 0x052F:
        CMD_052F(Port, pUART_Command->Buffer);
        break;
#endif

    case 0x05DD: // reset
#if defined(ENABLE_OVERLAY)
        overlay_FLASH_RebootToBootloader();
#else
        NVIC_SystemReset();
#endif
        break;

#ifdef ENABLE_FEAT_F4HWN_MULTIBOOT
    // ---- M4 slot management ("Firmware Slots") ------------------------
    case 0x0720: // slot info: read the 64-byte header only (fast, no CRC)
    {
        if (pUART_Command->Header.Size < 1u)
            break; // needs Data[0] (slot)
        uint8_t slot = pUART_Command->Data[0];
        mb_slot_header_t hdr;
        memset(&hdr, 0, sizeof(hdr));
        uint8_t status = MB_SlotInfo(slot, &hdr);
        struct __attribute__((packed))
        {
            Header_t Header;
            uint8_t Slot;
            uint8_t Status;
            uint8_t Hdr[sizeof(mb_slot_header_t)];
        } Reply;
        Reply.Header.ID = 0x0721;
        Reply.Header.Size = 2 + sizeof(mb_slot_header_t);
        Reply.Slot = slot;
        Reply.Status = status;
        memcpy(Reply.Hdr, &hdr, sizeof(hdr));
        SendReply(Port, &Reply, sizeof(Reply));
        break;
    }

    case 0x0722: // slot erase: wipe the whole 128 KiB slot region
    {
        if (pUART_Command->Header.Size < 6u)
            break;                         // needs Data[0] slot + Data[2..5] timestamp
        gSerialConfigCountDown_500ms = 12; // keep serial mode alive (6 s)
        uint8_t slot = pUART_Command->Data[0];
        uint32_t ts = (uint32_t)pUART_Command->Data[2] | ((uint32_t)pUART_Command->Data[3] << 8) | ((uint32_t)pUART_Command->Data[4] << 16) | ((uint32_t)pUART_Command->Data[5] << 24);
        uint8_t status = (ts != mb_port_timestamp(Port))
                             ? MB_ERR_AUTH
                             : MB_SlotErase(slot);
        struct __attribute__((packed))
        {
            Header_t Header;
            uint8_t Slot;
            uint8_t Status;
        } Reply;
        Reply.Header.ID = 0x0723;
        Reply.Header.Size = 2;
        Reply.Slot = slot;
        Reply.Status = status;
        SendReply(Port, &Reply, sizeof(Reply));
        break;
    }

    case 0x0724: // slot write: program bytes at slot+offset (slot pre-erased)
    {
        if (pUART_Command->Header.Size < 12u)
            break;
        gSerialConfigCountDown_500ms = 12; // keep serial mode alive (6 s)
        uint8_t slot = pUART_Command->Data[0];
        uint32_t offset = (uint32_t)pUART_Command->Data[2] | ((uint32_t)pUART_Command->Data[3] << 8) | ((uint32_t)pUART_Command->Data[4] << 16) | ((uint32_t)pUART_Command->Data[5] << 24);
        uint16_t len = (uint16_t)(pUART_Command->Data[6] | ((uint16_t)pUART_Command->Data[7] << 8));
        uint32_t ts = (uint32_t)pUART_Command->Data[8] | ((uint32_t)pUART_Command->Data[9] << 8) | ((uint32_t)pUART_Command->Data[10] << 16) | ((uint32_t)pUART_Command->Data[11] << 24);
        uint8_t status;
        if (ts != mb_port_timestamp(Port))
            status = MB_ERR_AUTH;
        else if (len > pUART_Command->Header.Size - 12u)
            status = MB_ERR_SIZE;
        else
            status = MB_SlotWrite(slot, offset, &pUART_Command->Data[12], len);
        struct __attribute__((packed))
        {
            Header_t Header;
            uint8_t Slot;
            uint8_t Status;
        } Reply;
        Reply.Header.ID = 0x0725;
        Reply.Header.Size = 2;
        Reply.Slot = slot;
        Reply.Status = status;
        SendReply(Port, &Reply, sizeof(Reply));
        break;
    }

    case 0x0726: // slot validate: full image CRC-32, no reflash
    {
        if (pUART_Command->Header.Size < 1u)
            break;                         // needs Data[0] (slot)
        gSerialConfigCountDown_500ms = 12; // keep serial mode alive (6 s)
        uint8_t slot = pUART_Command->Data[0];
        uint32_t crc = 0;
        uint8_t status = MB_ValidateSlot(slot, NULL, &crc);
        struct __attribute__((packed))
        {
            Header_t Header;
            uint32_t Crc32; // offset 4: 4-byte aligned, no unaligned store
            uint8_t Slot;
            uint8_t Status;
        } Reply;
        Reply.Header.ID = 0x0727;
        Reply.Header.Size = 6;
        Reply.Crc32 = crc;
        Reply.Slot = slot;
        Reply.Status = status;
        SendReply(Port, &Reply, sizeof(Reply));
        break;
    }

    case 0x0728: // config reset: wipe the 64 KiB of a config bank (1..4)
    {
        if (pUART_Command->Header.Size < 6u)
            break;                         // needs Data[0] bank + Data[2..5] timestamp
        gSerialConfigCountDown_500ms = 12; // keep serial mode alive (6 s)
        uint8_t bank = pUART_Command->Data[0];
        uint32_t ts = (uint32_t)pUART_Command->Data[2] | ((uint32_t)pUART_Command->Data[3] << 8) | ((uint32_t)pUART_Command->Data[4] << 16) | ((uint32_t)pUART_Command->Data[5] << 24);
        uint8_t status = (ts != mb_port_timestamp(Port))
                             ? MB_ERR_AUTH
                             : MB_BankErase(bank);
        struct __attribute__((packed))
        {
            Header_t Header;
            uint8_t Bank; // echoes the erased bank (same wire layout as slot replies)
            uint8_t Status;
        } Reply;
        Reply.Header.ID = 0x0729;
        Reply.Header.Size = 2;
        Reply.Bank = bank;
        Reply.Status = status;
        SendReply(Port, &Reply, sizeof(Reply));
        break;
    }

#ifdef ENABLE_FEAT_F4HWN_EXT_FLASH_RW
    // ---- full external-flash dump / restore (host: UV Studio) ----------
    // Raw access by physical address, bypassing the config-bank mapping;
    // all four commands are timestamp-authenticated like the slot ops.
    // Restore flow: erase each unprotected 4 KiB sector (0x073A), then
    // program it in chunks (0x073C). The calibration sector is rejected.
    case 0x0738: // read raw external flash by physical address (full-chip dump)
    {
        if (pUART_Command->Header.Size != 10u)
            break;                         // addr(4) + len(2) + timestamp(4)
        gSerialConfigCountDown_500ms = 12; // keep serial mode alive (6 s)
        uint32_t addr = (uint32_t)pUART_Command->Data[0] | ((uint32_t)pUART_Command->Data[1] << 8) | ((uint32_t)pUART_Command->Data[2] << 16) | ((uint32_t)pUART_Command->Data[3] << 24);
        uint16_t len = (uint16_t)(pUART_Command->Data[4] | ((uint16_t)pUART_Command->Data[5] << 8));
        uint32_t ts = (uint32_t)pUART_Command->Data[6] | ((uint32_t)pUART_Command->Data[7] << 8) | ((uint32_t)pUART_Command->Data[8] << 16) | ((uint32_t)pUART_Command->Data[9] << 24);
        struct __attribute__((packed))
        {
            Header_t Header;
            uint32_t Address;                     // echoes the requested physical address
            uint16_t Size;                        // bytes returned (0 on error)
            uint8_t Status;                       // MB_OK or an MB_ERR_* code
            uint8_t Data[PY25Q16_RAW_CHUNK_SIZE]; // capped to fit MAX_REPLY_SIZE
        } Reply;
        memset(&Reply, 0, sizeof(Reply));
        uint8_t status;
        if (ts != mb_port_timestamp(Port))
            status = MB_ERR_AUTH;
        else if (len == 0u || len > sizeof(Reply.Data))
            status = MB_ERR_SIZE;
        else if (addr > PY25Q16_TOTAL_SIZE || len > PY25Q16_TOTAL_SIZE - addr)
            status = MB_ERR_SIZE;
        else
        {
            PY25Q16_ReadBufferPhysical(addr, Reply.Data, len);
            status = MB_OK;
        }
        Reply.Header.ID = 0x0739;
        Reply.Address = addr;
        Reply.Size = (status == MB_OK) ? len : 0;
        Reply.Status = status;
        Reply.Header.Size = 7 + Reply.Size;       // Address(4)+Size(2)+Status(1)+Data
        SendReply(Port, &Reply, 11 + Reply.Size); // + Header(4)
        break;
    }

    case 0x073A: // erase one 4 KiB external-flash sector by physical address
    {
        if (pUART_Command->Header.Size != 8u)
            break;                         // addr(4) + timestamp(4)
        gSerialConfigCountDown_500ms = 12; // keep serial mode alive (6 s)
        uint32_t addr = (uint32_t)pUART_Command->Data[0] | ((uint32_t)pUART_Command->Data[1] << 8) | ((uint32_t)pUART_Command->Data[2] << 16) | ((uint32_t)pUART_Command->Data[3] << 24);
        uint32_t ts = (uint32_t)pUART_Command->Data[4] | ((uint32_t)pUART_Command->Data[5] << 8) | ((uint32_t)pUART_Command->Data[6] << 16) | ((uint32_t)pUART_Command->Data[7] << 24);
        uint8_t status;
        if (ts != mb_port_timestamp(Port))
            status = MB_ERR_AUTH;
        else if (addr >= PY25Q16_TOTAL_SIZE ||
                 (addr % PY25Q16_SECTOR_SIZE) != 0u)
            status = MB_ERR_SIZE;
        else if (addr == PY25Q16_CALIBRATION_SECTOR_BASE)
            status = MB_ERR_PROTECTED; // device-specific data is never restorable
        else
        {
            PY25Q16_SectorErasePhysical(addr);
            status = MB_OK;
        }
        struct __attribute__((packed))
        {
            Header_t Header;
            uint32_t Address; // echoes the requested physical address
            uint8_t Status;   // MB_OK or an MB_ERR_* code
        } Reply;
        Reply.Header.ID = 0x073B;
        Reply.Address = addr;
        Reply.Status = status;
        Reply.Header.Size = 5;      // Address(4)+Status(1)
        SendReply(Port, &Reply, 9); // + Header(4)
        break;
    }

    case 0x073C: // program bytes into external flash by physical address (sector pre-erased)
    {
        if (pUART_Command->Header.Size < 10u)
            break;                         // fixed fields + data
        gSerialConfigCountDown_500ms = 12; // keep serial mode alive (6 s)
        uint32_t addr = (uint32_t)pUART_Command->Data[0] | ((uint32_t)pUART_Command->Data[1] << 8) | ((uint32_t)pUART_Command->Data[2] << 16) | ((uint32_t)pUART_Command->Data[3] << 24);
        uint16_t len = (uint16_t)(pUART_Command->Data[4] | ((uint16_t)pUART_Command->Data[5] << 8));
        uint32_t ts = (uint32_t)pUART_Command->Data[6] | ((uint32_t)pUART_Command->Data[7] << 8) | ((uint32_t)pUART_Command->Data[8] << 16) | ((uint32_t)pUART_Command->Data[9] << 24);
        uint8_t status;
        if (ts != mb_port_timestamp(Port))
            status = MB_ERR_AUTH;
        else if (len == 0u || len > PY25Q16_RAW_CHUNK_SIZE ||
                 len != pUART_Command->Header.Size - 10u)
            status = MB_ERR_SIZE;
        else if (addr > PY25Q16_TOTAL_SIZE || len > PY25Q16_TOTAL_SIZE - addr)
            status = MB_ERR_SIZE;
        else if (addr < PY25Q16_CALIBRATION_SECTOR_BASE + PY25Q16_SECTOR_SIZE &&
                 addr + len > PY25Q16_CALIBRATION_SECTOR_BASE)
            status = MB_ERR_PROTECTED; // device-specific data is never restorable
        else
        {
            PY25Q16_WriteBufferPhysical(addr, &pUART_Command->Data[10], len);
            status = MB_OK;
        }
        struct __attribute__((packed))
        {
            Header_t Header;
            uint32_t Address; // echoes the requested physical address
            uint16_t Size;    // bytes written (0 on error)
            uint8_t Status;   // MB_OK or an MB_ERR_* code
        } Reply;
        Reply.Header.ID = 0x073D;
        Reply.Address = addr;
        Reply.Size = (status == MB_OK) ? len : 0;
        Reply.Status = status;
        Reply.Header.Size = 7;       // Address(4)+Size(2)+Status(1)
        SendReply(Port, &Reply, 11); // + Header(4)
        break;
    }

    case 0x073E: // CRC-32 of a physical external-flash range
    {
        if (pUART_Command->Header.Size != 12u)
            break;                         // addr(4) + len(4) + timestamp(4)
        gSerialConfigCountDown_500ms = 12; // keep serial mode alive (6 s)
        uint32_t addr = (uint32_t)pUART_Command->Data[0] | ((uint32_t)pUART_Command->Data[1] << 8) | ((uint32_t)pUART_Command->Data[2] << 16) | ((uint32_t)pUART_Command->Data[3] << 24);
        uint32_t len = (uint32_t)pUART_Command->Data[4] | ((uint32_t)pUART_Command->Data[5] << 8) | ((uint32_t)pUART_Command->Data[6] << 16) | ((uint32_t)pUART_Command->Data[7] << 24);
        uint32_t ts = (uint32_t)pUART_Command->Data[8] | ((uint32_t)pUART_Command->Data[9] << 8) | ((uint32_t)pUART_Command->Data[10] << 16) | ((uint32_t)pUART_Command->Data[11] << 24);
        uint32_t crc = 0;
        uint8_t status;
        if (ts != mb_port_timestamp(Port))
            status = MB_ERR_AUTH;
        else if (len == 0u || len > PY25Q16_SECTOR_SIZE)
            status = MB_ERR_SIZE;
        else
            status = MB_ExternalFlashCrc32(addr, len, &crc);
        struct __attribute__((packed))
        {
            Header_t Header;
            uint32_t Address; // echoes the requested physical address
            uint32_t Size;    // bytes covered by the CRC
            uint32_t Crc32;   // zlib-compatible CRC-32
            uint8_t Status;   // MB_OK or an MB_ERR_* code
        } Reply;
        Reply.Header.ID = 0x073F;
        Reply.Address = addr;
        Reply.Size = (status == MB_OK) ? len : 0;
        Reply.Crc32 = (status == MB_OK) ? crc : 0;
        Reply.Status = status;
        Reply.Header.Size = 13; // Address(4)+Size(4)+CRC32(4)+Status(1)
        SendReply(Port, &Reply, sizeof(Reply));
        break;
    }
#endif // ENABLE_FEAT_F4HWN_EXT_FLASH_RW
#endif

#ifdef ENABLE_FEAT_F4HWN_OVERLAY_APPS
    // ---- overlay-app slot management ("Apps") -------------------------
    // Parallels the firmware-slot family (0x072x); targets the external-flash
    // Apps region. External flash only, never brick-critical.
    case 0x0730: // app slot info: read the 64-byte header only
    {
        if (pUART_Command->Header.Size < 1u)
            break;                         // needs Data[0] (slot)
        gSerialConfigCountDown_500ms = 12; // keep serial mode alive (6 s)
        uint8_t slot = pUART_Command->Data[0];
        app_header_t hdr;
        memset(&hdr, 0, sizeof(hdr));
        uint8_t status = APP_SlotInfo(slot, &hdr);
        struct __attribute__((packed))
        {
            Header_t Header;
            uint8_t Slot;
            uint8_t Status;
            uint8_t Hdr[sizeof(app_header_t)];
        } Reply;
        Reply.Header.ID = 0x0731;
        Reply.Header.Size = 2 + sizeof(app_header_t);
        Reply.Slot = slot;
        Reply.Status = status;
        memcpy(Reply.Hdr, &hdr, sizeof(hdr));
        SendReply(Port, &Reply, sizeof(Reply));
        break;
    }

    case 0x0732: // app slot erase: wipe the 8 KiB slot region, but the app's tagged settings (see APP_SlotErase)
    {
        if (pUART_Command->Header.Size < 6u)
            break;                         // needs Data[0] slot + Data[2..5] timestamp
        gSerialConfigCountDown_500ms = 12; // keep serial mode alive (6 s)
        uint8_t slot = pUART_Command->Data[0];
        uint32_t ts = (uint32_t)pUART_Command->Data[2] | ((uint32_t)pUART_Command->Data[3] << 8) | ((uint32_t)pUART_Command->Data[4] << 16) | ((uint32_t)pUART_Command->Data[5] << 24);
        uint8_t status = (ts != mb_port_timestamp(Port))
                             ? APP_ERR_AUTH
                             : APP_SlotErase(slot);
        struct __attribute__((packed))
        {
            Header_t Header;
            uint8_t Slot;
            uint8_t Status;
        } Reply;
        Reply.Header.ID = 0x0733;
        Reply.Header.Size = 2;
        Reply.Slot = slot;
        Reply.Status = status;
        SendReply(Port, &Reply, sizeof(Reply));
        break;
    }

    case 0x0734: // app slot write: program bytes at slot+offset (pre-erased)
    {
        if (pUART_Command->Header.Size < 12u)
            break;
        gSerialConfigCountDown_500ms = 12; // keep serial mode alive (6 s)
        uint8_t slot = pUART_Command->Data[0];
        uint32_t offset = (uint32_t)pUART_Command->Data[2] | ((uint32_t)pUART_Command->Data[3] << 8) | ((uint32_t)pUART_Command->Data[4] << 16) | ((uint32_t)pUART_Command->Data[5] << 24);
        uint16_t len = (uint16_t)(pUART_Command->Data[6] | ((uint16_t)pUART_Command->Data[7] << 8));
        uint32_t ts = (uint32_t)pUART_Command->Data[8] | ((uint32_t)pUART_Command->Data[9] << 8) | ((uint32_t)pUART_Command->Data[10] << 16) | ((uint32_t)pUART_Command->Data[11] << 24);
        uint8_t status;
        if (ts != mb_port_timestamp(Port))
            status = APP_ERR_AUTH;
        else if (len > pUART_Command->Header.Size - 12u)
            status = APP_ERR_SIZE;
        else
            status = APP_SlotWrite(slot, offset, &pUART_Command->Data[12], len);
        struct __attribute__((packed))
        {
            Header_t Header;
            uint8_t Slot;
            uint8_t Status;
        } Reply;
        Reply.Header.ID = 0x0735;
        Reply.Header.Size = 2;
        Reply.Slot = slot;
        Reply.Status = status;
        SendReply(Port, &Reply, sizeof(Reply));
        break;
    }

    case 0x0736: // app slot validate: header only (code CRC is checked at launch)
    {
        if (pUART_Command->Header.Size < 1u)
            break;                         // needs Data[0] (slot)
        gSerialConfigCountDown_500ms = 12; // keep serial mode alive (6 s)
        uint8_t slot = pUART_Command->Data[0];
        uint8_t status = APP_ValidateSlot(slot, NULL);
        // Also when refused: an incompatible app is installed and listed all the same.
        APP_NotifySlotChanged();
        struct __attribute__((packed))
        {
            Header_t Header;
            uint8_t Slot;
            uint8_t Status;
        } Reply;
        Reply.Header.ID = 0x0737;
        Reply.Header.Size = 2;
        Reply.Slot = slot;
        Reply.Status = status;
        SendReply(Port, &Reply, sizeof(Reply));
        break;
    }
#endif

#ifdef ENABLE_UART_RW_BK_REGS
    case 0x0601:
        CMD_0601_ReadBK4819Reg(Port, pUART_Command->Buffer);
        break;

    case 0x0602:
        CMD_0602_WriteBK4819Reg(pUART_Command->Buffer);
        break;
#endif
    } // switch

#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
    gUART_LockK5Viewer = 20; // lock the K5Viewer stream
#endif
}

void UART_ServiceCommands(void)
{
#ifdef ENABLE_USB
    if (UART_IsCommandAvailable(UART_PORT_VCP))
        UART_HandleCommand(UART_PORT_VCP);
#endif

#ifdef ENABLE_UART
    if (UART_IsCommandAvailable(UART_PORT_UART))
        UART_HandleCommand(UART_PORT_UART);
#endif
}
