/*

Module:  catena5120-mfgtest.ino

Function:
        Test app for MCCI Catena 4610 Rev-D

Copyright notice and License:
        Copyright (C) 2019 MCCI Corporation.

        An unpublished work, all rights reserved.

Author:
        Terry Moore, MCCI Corporation	October 2019

*/

#include <Arduino.h>
#include <Catena.h>
#include <Catena_Led.h>
#include <SPI.h>
#include <wiring_private.h>
#include <Catena_Mx25v8035f.h>
#include <Catena_Guids.h>
#include <Catena-SHT3x.h>
#include <MCCI_Catena_LTR329.h>
#include <Wire.h>

#include <arduino_lmic.h>

#include <ArduinoUnit.h>

#include <catena_mfgtest.h>

static_assert(
    CATENA_ARDUINO_PLATFORM_VERSION >= CATENA_ARDUINO_PLATFORM_VERSION_CALC(0,17,0,60),
    "Please update your catena-arduion-platform library to v0.17.0.60 or later"
    );

using namespace McciCatena;
using namespace McciCatenaSht3x;

/****************************************************************************\
|
|	READ-ONLY DATA
|
\****************************************************************************/

static const char sVersion[] = "1.0.0";

/****************************************************************************\
|
|   handy constexpr to extract the base name of a file
|
\****************************************************************************/

// two-argument version: first arg is what to return if we don't find
// a directory separator in the second part.
static constexpr const char *filebasename(const char *s, const char *p)
    {
    return p[0] == '\0'                     ? s                            :
           (p[0] == '/' || p[0] == '\\')    ? filebasename(p + 1, p + 1)   :
                                              filebasename(s, p + 1)       ;
    }

static constexpr const char *filebasename(const char *s)
    {
    return filebasename(s, s);
    }

/****************************************************************************\
|
|	VARIABLES
|
\****************************************************************************/

Catena gCatena;

// declare LoRaWAN (so we can provision)
Catena::LoRaWAN gLoRaWAN;

// declare the LED object
StatusLed gLed (Catena::PIN_STATUS_LED);

SPIClass gSPI2(
    Catena::PIN_SPI2_MOSI,
    Catena::PIN_SPI2_MISO,
    Catena::PIN_SPI2_SCK
    );

//   The flash
class myMx25v8035f : public Catena_Mx25v8035f
    {
public:
    // size of device in bytes:
    static constexpr size_t deviceSizeBytes()
        {
        return 1 * 1024 * 1024;
        }
    static constexpr size_t sectorSizeBytes()
        {
        return 4096;
        }
    } gFlash;

bool fFlashDone = false;

bool flash_init(void);
void setup_flash(void);
void setup_platform(void);

//   The temperature/humidity sensor
cSHT3x gSht3x {Wire};

//   LTR329 LUX sensor
McciCatenaLtr329::cLTR329 gLTR329 {Wire};  

//  True if LoRaWAN passed init.
bool gfLoRaWANPassed = false;


/*

Name:	setup()

Function:
    Arduino setup function.

Definition:
    void setup(
        void
        );

Description:
    This function is called by the Arduino framework after
    basic framework has been initialized. We initialize the sensors
    that are present on the platform, set up the LoRaWAN connection,
    and (ultimately) return to the framework, which then calls loop()
    forever.

Returns:
    No explicit result.

*/

void setup()
    {
    gCatena.begin();

    setup_platform();

    //** set up for flash work **
    setup_flash();

    // set up commands
    setup_commands();
    }

void setup_platform()
    {
    while (! Serial)
            /* wait for USB attach */
            yield();

    gCatena.SafePrintf("\n");
    gCatena.SafePrintf(
        "-------------------------------------------------------------------------------\n"
        );
    gCatena.SafePrintf(
        "This is %s V%s.\n",
        filebasename(__FILE__),
        sVersion
        );
    do
        {
        char sRegion[16];
        gCatena.SafePrintf("Target network: %s / %s\n",
                        gLoRaWAN.GetNetworkName(),
                        gLoRaWAN.GetRegionString(sRegion, sizeof(sRegion))
                        );
        } while (0);

    gCatena.SafePrintf("System clock rate is %u.%03u MHz\n",
        ((unsigned)gCatena.GetSystemClockRate() / (1000*1000)),
        ((unsigned)gCatena.GetSystemClockRate() / 1000 % 1000)
        );
    gCatena.SafePrintf("Enter 'help' for a list of commands.\n");
    gCatena.SafePrintf("Please select 'Line Ending: Newline' in the IDE monitor window.\n");
    gCatena.SafePrintf("If using a terminal emulator, please turn off local echo.\n");
    gCatena.SafePrintf(
        "--------------------------------------------------------------------------------\n"
        );
    gCatena.SafePrintf(
        "\n"
        );

    gLed.begin();
    gCatena.registerObject(&gLed);
    gLed.Set(LedPattern::FastFlash);

    // record success of gLoRaWAN.begin().
    gfLoRaWANPassed = gLoRaWAN.begin(&gCatena);

    // always register
    gCatena.registerObject(&gLoRaWAN);

    Catena::UniqueID_string_t CpuIDstring;

    gCatena.SafePrintf(
            "CPU Unique ID: %s\n",
            gCatena.GetUniqueIDstring(&CpuIDstring)
            );
    }

void setup_flash()
    {
    // we need to test this, so the real work is in the test procedure
    gSPI2.begin();
    }

cMfgTest::doSleepCb_t doDeepSleep;

void setup_commands()
    {
    // add the commom command tools
    setup_mfg_commands(
        gCatena,
        234001129, // number on the PCB.
        4610,
        0, // no mod
        3, // rev d
        0  // no dash
        );

    gMfgTest.setDeepSleepCb(doDeepSleep, nullptr);
    }

void doDeepSleep(
    void *pClientData,
    std::uint32_t howLong
    )
    {
    auto saveLed = gLed.Set(LedPattern::Off);
    deepSleepPrepare();

    /* sleep */
    gCatena.Sleep(howLong);

    /* recover from sleep */
    deepSleepRecovery();

    /* and now... we're awake again. */
    gLed.Set(saveLed);
    }

void deepSleepPrepare(void)
        {
        Serial.end();
        Wire.end();
        SPI.end();
        // if (fFlash)
                gSPI2.end();
        }

void deepSleepRecovery(void)
        {
        Serial.begin();
        Wire.begin();
        SPI.begin();
        // if (fFlash)
                gSPI2.begin();
        }

/*

Name:	loop()

Function:
    Arduino poll function.

Definition:
    void loop(
        void
        );

Description:
    This function is called repeatedly by the Arduino framework
    after setup() has been called.  We poll the catena framework,
    then call the unit test runner.

Returns:
    No explicit result.

*/

void loop()
    {
    gCatena.poll();
    // Test::run() is called by gMfgTest::poll(), if "test start"
    // has been entered.
    }

//-----------------------------------------------------
//      Flash tests
//-----------------------------------------------------
void logMismatch(
    uint32_t addr,
    uint8_t expect,
    uint8_t actual
    )
    {
    gCatena.SafePrintf(
        "mismatch address %#x: expect %#02x got %02x\n",
        addr, expect, actual
        );
    }

uint32_t vNext(uint32_t v)
    {
    return v * 31413 + 3;
    }

// choose a sector.
const uint32_t sectorAddress = gFlash.deviceSizeBytes() - gFlash.sectorSizeBytes();

union sectorBuffer_t {
    uint8_t b[gFlash.sectorSizeBytes()];
    uint32_t dw[gFlash.sectorSizeBytes() / sizeof(uint32_t)];
    } sectorBuffer;

test(1_flash_00init)
    {
    assertTrue(flash_init(), "flash_init()");
    pass();
    }

// skip this test if previous test failed
#define skipIfFailed(t) do { if (! checkTestPass(t)) { skip(); return; } } while (0)

test(1_flash_01erase)
    {
    // erase the sector.
    skipIfFailed(1_flash_00init);
    // todo -- change lib to return a bool, and check.
    gFlash.powerUp();
    gFlash.eraseSector(sectorAddress);
    pass();
    }

uint32_t flashBlankCheck(
    uint32_t a = sectorAddress,
    sectorBuffer_t &buffer = sectorBuffer
    )
    {
    // make sure the sector is blank
    memset(buffer.b, 0, sizeof(buffer));
    gFlash.read(a, buffer.b, sizeof(buffer.b));
    unsigned errs = 0;
    for (auto i = 0; i < sizeof(buffer.b); ++i)
        {
        if (buffer.b[i] != 0xFF)
            {
            logMismatch(a + i, 0xFF, buffer.b[i]);
            ++errs;
            }
        }
    return errs;
    }

test(1_flash_02blankcheck)
    {
    skipIfFailed(1_flash_00init);

    auto errs = flashBlankCheck();

    assertEqual(errs, 0, "mismatch errors: " << errs);
    pass();
    }

void initBuffer(
    uint32_t v,
    sectorBuffer_t &buffer = sectorBuffer
    )
    {
    for (auto i = 0;
         i < sizeof(buffer.dw) / sizeof(buffer.dw[0]);
         ++i, v = vNext(v))
        {
        buffer.dw[i] = v;
        }
    }

const uint32_t vStart = 0x55555555u;

test(1_flash_03writepattern)
    {
    skipIfFailed(1_flash_00init);

    // write a pattern
    initBuffer(vStart, sectorBuffer);

    // assertTrue(gFlash.program(sectorAddress, sectorBuffer.b, sizeof(sectorBuffer.b)),
    //	"Failed to program sector " << sectorAddress
    //	);
    gFlash.program(sectorAddress, sectorBuffer.b, sizeof(sectorBuffer.b));
    pass();
    }

test(1_flash_04readpattern)
    {
    skipIfFailed(1_flash_00init);

    // read the buffer
    for (auto i = 0; i < sizeof(sectorBuffer.b); ++i)
        sectorBuffer.b[i] = ~sectorBuffer.b[i];

    gFlash.read(sectorAddress, sectorBuffer.b, sizeof(sectorBuffer.b));

    union 	{
        uint8_t b[sizeof(uint32_t)];
        uint32_t dw;
        } vTest, v;
    v.dw = vStart;

    auto errs = 0;
    for (auto i = 0;
         i < sizeof(sectorBuffer.dw) / sizeof(sectorBuffer.dw[0]);
         ++i, v.dw = vNext(v.dw))
        {
        vTest.dw = sectorBuffer.dw[i];

        for (auto j = 0; j < sizeof(v.b); ++j)
            {
            if (v.b[j] != vTest.b[j])
                {
                ++errs;
                logMismatch(sectorAddress + sizeof(uint32_t) * i + j, v.b[j], vTest.b[j]);
                }
            }
        }
    assertEqual(errs, 0, "mismatch errors: " << errs);
    pass();
    }

test(1_flash_05posterase)
    {
    // do an assert so this will fail (& be done) always.
    skipIfFailed(1_flash_00init);

//	assertTrue(gFlash.eraseSector(sectorAddress));
    gFlash.eraseSector(sectorAddress);
    pass();
    }

testing(1_flash_99done)
    {
    if (checkTestDone(1_flash_05posterase) || checkTestSkip(1_flash_05posterase))
        {
        fFlashDone = true;
        gFlash.powerDown();
        assertTestPass(1_flash_05posterase);
        pass();
        }
    }

// boilerplate setup code
bool flash_init(void)
    {
    bool fFlashFound;

    gCatena.SafePrintf("Init FLASH\n");

    if (gFlash.begin(&gSPI2, Catena::PIN_SPI2_FLASH_SS))
        {
        uint8_t ManufacturerId;
        uint16_t DeviceId;

        gFlash.readId(&ManufacturerId, &DeviceId);
        gCatena.SafePrintf(
            "FLASH found, ManufacturerId=%02x, DeviceId=%04x\n",
            ManufacturerId, DeviceId
            );
        gFlash.powerDown();
        fFlashFound = true;
        }
    else
        {
        gCatena.SafePrintf("No FLASH found\n");
        fFlashFound = false;
        }
    return fFlashFound;
    }

//-----------------------------------------------------
// platform tests
//-----------------------------------------------------

test(2_platform_05_check_platform_guid)
    {
    const CATENA_PLATFORM * const pPlatform = gCatena.GetPlatform();
    const MCCIADK_GUID_WIRE Guid4610 = GUID_HW_CATENA_4610_BASE(WIRE);

    assertTrue(pPlatform != nullptr, "gCatena.GetPlatform() failed -- this is normal on first boot");
    assertEqual(
        memcmp(&Guid4610, &pPlatform->Guid, sizeof(Guid4610)), 0,
        "platform GUID mismatch"
        );
    pass();
    }

test(2_platform_10_check_syseui)
    {
    const Catena::EUI64_buffer_t *pSysEUI = gCatena.GetSysEUI();
    static const Catena::EUI64_buffer_t ZeroEUI = { 0 };
    bool fNonZeroEUI;

    assertTrue(pSysEUI != nullptr);
    fNonZeroEUI = memcmp(pSysEUI, &ZeroEUI, sizeof(ZeroEUI)) != 0;
    assertTrue(fNonZeroEUI, "SysEUI is zero. This is normal on first boot.");

    for (unsigned i = 0; i < sizeof(pSysEUI->b); ++i)
        {
        gCatena.SafePrintf("%s%02x", i == 0 ? "  SysEUI: " : "-", pSysEUI->b[i]);
        }
    gCatena.SafePrintf("\n");
    pass();
    }

test(2_platform_20_lorawan_begin)
    {
    assertTrue(gfLoRaWANPassed, "gLoRaWAN.begin() failed");
    
    pass();
    }

test(2_platform_30_init_lux)
    {
    uint32_t flags = gCatena.GetPlatformFlags();
    bool fLtr329 = false;

    assertNotEqual(flags & Catena::fHasLuxSi1133, 0, "No light sensor in platform flags?");

    byte error, address;
    int nDevices;
    gCatena.SafePrintf("Scanning for LTR329ALS01\n");

    nDevices = 0;
    for(address = 1; address < 127; address++ )
        {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
            {
            if (address == 41)
                fLtr329 = true;

            nDevices++;
            }
        }
    if (nDevices == 0)
        gCatena.SafePrintf("No I2C devices found\n");
    else
        gCatena.SafePrintf("done\n");

    assertTrue(
        fLtr329,
        "LTR329ALS01 sensor not present"
        );

    pass();
    }

test(2_platform_40_init_SHT3x)
    {
    uint32_t flags = gCatena.GetPlatformFlags();

    assertNotEqual(flags & Catena::fHasBme280, 0, "No Temperature sensor in platform flags?");

    assertTrue(
        gSht3x.begin(),
        "SHT3x sensor failed begin()"
        );
    pass();

    }

//
// Platform peripheral testing
//
testing(3_platform_10_SHT3x)
    {
    skipIfFailed(2_platform_40_init_SHT3x);

    const uint32_t interval = 2000;
    const uint32_t ntries = 10;
    static uint32_t lasttime, thistry;
    uint32_t now = millis();

    if (lasttime != 0 && (int32_t)(now - lasttime) < interval)
        /* skip */;
    else
        {
        lasttime = now;

        cSHT3x::Measurements m;
        bool fResult = gSht3x.getTemperatureHumidity(m);

        assertTrue(
            fResult,
            "gSht3x.getTemperatureHumidity() failed"
            );
        Serial.print("SHT3x:  T: "); Serial.print(m.Temperature);
        Serial.print("  RH: "); Serial.print(m.Humidity); Serial.println("%");

        assertMore(m.Temperature, 10, "Temperature in lab must be > 10 deg C: " << m.Temperature);
        assertLess(m.Temperature, 40, "Temperature in lab must be < 40 deg C: " << m.Temperature);
        assertMore(m.Humidity, 5);
        assertLess(m.Humidity, 80);

        if (++thistry >= ntries)
            pass();
        }

    }

testing(3_platform_20_Lux)
    {
    skipIfFailed(2_platform_30_init_lux);

    bool fResult = gLTR329.readLux();

    assertTrue(
        fResult,
        "gLTR329.readLux() failed"
        );

    float lux = gLTR329.readLux();
    gCatena.SafePrintf(
            "LTR329: %d Lux\n",
            (int)lux
            );

    assertMore(lux, 250, "The Lux in lab must be > 250 lux: " << lux);
    assertLess(lux, 12500, "The Lux in lab must be < 12500 lux: " << lux);

    pass();
    }

static constexpr double getVbat_min() { return 2.2; }
static constexpr double getVbat_max() { return 4.3; }

testing(3_platform_70_vBat)
    {
    const uint32_t interval = 2000;
    const uint32_t ntries = 10;
    static uint32_t lasttime, thistry;
    uint32_t now = millis();

    if (lasttime != 0 && (now - lasttime) < interval)
        /* skip */;
    else
        {
        lasttime = now;

        float vBat = gCatena.ReadVbat();
        Serial.print("Vbat:   "); Serial.println(vBat);

        assertMore(vBat, getVbat_min());
        assertLess(vBat, getVbat_max());
        if (++thistry >= ntries)
                {
                fFlashDone = true;
                pass();
                }
        }
    }

testing(3_platform_99)
    {
    if (checkTestDone(3_platform_10_SHT3x) &&
        checkTestDone(3_platform_20_Lux) &&
        checkTestDone(3_platform_70_vBat) &&
        true /* for symmetry in cut/paste above */
        )
            {
            assertTestPass(3_platform_10_SHT3x);
            assertTestPass(3_platform_20_Lux);
            assertTestPass(3_platform_70_vBat);
            pass();
            }
    }

//-----------------------------------------------------
//      Network tests
//-----------------------------------------------------

// make sure we're provisioned.
testing(4_lora_00_provisioned)
    {
    if (! checkTestDone(3_platform_99))
            return;

    assertTrue(gLoRaWAN.IsProvisioned(), "Not provisioned yet");
    pass();
    }

// Send a confirmed uplink message
Arduino_LoRaWAN::SendBufferCbFn uplinkDone;

uint8_t noncePointer;
bool gfSuccess;
bool gfTxDone;
void *gpCtx;

uint8_t uplinkBuffer[] = { /* port */ 0x10, 0xCA, 0xFE, 0xF0, 0x0D };

const uint32_t kLoRaSendTimeout = 40 * 1000;

uint32_t gTxStartTime;

testing(4_lora_10_senduplink)
    {
    if (! checkTestDone(4_lora_00_provisioned))
        return;

    // set the clock tolerance
    LMIC_setClockError(10*65536/100);

    // send a confirmed message.
    if (! checkTestPass(4_lora_00_provisioned))
        {
        skip();
        return;
        }

    assertTrue(gLoRaWAN.SendBuffer(uplinkBuffer, sizeof(uplinkBuffer), uplinkDone, (void *) &noncePointer, true), "SendBuffer failed");
    gTxStartTime = millis();
    pass();
    }

void uplinkDone(void *pCtx, bool fSuccess)
    {
    gfTxDone = true;
    gfSuccess = fSuccess;
    gpCtx = pCtx;
    }

testing(4_lora_20_uplink_done)
    {
    if (checkTestSkip(4_lora_10_senduplink))
        {
        skip();
        return;
        }

    if (!checkTestDone(4_lora_10_senduplink))
        return;

    if (!checkTestPass(4_lora_10_senduplink))
        {
        skip();
        return;
        }

    if (! gfTxDone)
        {
        assertLess((int32_t)(millis() - gTxStartTime), kLoRaSendTimeout, "LoRaWAN transmit timed out");

        return;
        }

    assertTrue(gfSuccess, "Message uplink failed");
    assertTrue(gpCtx == (void *)&noncePointer, "Context pointer was wrong on callback");
    pass();
    }
