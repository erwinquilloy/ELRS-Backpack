#include "mfd_crossbow.h"
#include "common/mavlink.h"
#include "common.h"

#if defined(OLED_SSD1306)
#include <Wire.h>
#endif

MFDCrossbow::MFDCrossbow(HardwareSerial *port) :
#if defined(OLED_SSD1306)
    m_oled(nullptr),
    displayLastDrawn(0),
#endif
    m_port(port),
    gpsLastSent(0),
    gpsLastUpdated(0),
    heading(0),
    lat(0.0),
    lon(0.0),
    alt(0.0),
    groundspeed(0.0),
    gps_sats(0),
    gps_alt(0),
    gps_hdop(100),
    fixType(3)
{
}

void
MFDCrossbow::Init()
{
#if defined(OLED_SSD1306)
    InitDisplay();
#endif
}

#if defined(OLED_SSD1306)
void
MFDCrossbow::InitDisplay()
{
#if defined(HELTEC_VEXT)
    // Heltec gates external 3V3 (and the OLED, on some revs) via Vext on
    // GPIO 21 — active LOW. Generic ESP32+OLED setups skip this.
    pinMode(HELTEC_VEXT, OUTPUT);
    digitalWrite(HELTEC_VEXT, LOW);
    delay(20);
#endif

#if defined(OLED_RST)
    // Boards that wire the OLED reset to a GPIO need an explicit pulse.
    // Standalone SSD1306 modules have their own RC reset and skip this.
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(50);
    digitalWrite(OLED_RST, HIGH);
    delay(50);
#endif

    Wire.begin(OLED_SDA, OLED_SCL);

    m_oled = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
    m_oled->setBusClock(400000);
    m_oled->begin();

    // Force the brightness-related SSD1306 registers to their maximum so
    // dim panels (like Heltec V2) are still legible:
    //   0x8D 0x14 — enable internal charge pump
    //   0xD9 0xF1 — pre-charge period: phase1=1, phase2=15 (max)
    //   0xDB 0x30 — VCOMH deselect level high (brightest)
    //   0x81 0xFF — contrast max
    u8x8_t *u8x8 = m_oled->getU8x8();
    u8x8_cad_StartTransfer(u8x8);
    u8x8_cad_SendCmd(u8x8, 0x8D); u8x8_cad_SendArg(u8x8, 0x14);
    u8x8_cad_SendCmd(u8x8, 0xD9); u8x8_cad_SendArg(u8x8, 0xF1);
    u8x8_cad_SendCmd(u8x8, 0xDB); u8x8_cad_SendArg(u8x8, 0x30);
    u8x8_cad_SendCmd(u8x8, 0x81); u8x8_cad_SendArg(u8x8, 0xFF);
    u8x8_cad_EndTransfer(u8x8);

    m_oled->setPowerSave(0);

#if defined(OLED_INVERT)
    // Lit background, dark text — used on dim panels to maximize light
    // output. External modules don't need this.
    m_oled->sendF("c", 0xA7);
#endif

    m_oled->setFont(u8g2_font_7x13B_tr);
    m_oled->clearBuffer();
    m_oled->drawStr(0, 12, "ELRS Tracker");
    m_oled->drawStr(0, 28, "MFD Crossbow");
    m_oled->sendBuffer();
}

void
MFDCrossbow::DrawDisplay(uint32_t now)
{
    if (!m_oled || now - displayLastDrawn < 250)
        return;
    displayLastDrawn = now;

    char buf[24];

    m_oled->clearBuffer();
    m_oled->setFont(u8g2_font_7x13B_tr);

    if (connectionState == wifiUpdate)
    {
        m_oled->drawStr(0, 12, "WiFi Update");
        m_oled->setFont(u8g2_font_6x10_tr);
        m_oled->drawStr(0, 26, "SSID: ExpressLRS");
        m_oled->drawStr(0, 36, "      VRx Backpack");
        m_oled->drawStr(0, 48, "Pass: expresslrs");
        m_oled->drawStr(0, 60, "URL:  10.0.0.1");
        m_oled->sendBuffer();
        return;
    }

    if (connectionState == binding)
    {
        m_oled->drawStr(0, 14, "Binding...");
        m_oled->setFont(u8g2_font_6x10_tr);
        m_oled->drawStr(0, 30, "Bind from your TX");
        m_oled->drawStr(0, 42, "with the same");
        m_oled->drawStr(0, 54, "binding phrase");
        m_oled->sendBuffer();
        return;
    }

    bool gpsFresh = (gps_sats > 0) && (now - gpsLastUpdated < 10000);

    // Header — bold and prominent
    snprintf(buf, sizeof(buf), "TRK %s S:%d",
             gpsFresh ? "LINK" : "----", gps_sats);
    m_oled->drawStr(0, 13, buf);

    // Body — small font, 4 lines of telemetry
    m_oled->setFont(u8g2_font_6x10_tr);
    snprintf(buf, sizeof(buf), "Lat %10.6f", lat / 1.0e7);
    m_oled->drawStr(0, 26, buf);
    snprintf(buf, sizeof(buf), "Lon %10.6f", lon / 1.0e7);
    m_oled->drawStr(0, 38, buf);
    snprintf(buf, sizeof(buf), "Alt %5.0fm  Spd %4.1fm/s",
             alt / 1000.0f, groundspeed / 100.0f);
    m_oled->drawStr(0, 50, buf);
    snprintf(buf, sizeof(buf), "Hdg %5.1f deg  Fix %d",
             heading / 100.0f, fixType);
    m_oled->drawStr(0, 62, buf);

    m_oled->sendBuffer();
}
#endif

void
MFDCrossbow::SendHeartbeat()
{
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
    // Pack the message
    mavlink_msg_heartbeat_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, MAVLINK_SYSTEM_TYPE, MAVLINK_AUTOPILOT_TYPE, MAVLINK_SYSTEM_MODE, MAVLINK_CUSTOM_MODE, MAVLINK_SYSTEM_STATE);
  
    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
    // Send the message
    m_port->write(buf, len);
}

void
MFDCrossbow::SendGpsRawInt()
{
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    const uint16_t eph = UINT16_MAX;
    const uint16_t epv = UINT16_MAX;
    const uint16_t cog = UINT16_MAX;
    const uint32_t alt_ellipsoid = 0;
    const uint32_t h_acc = 0;
    const uint32_t v_acc = 0;
    const uint32_t vel_acc = 0;
    const uint32_t hdg_acc = 0;

    // Pack the message
    mavlink_msg_gps_raw_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, MAVLINK_UPTIME, fixType, lat, lon, alt, eph, epv, groundspeed, cog, gps_sats, alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc, heading);
    
    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
    // Send the message
    m_port->write(buf, len);
}

void
MFDCrossbow::SendGlobalPositionInt()
{
    const int16_t velx = 0;
    const int16_t vely = 0;
    const int16_t velz = 0;

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_global_position_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, MAVLINK_UPTIME, lat, lon, gps_alt, alt, velx, vely, velz, heading);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message
    m_port->write(buf, len);
}

void
MFDCrossbow::SendGpsTelemetry(crsf_packet_gps_t *packet)
{
    int32_t rawLat = be32toh(packet->p.lat); // Convert to host byte order
    int32_t rawLon = be32toh(packet->p.lon); // Convert to host byte order
    lat = static_cast<double>(rawLat);
    lon = static_cast<double>(rawLon);

    // Convert from CRSF scales to mavlink scales
    groundspeed = be16toh(packet->p.speed) / 36.0 * 100.0;
    heading = be16toh(packet->p.heading);
    alt = (be16toh(packet->p.altitude) - 1000) * 1000.0;
    gps_alt = alt;
    gps_sats = packet->p.satcnt;

    // Send heartbeat and GPS mavlink messages to the tracker
    SendHeartbeat();
    SendGpsRawInt();
    SendGlobalPositionInt();

    // Log the last time we received new GPS coords
    gpsLastUpdated = millis();
}

void
MFDCrossbow::Loop(uint32_t now)
{
    ModuleBase::Loop(now);

    // If the GPS data is <= 10 seconds old, keep spamming it out at 10hz
    bool gpsIsValid = (now < gpsLastUpdated + 10000) && gps_sats > 0;

    if (now > gpsLastSent + 100 && gpsIsValid)
    {
        SendHeartbeat();
        SendGpsRawInt();
        SendGlobalPositionInt();

        gpsLastSent = now;
    }

#if defined(OLED_SSD1306)
    DrawDisplay(now);
#endif
}
