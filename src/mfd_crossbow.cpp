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
    fixType(3),
    batteryLastUpdated(0),
    battery_dV(0),
    battery_percent(0),
    linkLastUpdated(0),
    uplink_rssi_dbm(0),
    uplink_lq(0)
{
}

void
MFDCrossbow::SendBatteryTelemetry(uint8_t *rawCrsfPacket)
{
    // CRSF BATTERY_SENSOR payload (after type byte at index 2):
    //   [3..4] voltage   big-endian, 0.1 V
    //   [5..6] current   big-endian, 0.1 A
    //   [7..9] mAh used  big-endian
    //   [10]   remaining %
    battery_dV      = ((uint16_t)rawCrsfPacket[3] << 8) | rawCrsfPacket[4];
    battery_percent = rawCrsfPacket[10];
    batteryLastUpdated = millis();
}

void
MFDCrossbow::SendLinkTelemetry(uint8_t *rawCrsfPacket)
{
    // CRSF LINK_STATISTICS payload (after type byte at index 2):
    //   [3]  uplink_RSSI_1   negate for dBm
    //   [4]  uplink_RSSI_2
    //   [5]  uplink_LQ       % (0..100)
    //   [6]  uplink_SNR      int8
    //   ... downlink fields at [10..12]
    uplink_rssi_dbm = -(int8_t)rawCrsfPacket[3];
    uplink_lq       = rawCrsfPacket[5];
    linkLastUpdated = millis();
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
    bool linkFresh = (linkLastUpdated != 0) && (now - linkLastUpdated < 5000);
    bool batFresh  = (batteryLastUpdated != 0) && (now - batteryLastUpdated < 10000);

    // Single-page layout: 6 rows of 6x10 text, fills 60 of 64 px.
    m_oled->setFont(u8g2_font_6x10_tr);

    if (linkFresh)
        snprintf(buf, sizeof(buf), "TRK %s S:%-2d LQ:%-3d",
                 gpsFresh ? "LINK" : "----", gps_sats, uplink_lq);
    else
        snprintf(buf, sizeof(buf), "TRK %s S:%-2d LQ:--",
                 gpsFresh ? "LINK" : "----", gps_sats);
    m_oled->drawStr(0, 10, buf);

    snprintf(buf, sizeof(buf), "Lat %10.6f", lat / 1.0e7);
    m_oled->drawStr(0, 20, buf);
    snprintf(buf, sizeof(buf), "Lon %10.6f", lon / 1.0e7);
    m_oled->drawStr(0, 30, buf);
    snprintf(buf, sizeof(buf), "Alt %4.0fm Spd %4.1fm/s",
             alt / 1000.0f, groundspeed / 100.0f);
    m_oled->drawStr(0, 40, buf);
    snprintf(buf, sizeof(buf), "Hdg %5.1f  Fix %d",
             heading / 100.0f, fixType);
    m_oled->drawStr(0, 50, buf);

    if (batFresh && linkFresh)
        snprintf(buf, sizeof(buf), "Bat %2u.%uV %3u%% R%d",
                 battery_dV / 10, battery_dV % 10, battery_percent,
                 uplink_rssi_dbm);
    else if (batFresh)
        snprintf(buf, sizeof(buf), "Bat %2u.%uV %3u%% R---",
                 battery_dV / 10, battery_dV % 10, battery_percent);
    else if (linkFresh)
        snprintf(buf, sizeof(buf), "Bat --.-V  --%% R%d",
                 uplink_rssi_dbm);
    else
        snprintf(buf, sizeof(buf), "Bat --.-V  --%% R---");
    m_oled->drawStr(0, 60, buf);

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
