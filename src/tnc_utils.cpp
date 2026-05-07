/* Copyright (C) 2025 Ricardo Guzman - CA2RXU
 *
 * This file is part of LoRa APRS iGate.
 *
 * LoRa APRS iGate is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LoRa APRS iGate is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LoRa APRS iGate. If not, see <https://www.gnu.org/licenses/>.
 */

// ============================================================
//  MODIFICATIONS vs version originale CA2RXU :
//
//  Supprimé :
//    - wrapRNSFrame()       : ajoutait 0x60 + FCS AX.25 au payload
//    - ax25Fcs()            : calcul CRC-16 pour wrapRNSFrame
//    - decapsulateRNSKISS() : retirait le wrapper avant envoi LoRa
//
//  Résultat :
//    - Le port TCP RNS est un KISS transparent pur :
//        TCP→LoRa : payload extrait de la trame KISS, envoyé tel quel
//        LoRa→TCP : payload brut reçu de LoRa, encapsulé en KISS
//    - Aucun header Reticulum n'est ajouté ni interprété par l'ESP32
//    - Le debug série (rnsDebug) affiche les octets IN/OUT en hex
//      avec les labels [RNS IN] et [RNS OUT]
// ============================================================

#include <WiFi.h>
#include "ESPmDNS.h"
#include "configuration.h"
#include "station_utils.h"
#include "kiss_protocol.h"
#include "aprs_is_utils.h"
#include "kiss_utils.h"
#include "lora_utils.h"
#include "tnc_utils.h"
#include "utils.h"


extern Configuration    Config;
extern WiFiClient       aprsIsClient;
extern bool             passcodeValid;

// ---- Serveur TNC APRS existant ----------------------------
#define MAX_CLIENTS         4
#define INPUT_BUFFER_SIZE   (2 + MAX_CLIENTS)
#define TNC_PORT            8001

WiFiClient*     clients[MAX_CLIENTS];
WiFiServer      tncServer(TNC_PORT);
String          inputServerBuffer[INPUT_BUFFER_SIZE];
String          inputSerialBuffer = "";

// ---- Serveur TCP RNS (KISS transparent) -------------------
#define MAX_RNS_CLIENTS     2

WiFiServer*     rnsTncServer   = nullptr;
WiFiClient*     rnsClients[MAX_RNS_CLIENTS];
String          inputRNSBuffer[MAX_RNS_CLIENTS];


// -----------------------------------------------------------
//  Debug série RNS : affiche label + octets hex sur Serial
//  Appelé uniquement si Config.tnc.rnsDebug == true
// -----------------------------------------------------------
static void rnsDebugHex(const char* label, const uint8_t* data, size_t len) {
    Serial.print(label);
    for (size_t i = 0; i < len; i++) {
        if (data[i] < 0x10) Serial.print('0');
        Serial.print(data[i], HEX);
        if (i < len - 1) Serial.print(' ');
    }
    Serial.println();
}

namespace TNC_Utils {

    // =======================================================
    //  setup() — démarre TNC APRS + serveur TCP RNS
    // =======================================================
    void setup() {
        // --- Serveur TNC APRS existant ---
        if (Config.tnc.enableServer && Config.digi.ecoMode == 0) {
            tncServer.stop();
            tncServer.begin();
            String host = "igate-" + Config.callsign;
            if (!MDNS.begin(host.c_str())) {
                Serial.println("Error Starting mDNS");
                tncServer.stop();
                return;
            }
            if (!MDNS.addService("tnc", "tcp", TNC_PORT)) {
                Serial.println("Error: Could not add mDNS service");
            }
            Serial.println("TNC server started successfully");
            Serial.println("mDNS Host: " + host + ".local");
        }

        // --- Serveur TCP RNS (port dédié, KISS transparent) ---
        if (Config.tnc.rnsActive && Config.digi.ecoMode == 0) {
            if (rnsTncServer) {
                rnsTncServer->stop();
                delete rnsTncServer;
                rnsTncServer = nullptr;
            }
            rnsTncServer = new WiFiServer(Config.tnc.rnsPort);
            rnsTncServer->begin();
            MDNS.addService("tnc-rns", "tcp", Config.tnc.rnsPort);
            Serial.println("[RNS] TCP server started on port "
                           + String(Config.tnc.rnsPort));
            if (Config.tnc.rnsDebug) {
                Serial.println("[RNS] Debug mode ON");
            }
        }
    }


    // =======================================================
    //  Gestion des clients TNC APRS (inchangé)
    // =======================================================
    void checkNewClients() {
        WiFiClient new_client = tncServer.accept();
        if (new_client.connected()) {
            for (int i = 0; i < MAX_CLIENTS; i++) {
                if (clients[i] == nullptr) {
                    clients[i] = new WiFiClient(new_client);
                    Utils::println("New TNC client connected");
                    break;
                }
            }
        }
    }

    void handleInputData(char character, int bufferIndex) {
        String* data = (bufferIndex == -1)
                       ? &inputSerialBuffer
                       : &inputServerBuffer[bufferIndex];

        if (data->length() == 0 && character != (char)FEND) return;
        data->concat(character);

        if (character == (char)FEND && data->length() > 3) {
            bool isDataFrame = false;
            const String& frame = decodeKISS(*data, isDataFrame);
            if (isDataFrame) {
                if (bufferIndex != -1) {
                    Utils::print("<--- Got from TNC      : ");
                    Utils::println(frame);
                }
                String sender = frame.substring(0, frame.indexOf(">"));
                if (Config.tnc.acceptOwn || sender != Config.callsign) {
                    if (Config.loramodule.txActive)
                        STATION_Utils::addToOutputPacketBuffer(frame);
                    if (Config.tnc.aprsBridgeActive
                        && Config.aprs_is.active
                        && passcodeValid
                        && aprsIsClient.connected())
                        APRS_IS_Utils::upload(frame);
                } else {
                    Utils::println("Ignored own frame from KISS");
                }
            }
            data->clear();
        }
        if (data->length() > 255) data->clear();
    }

    void readFromClients() {
        for (int i = 0; i < MAX_CLIENTS; i++) {
            auto client = clients[i];
            if (client != nullptr) {
                if (client->connected()) {
                    while (client->available() > 0) {
                        handleInputData(client->read(), 2 + i);
                    }
                } else {
                    delete client;
                    clients[i] = nullptr;
                }
            }
        }
    }

    void readFromSerial() {
        while (Serial.available() > 0) {
            handleInputData(Serial.read(), -1);
        }
    }

    void sendToClients(const String& packet, bool stripBytes) {
        String cleanPacket = stripBytes ? packet.substring(3) : packet;
        const String kissEncoded = encodeKISS(cleanPacket);
        for (int i = 0; i < MAX_CLIENTS; i++) {
            auto client = clients[i];
            if (client != nullptr) {
                if (client->connected()) {
                    client->print(kissEncoded);
                    client->flush();
                } else {
                    delete client;
                    clients[i] = nullptr;
                }
            }
        }
        Utils::print("---> Sent to TNC       : ");
        Utils::println(cleanPacket);
    }

    void sendToSerial(const String& packet, bool stripBytes) {
        String cleanPacket = stripBytes ? packet.substring(3) : packet;
        Serial.print(encodeKISS(cleanPacket));
        Serial.flush();
    }


    // =======================================================
    //  Serveur TCP RNS — KISS transparent
    // =======================================================

    void checkNewRNSClients() {
        if (!rnsTncServer) return;
        WiFiClient new_client = rnsTncServer->accept();
        if (new_client.connected()) {
            for (int i = 0; i < MAX_RNS_CLIENTS; i++) {
                if (rnsClients[i] == nullptr) {
                    rnsClients[i] = new WiFiClient(new_client);
                    Serial.println("[RNS] Client connected");
                    break;
                }
            }
        }
    }

    // -------------------------------------------------------
    //  handleRNSInputData()
    //
    //  Reçoit les octets TCP octet par octet, reconstruit la
    //  trame KISS complète (FEND…FEND), extrait le payload
    //  (désescaping FESC/TFEND/TFESC) et l'envoie tel quel
    //  sur LoRa via sendRawPacket().
    //
    //  Seule la commande DATA (0x00) est traitée.
    //  Toute autre commande est ignorée silencieusement
    //  (les commandes RNode étendues tombent ici aussi).
    // -------------------------------------------------------
    void handleRNSInputData(char character, int bufferIndex) {
        String* buf = &inputRNSBuffer[bufferIndex];

        // Attendre le premier FEND
        if (buf->length() == 0 && character != (char)FEND) return;
        buf->concat(character);

        // Trame complète : FEND...FEND avec au moins 3 octets intérieurs
        if (character == (char)FEND && buf->length() > 3) {

            // Vérifier que c'est une commande DATA (octet 1 = 0x00)
            if (buf->charAt(1) == (char)CMD_DATA) {

                // --- Désescaping KISS ---
                // La trame brute est : FEND CMD_DATA [escaped payload] FEND
                // On itère de l'index 2 jusqu'à len-2 (exclut les deux FEND)
                size_t rawLen = buf->length();
                uint8_t* payload = new uint8_t[rawLen]; // taille max
                size_t   payLen  = 0;

                for (size_t i = 2; i < rawLen - 1; i++) {
                    uint8_t b = (uint8_t)buf->charAt(i);
                    if (b == FESC) {
                        i++;
                        if (i < rawLen - 1) {
                            uint8_t next = (uint8_t)buf->charAt(i);
                            if (next == TFEND)      payload[payLen++] = FEND;
                            else if (next == TFESC) payload[payLen++] = FESC;
                            // octet invalide après FESC : ignoré
                        }
                    } else {
                        payload[payLen++] = b;
                    }
                }

                if (payLen > 0) {
                    // Debug IN : payload brut avant envoi LoRa
                    if (Config.tnc.rnsDebug) {
                        rnsDebugHex("[RNS IN ] ", payload, payLen);
                    }

                    // Envoi LoRa brut (sendRawPacket utilise uint8_t*
                    // avec longueur explicite → safe pour binaire)
                    if (Config.loramodule.txActive) {
                        // Construire un String pour sendRawPacket
                        // (API existante) sans troncature null
                        String loraPacket;
                        loraPacket.reserve(payLen);
                        for (size_t i = 0; i < payLen; i++) {
                            loraPacket.concat((char)payload[i]);
                        }
                        LoRa_Utils::sendRawPacket(loraPacket);
                    }
                }
                delete[] payload;

            } else {
                // Commande non-DATA : ignorée
                if (Config.tnc.rnsDebug) {
                    uint8_t cmd = (uint8_t)buf->charAt(1);
                    Serial.print("[RNS] Ignoring non-DATA cmd: 0x");
                    if (cmd < 0x10) Serial.print('0');
                    Serial.println(cmd, HEX);
                }
            }

            buf->clear();
        }

        // Protection débordement
        if (buf->length() > 512) {
            if (Config.tnc.rnsDebug) {
                Serial.println("[RNS] RX buffer overflow, cleared");
            }
            buf->clear();
        }
    }

    void readFromRNSClients() {
        for (int i = 0; i < MAX_RNS_CLIENTS; i++) {
            auto client = rnsClients[i];
            if (client != nullptr) {
                if (client->connected()) {
                    while (client->available() > 0) {
                        handleRNSInputData((char)client->read(), i);
                    }
                } else {
                    delete client;
                    rnsClients[i] = nullptr;
                    Serial.println("[RNS] Client disconnected");
                }
            }
        }
    }

    // -------------------------------------------------------
    //  sendToRNSClients()
    //
    //  Appelé depuis LoRa_APRS_iGate.cpp quand un paquet
    //  non-APRS est reçu sur LoRa (bloc else du main loop).
    //
    //  Encapsule le payload brut LoRa dans une trame KISS
    //  standard (FEND CMD_DATA [escaped payload] FEND) et
    //  l'envoie à tous les clients RNS connectés.
    //
    //  Utilise uint8_t* + longueur explicite pour éviter
    //  toute troncature sur les payloads binaires contenant
    //  des octets nuls.
    // -------------------------------------------------------
    void sendToRNSClients(const String& rawPacket) {
        size_t payloadLen = rawPacket.length();
        if (payloadLen == 0) return;

        const uint8_t* payload =
            reinterpret_cast<const uint8_t*>(rawPacket.c_str());

        // Debug OUT : payload brut reçu de LoRa
        if (Config.tnc.rnsDebug) {
            rnsDebugHex("[RNS OUT] ", payload, payloadLen);
        }

        // --- Construire la trame KISS avec escaping ---
        // Taille max : FEND + CMD_DATA + 2*payload (full escape) + FEND
        size_t   maxFrameLen = 2 + 2 * payloadLen + 1;
        uint8_t* frame       = new uint8_t[maxFrameLen];
        size_t   frameLen    = 0;

        frame[frameLen++] = FEND;
        frame[frameLen++] = CMD_DATA;   // 0x00

        for (size_t i = 0; i < payloadLen; i++) {
            uint8_t b = payload[i];
            if (b == FEND) {
                frame[frameLen++] = FESC;
                frame[frameLen++] = TFEND;
            } else if (b == FESC) {
                frame[frameLen++] = FESC;
                frame[frameLen++] = TFESC;
            } else {
                frame[frameLen++] = b;
            }
        }
        frame[frameLen++] = FEND;

        // Debug : trame KISS complète
        if (Config.tnc.rnsDebug) {
            rnsDebugHex("[RNS KISS frame] ", frame, frameLen);
        }

        // --- Envoi à tous les clients connectés ---
        for (int i = 0; i < MAX_RNS_CLIENTS; i++) {
            auto client = rnsClients[i];
            if (client != nullptr) {
                if (client->connected()) {
                    client->write(frame, frameLen);
                    client->flush();
                } else {
                    delete client;
                    rnsClients[i] = nullptr;
                }
            }
        }

        delete[] frame;
    }


    // =======================================================
    //  loop() — boucle principale TNC + RNS
    // =======================================================
    void loop() {
        if (Config.digi.ecoMode == 0) {
            if (Config.tnc.enableServer) {
                checkNewClients();
                readFromClients();
            }
            if (Config.tnc.enableSerial) {
                readFromSerial();
            }
            if (Config.tnc.rnsActive) {
                checkNewRNSClients();
                readFromRNSClients();
            }
        }
    }

} // namespace TNC_Utils
