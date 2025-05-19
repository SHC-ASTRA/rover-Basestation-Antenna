/**
 * @file main.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @brief description
 *
 */

//------------//
//  Includes  //
//------------//

#include <Arduino.h>
#include <LSS.h>
#include <SPI.h>
#include <Ethernet.h>

#include "AstraMisc.h"
#include "AstraSensors.h"
#include "AntennaMainMCU.h"


//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);

const unsigned int localPort = 8888;  // local port to listen on for UDP packets


//---------------------//
//  Component classes  //
//---------------------//

LSS myLSS(0);

SFE_UBLOX_GNSS myGNSS;

Adafruit_BNO055 bno;

EthernetUDP Udp;


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

long lastAlignment = 0;
long lastRoverPos = 0;
long roverlat = 0;
long roverlon = 0;


//--------------//
//  Prototypes  //
//--------------//

float calcHeading(double mylat, double mylon, double targetlat, double targetlon);


//------------------------------------------------------------------------------------------------//
//  Setup
//------------------------------------------------------------------------------------------------//
//
//
//------------------------------------------------//
//                                                //
//      ////////    //////////    //////////      //
//    //                //        //        //    //
//    //                //        //        //    //
//      //////          //        //////////      //
//            //        //        //              //
//            //        //        //              //
//    ////////          //        //              //
//                                                //
//------------------------------------------------//
void setup() {
    //--------//
    //  Pins  //
    //--------//

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);


    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);
    Serial1.begin(LSS_BAUD);

    Ethernet.init(ETHERNET_CS);
    Ethernet.begin(mac, ip);


    //-----------//
    //  Sensors  //
    //-----------//

    if(!bno.begin()) 
        Serial.println("!BNO failed to start...");
    else 
        Serial.println("BNO055 Started Successfully");

    if(!myGNSS.begin()) 
        Serial.println("GPS not working");
    else 
        Serial.println("GPS is working");

    // Setup for GPS (copied directly from Core)

    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGNSS.setNavigationFrequency(30);
    // Create storage for the time pulse parameters
    UBX_CFG_TP5_data_t timePulseParameters;

    // Get the time pulse parameters
    if (myGNSS.getTimePulseParameters(&timePulseParameters) == false)
    {
        Serial.println(F("getTimePulseParameters failed! not Freezing..."));
    }

    // Print the CFG TP5 version
    Serial.print(F("UBX_CFG_TP5 version: "));
    Serial.println(timePulseParameters.version);

    timePulseParameters.tpIdx = 0; // Select the TIMEPULSE pin
    //timePulseParameters.tpIdx = 1; // Or we could select the TIMEPULSE2 pin instead, if the module has one

    // We can configure the time pulse pin to produce a defined frequency or period
    // Here is how to set the frequency:

    // While the module is _locking_ to GNSS time, make it generate 2kHz
    timePulseParameters.freqPeriod = 2000; // Set the frequency/period to 2000Hz
    timePulseParameters.pulseLenRatio = 0x55555555; // Set the pulse ratio to 1/3 * 2^32 to produce 33:67 mark:space

    // When the module is _locked_ to GNSS time, make it generate 1kHz
    timePulseParameters.freqPeriodLock = 1000; // Set the frequency/period to 1000Hz
    timePulseParameters.pulseLenRatioLock = 0x80000000; // Set the pulse ratio to 1/2 * 2^32 to produce 50:50 mark:space

    timePulseParameters.flags.bits.active = 1; // Make sure the active flag is set to enable the time pulse. (Set to 0 to disable.)
    timePulseParameters.flags.bits.lockedOtherSet = 1; // Tell the module to use freqPeriod while locking and freqPeriodLock when locked to GNSS time
    timePulseParameters.flags.bits.isFreq = 1; // Tell the module that we want to set the frequency (not the period)
    timePulseParameters.flags.bits.isLength = 0; // Tell the module that pulseLenRatio is a ratio / duty cycle (* 2^-32) - not a length (in us)
    timePulseParameters.flags.bits.polarity = 1; // Tell the module that we want the rising edge at the top of second. (Set to 0 for falling edge.)

    // Now set the time pulse parameters
    if (myGNSS.setTimePulseParameters(&timePulseParameters) == false)
    {
        Serial.println(F("setTimePulseParameters failed!"));
    }
    else
    {
        Serial.println(F("Success!"));
    }


    //--------------------//
    //  Misc. Components  //
    //--------------------//

    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found.");
    }
    if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
    }

    Udp.begin(localPort);
}


//------------------------------------------------------------------------------------------------//
//  Loop
//------------------------------------------------------------------------------------------------//
//
//
//-------------------------------------------------//
//                                                 //
//    /////////      //            //////////      //
//    //      //     //            //        //    //
//    //      //     //            //        //    //
//    ////////       //            //////////      //
//    //      //     //            //              //
//    //       //    //            //              //
//    /////////      //////////    //              //
//                                                 //
//-------------------------------------------------//
void loop() {
    //----------//
    //  Timers  //
    //----------//
#ifdef BLINK
    if (millis() - lastBlink > 1000) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }
#endif

    if (millis() - lastAlignment > 250 && millis() - lastRoverPos < 5000) {  // Requires update <5 seconds ago
        lastAlignment = millis();

        // Get lat/lon from GNSS
        double gps_data[3] = {0};
        getPosition(myGNSS, gps_data);
        double mylat = gps_data[0];
        double mylon = gps_data[1];

        // Calculate required heading to point antenna at rover
        float requiredHeading = calcHeading(mylat, mylon, roverlat, roverlon);

        // Get heading measurement from IMU
        float currentHeading = getBNOOrient(bno);

        // Make LSS rotate towards the required heading
        if (abs(currentHeading - requiredHeading) < 5) {  // arbitrary tolerance
            myLSS.wheel(0);  // stop if within tolerance
        } else if (currentHeading < requiredHeading) {
            myLSS.wheel(10);
        } else if (currentHeading > requiredHeading) {
            myLSS.wheel(-10);
        }
    }


    //-------------//
    //  UDP Input  //
    //-------------//
    
    char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet

    int packetSize = Udp.parsePacket();
    if (packetSize) {
        Serial.print("Received packet of size ");
        Serial.println(packetSize);
        Serial.print("From ");
        IPAddress remote = Udp.remoteIP();
        for (int i = 0; i < 4; i++) {
            Serial.print(remote[i], DEC);
            if (i < 3) {
                Serial.print(".");
            }
        }
        Serial.print(", port ");
        Serial.println(Udp.remotePort());

        // read the packet into packetBuffer
        Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
        Serial.println("Contents:");
        Serial.println(packetBuffer);
    }


    //------------------//
    //  UART/USB Input  //
    //------------------//
    //
    //
    //-------------------------------------------------------//
    //                                                       //
    //      /////////    //\\        ////    //////////      //
    //    //             //  \\    //  //    //        //    //
    //    //             //    \\//    //    //        //    //
    //    //             //            //    //        //    //
    //    //             //            //    //        //    //
    //    //             //            //    //        //    //
    //      /////////    //            //    //////////      //
    //                                                       //
    //-------------------------------------------------------//
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');

        input.trim();                   // Remove preceding and trailing whitespace
        std::vector<String> args = {};  // Initialize empty vector to hold separated arguments
        parseInput(input, args);   // Separate `input` by commas and place into args vector
        args[0].toLowerCase();          // Make command case-insensitive
        String command = args[0];       // To make processing code more readable

        //--------//
        //  Misc  //
        //--------//
        if (command == "ping") {
            Serial.println("pong");
        }

        else if (command == "time") {
            Serial.println(millis());
        }

        else if (command == "led") {
            if (args[1] == "on")
                digitalWrite(LED_BUILTIN, HIGH);
            else if (args[1] == "off")
                digitalWrite(LED_BUILTIN, LOW);
            else if (args[1] == "toggle") {
                ledState = !ledState;
                digitalWrite(LED_BUILTIN, ledState);
            }
        }

        //-----------//
        //  Sensors  //
        //-----------//

        //----------//
        //  Motors  //
        //----------//
    }
}


//------------------------------------------------------------------------------------------------//
//  Function definitions
//------------------------------------------------------------------------------------------------//
//
//
//----------------------------------------------------//
//                                                    //
//    //////////    //          //      //////////    //
//    //            //\\        //    //              //
//    //            //  \\      //    //              //
//    //////        //    \\    //    //              //
//    //            //      \\  //    //              //
//    //            //        \\//    //              //
//    //            //          //      //////////    //
//                                                    //
//----------------------------------------------------//

float calcHeading(double mylat, double mylon, double targetlat, double targetlon) {
    double x = cos(targetlat) * sin(targetlon - mylon);
    double y = cos(mylat) * sin(targetlat) - sin(mylat) * cos(targetlat) * cos(targetlon - mylon);

    return atan2(x, y) * 180 / M_PI;  // Convert to degrees
}
