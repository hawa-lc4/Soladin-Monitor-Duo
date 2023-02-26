/*
   hawa-lc4 09.01.2023
   Board: Arduino Nano V3.0
*/
/*
   Soladin 600 Monitor
   Programm zur Abfrage und Anzeige der Betriebsdaten der Soladin 600 Solar-Einspeisewechselrichter.
   Anzeige auf TFT Display 1,8" mit 128x160 Pixel über SPI Schnittstelle;
     Querformat: 20 Zeichen bei 8 Zeilen; Zeichen: b:8, h:16 Pixel

   Diese Variante ausschließlich zur Darstellung der Daten von zwei WR wie ich es verwende.
   Ausgangspunkt ist die Single-Variante 1.41s; jetzt 1.50s

  07.04.19:
  - für mein Display brauch ich den aufwändigen DoDpClr, daher fets eingebaut.
  - für die Fonts ucg_font_8x13_mf und ucg_font_8x13_mr muß jede Zeile bei Pixel 1 beginnen!
    Das liegt aber nur am Pixel-Versatzfehler des Displays!

  10.04.19
  - Setup sollte fertig sein
  - bin jetzt mitten in der loop-Routine
  - alles abspecken was nicht wirklich wichtig ist!
  - Kann auch als Vorlage für die Single-Version verwendet werden.
  - Ausgaben der Betriebsdaten über seriellen Monitor auch einfach automatisch machen.
  - Ausgabe der Hostory-Daten auf Anfrage; Taste "h" gedrückt

  12.04.19:
  - Setup ist fertig
  - Ausgabe der Betriebsdaten auch ok
  - bin mitten in den History Daten.
  - Zeit für Anzeige vergrößern? (interval1)

  13.04.2019:
  - Umstellung auf HW-SPI
  - Reduzierung RAM durch Anzeigetexte in PROGMEM "F()"
  - Änderung des Abfrage-Tasters auf D12; Reset des Displays über Reset des Arduino

  15.04.2019:
  - Umbau auf HW-SPI
  - Änderung der Ausgaben seriell und am Display zu "F()" um RAM zu sparen: perfekt gelungen!

  27.02.2022:
  - Vertauschen der Kanäle solcom1 und solcom2 für meine neue Einspeisung "Klein-PV".

  29./30.03.2022:
  - Erweiterung der Kommandos nur zur Abfrage der AC Leistung. Ziel: mein futro oder bpi-M2
    können dann die Leistung des zweiten WR abschätzen um den bei genügend Last einzuschalten
    oder bei zuviel Einspeisung wieder auszuschalten.
  - da jeder Zugriff auf die USB Schnittstelle der Arduino einen Reset auslöst muß ein
    SoftwareSerial "mySerial" für die Kommunikation mit futro/bpi-M2 her.
  - Ausgabe der AC-Power nur auf Anfrage mit Kommando "a" auf mySerial; Ausgaben am USB
    belassen.

  21.04.2022: "v0.63d"
  - wenn ein Wechselrichter fehlt gibt das Programm "-1" zurück als Signal für den Haupt-
    controller sich "schlafen zu legen".
  
  01.01.2023: "v0.64d"
  - Anpassung in main.cpp bei Ausgaben der "TotalOperaTime"; %04d ==> %04ld (Compiler-Warnmeldung)
  - Anpassung in Soladin.cpp, Zeile 41: "return false"  eingefügt wegen Compiler Warnmeldung.
  - Verlegung des "Display Command Data (A0)" auf den Pin A0 um D9 (PWM fähig) freizubekommen
    für die PV-Control.
  - Änderung der Routine zur Datenabfrage der WR über futro oder BPI-M2:
    futro/BPI-M2 sendet über mySerial ein Byte mit dem PWM Wert der dann auf D9 ausgegeben wird.
    Danach werden dann die aktuellen Daten für AC-Power und Gesamtenergie der WR an futro/BPI-M2
    gesandt. Implementierung der Funktionen zusammen mit wall-E.
  - Umbau auf non-blocking code für wall-E; alle delay im Bereich der "loop" müssen raus.
    
  09.01.2023: "v0.86d"
  - Versionssprung zur Unterscheidung der Versionen in der Arduino-IDE.
  - Ausgaben über den seriellen Kanal (Monitor) nur noch auf Anfrage um Zeit zu sparen.
  - die Zeitschleifen des non-blocking code sind noch nicht fertig!

  21.01.2023: "v0.89d"
  - sol_Gridpower wird immer mit "0" geliefert wenn die WR abgeschaltet sind.
  - Speicherung des letzten Stand von sol_Totalpower im EEPROM um einen Wert liefern zu können 
    auch wenn ein Neustart in der Nacht stattfand. Es ist noch nicht klar ob das sauber läuft.



  Dokumentationen:
  Zum Display:
   - https://github.com/Bodmer/TFT_ST7735/issues/1   Pixel-Versatz am Rand von Display-a
   - TFT-Display-a_1.8Z-ST7735.odt  bzw.  TFT-Display-b_1.8Z-ST7735.odt
   - zum Font: https://github.com/olikraus/ucglib/wiki/fontgroupunifont
     bzw. https://github.com/olikraus/ucglib/wiki/fontgroupx11

  Zum Soladin 600:
   - readme.txt in  Arduino\libraries\SolaDin-master (Achtung auf Korrektur der Pin-Bezeichnung)
   - Kommandos:
  Serial.println("f - Firmware");
  Serial.println("r - Read Max power");
  Serial.println("o - Reset Max power");
  Serial.println("h - Read history data");
  Serial.println("d - Read current device status");
  Serial.println("a - Read AC-Power only");
  Serial.println("x - Write RXbuffer");  // ist hier nicht sinnvoll einsetzbar, wurde entfernt
  -- zusätzlich und modifiziert für Daueranzeige ist der "Read device status"
*/


/*
   Definition der Variablen und Namen/Ports für I/O
*/
// includes:
#include <Arduino.h>
#include <EEPROMWearLevel.h>
#include <Soladin.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "Ucglib.h"

#define SysVer "v0.89d"     // System-Version; immer anpassen!!!
#define NoOfSol 2           // Soladin Monitor Duo = 2 Soladin WR; das hier wird auch nur mit 2 funktionieren!!!!
#define DispA0 A0           // Display Command Data (A0)
#define DispCS 8            // Display Chip Select
#define DispLED 10          // braucht PWM zur Helligkeitssteuerung
#define HDQ A1              // HDQ = History Data Query; Taster-input
#define PVc 9               // PWM Ausgang zur PV-Control
#define Ln1 14              // Pixel-Position der 1. Zeile; und so weiter....
#define Ln2 30
#define Ln3 46
#define Ln4 62
#define Ln5 78
#define Ln6 94
#define Ln7 110
#define Ln8 126
#define EEPROM_LAYOUT_VERSION 0
#define AMOUNT_OF_INDEXES 2
#define IDX_VAR1 0
#define IDX_VAR2 1

// Belegung der analogen und digitalen I/O-PINs:
//
// A0 = ?                         // Output: Display Command Data (A0)
// A1 = ?                         // Input: Abfrage Taster History-Data (HDQ)
// A2 = ?                         //
// A3 = ?                         //
// A4 und A5 bleiben reserviert für mögliche I2C Erweiterungen
// A6 = ?           (nur analog!) //
// A7 = ?           (nur analog!) //
//
// D0 und D1 bleiben frei für USB-Schnittstelle (Rx/Tx)
// D2 = ?                         // Soladin-2 Tx-data
// D3 = ? (PWM)                   // Soladin-2 Rx-data
// D4 = ?                         // Soladin-1 Tx-data
// D5 = ? (PWM)                   // Soladin-1 Rx-data
// D6 = ? (PWM)                   // mySerial Tx
// D7 = ?                         // mySerial Rx
// D8 = ?                         // Display Chip select
// D9 = ?                   (PWM) // PWM Signal zur PV-Control
// D10 = ?                  (PWM) // (PWM) Display LED Helligkeit
// D11 = ?           (MOSI) (PWM) // Display SPI data signal
// D12 = ?           (MISO)       // HWSPI
// D13 = ?      (SCK)(LED an GND) // Display SPI clock signal

uint32_t timeNow = 0;             // Merker für die aktuelle Zeit
uint32_t prevQuery = 0;           // speichert die Zeit des letzten Abfrage der WR
uint32_t prevDisp = 0;            // speichert die Zeit des letzten Anzeige
uint32_t prevBtn = 0;             // speichert die Zeit des letzten Tastendrucks
const uint32_t interval1 = 2000;      // Interval für Abfrage Soladin in Millisekunden
const uint32_t interval2 = 1800000;   // Display-LED in Millisekunden (30 Minuten)
boolean normalDS = true;          // normale Daten-Anzeige oder History Daten? (Taster-Abfrage)
byte HDcount = 1;                 // Zähler für die Durchläufe der HD Datenanzeige
byte iii = 0;
byte Lnx = 0;                     // Hilfsvariable zur Zeilenberechnung
byte doSU;                        // Zähler "do setup"; Zeichen 's' muß 3x in direkter Folge gesendet werden

boolean sol_present[NoOfSol];
uint16_t sol_FW_ID[NoOfSol];
uint16_t sol_FW_version[NoOfSol];
uint16_t sol_FW_date[NoOfSol];
uint16_t sol_PVvolt[NoOfSol];
uint16_t sol_PVamp[NoOfSol];
uint16_t sol_Gridpower[NoOfSol];
uint32_t sol_Totalpower[NoOfSol];
uint32_t sol_TotalpowerSaved[NoOfSol];
uint32_t sol_TotalOpTime[NoOfSol];
uint16_t sol_Flag[NoOfSol];
const uint32_t savedTotalPowS1 = 179274;
const uint32_t savedTotalPowS2 = 190270;
uint16_t dataLength1 = sizeof(savedTotalPowS1);
uint16_t dataLength2 = sizeof(savedTotalPowS2);

SoftwareSerial solcom1(5, 4);     // serial to conect to Soladin#1 Rx=5 Tx=4
SoftwareSerial solcom2(3, 2);     // serial to conect to Soladin#2 Rx=3 Tx=2
SoftwareSerial mySerial(7, 6);    // serial to conect to futro or bpi-M2 or ESP8266 wbec; Rx=7 Tx=6
Soladin sol;                      // copy of soladin class
Ucglib_ST7735_18x128x160_HWSPI TFT(/*cd=*/ DispA0, /*cs=*/ DispCS, /*reset=*/ -1);


/*
Routinen für serielle Ausgabe am Monitor-Kanal (HW-Serial)
*/

void SPrintflag() {
  if ( sol.Flag & 0x0001 )Serial.println(F(" Usolar too high"));
  if ( sol.Flag & 0x0002 )Serial.println(F(" Usolar too low"));
  if ( sol.Flag & 0x0004 )Serial.println(F(" No Grid"));
  if ( sol.Flag & 0x0008 )Serial.println(F(" Uac too high"));
  if ( sol.Flag & 0x0010 )Serial.println(F(" Uac too low"));
  if ( sol.Flag & 0x0020 )Serial.println(F(" Fac too high"));
  if ( sol.Flag & 0x0040 )Serial.println(F(" Fac too low"));
  if ( sol.Flag & 0x0080 )Serial.println(F(" Temperature to high"));
  if ( sol.Flag & 0x0100 )Serial.println(F(" Hardware failure"));
  if ( sol.Flag & 0x0200 )Serial.println(F(" Starting"));
  if ( sol.Flag & 0x0400 )Serial.println(F(" Max Power"));
  if ( sol.Flag & 0x0800 )Serial.println(F(" Max current"));
}

void SPrintDS(boolean k) {
  if (!k){Serial.println(F(":  missing")); return;} 
  Serial.print(F(":\r\nPV= "));
  Serial.print(float(sol.PVvolt) / 10);
  Serial.print(F("V;   "));
  Serial.print(float(sol.PVamp) / 100);
  Serial.println(F("A"));

  Serial.print(F("AC= "));
  Serial.print(sol.Gridpower);
  Serial.print(F("W;  "));
  Serial.print(float(sol.Gridfreq) / 100);
  Serial.print(F("Hz;  "));
  Serial.print(sol.Gridvolt);
  Serial.println(F("Volt"));

  Serial.print(F("Device Temperature= "));
  Serial.print(sol.DeviceTemp);
  Serial.println(F("° Celcius"));

  Serial.print(F("AlarmFlag= "));
  Serial.println(sol.Flag, HEX);
  if (sol.Flag != 0x00) SPrintflag();

  Serial.print(F("Total Power= "));
  Serial.print(float(sol.Totalpower) / 100);
  Serial.println(F("kWh"));
  // I really don't know, wy i have to split the sprintf ?
  Serial.print(F("Total Operating Time= "));
  char timeStr[14];
  sprintf(timeStr, "%04ld:", (sol.TotalOperaTime / 60));
  Serial.print(timeStr);
  sprintf(timeStr, "%02ld hh:mm ",  (sol.TotalOperaTime % 60));
  Serial.println(timeStr);
  Serial.println();
}

void SPrintFW(byte j, boolean k) {
  Serial.print(F("Soladin-"));
  Serial.println(j + 1);
  Serial.print(F("FW ID= "));
  Serial.print(sol_FW_ID[j], HEX);
  Serial.print(F("   Ver= "));
  Serial.print(sol_FW_version[j], HEX);
  Serial.print(F("   Date= "));
  Serial.println(sol_FW_date[j], HEX);
  Serial.println();
  if (k) {
    Serial.print(F("sol_present= ")); Serial.println(sol_present[j]);
    Serial.print(F("sol_PVvolt= ")); Serial.println(sol_PVvolt[j]);
    Serial.print(F("sol_PVamp= ")); Serial.println(sol_PVamp[j]);
    Serial.print(F("sol_Gridpower= ")); Serial.println(sol_Gridpower[j]);
    Serial.print(F("sol_Totalpower= ")); Serial.println(sol_Totalpower[j]);
    Serial.print(F("sol_TotalOpTime= ")); Serial.println(sol_TotalOpTime[j]);
    Serial.print(F("sol_Flag= ")); Serial.println(sol_Flag[j], HEX);
    Serial.print("sol_TotalpowerSaved= "); Serial.println(sol_TotalpowerSaved[j]);
    if (j == 0) {
      int IdxStart1 = EEPROMwl.getStartIndexEEPROM(IDX_VAR1);
      int IdxCurr1 = EEPROMwl.getCurrentIndexEEPROM(IDX_VAR1, dataLength1);
      Serial.print("sol_TotalpowerSaved written: ");
      Serial.print((IdxCurr1 - IdxStart1 + dataLength1) / dataLength1);
    }
    if (j == 1) {
      int IdxStart2 = EEPROMwl.getStartIndexEEPROM(IDX_VAR2);
      int IdxCurr2 = EEPROMwl.getCurrentIndexEEPROM(IDX_VAR2, dataLength2);
      Serial.print("sol_TotalpowerSaved written: ");
      Serial.print((IdxCurr2 - IdxStart2 + dataLength2) / dataLength2);
    }
    Serial.println(" times\n");
  }
  
}

void doHD1(byte j) {     // gibt die history data seriell aus
  if(!sol_present[j]){Serial.println(F(": missing")); return;}
  Serial.println(F(": History Data"));
  for (byte ii = 0 ; ii < 10 ; ii++) {  // loop this for the last 10 days
    if (sol.query(HSD, ii)) {        // Read history data
      Serial.print(ii * -1);
      Serial.print(F(": Op. Time= "));
      char timeStr[14];
      sprintf(timeStr, "%02d:%02d hh:mm  ", (sol.DailyOpTm / 12), ((sol.DailyOpTm % 12) * 5));
      Serial.print(timeStr);
      Serial.print(float(sol.Gridoutput) / 100);
      Serial.println(F("kWh"));
    }
  }
}

void SPrintMenu(){
  Serial.println(F("\n------menu------------"));
  Serial.println(F("m - print data in memory"));
  Serial.println(F("d - read device status"));
  Serial.println(F("h - read history data"));
  Serial.println(F("o - show mySerial output string"));
  Serial.println(F("s - goto Setup! 3x 's'"));
  Serial.println();
}

void SPrintMSOut(){
  if (sol_present[0]){
    Serial.print(sol_Gridpower[0]);
  } else{
    Serial.print("0");
  }
  Serial.print(";");
  Serial.print(sol_Totalpower[0]);
  Serial.print(";");
  if (sol_present[1]){
    Serial.print(sol_Gridpower[1]);
  } else{
    Serial.print("0");
  }
  Serial.print(";");
  Serial.print(sol_Totalpower[1]);
  Serial.print(";!\n");
}


/*
Routinen für die Ausgabe am Display
*/

void DispHead(){
    TFT.clearScreen();
    TFT.setPrintPos(1, Ln1);
    TFT.setColor(0, 199, 255);            // Zeichenfarbe cyan für Überschrift
    TFT.print(F("Soladin 600  Monitor")); // Überschrift
    TFT.drawHLine(0, 15, 160);            // unterstrichen
}

void doHD2(byte j, byte k) {  // gibt die history data am Display aus
  if (k == 1){
    DispHead();
    TFT.setColor(255, 255, 255);            // Zeichenfarbe weiß
    TFT.setPrintPos(1, Ln2);
    TFT.print(F("Daten der letzten 10"));
    TFT.setPrintPos(1, Ln3);
    TFT.print(F("Tage:  "));
  }

  if (j == 1) TFT.setColor(255, 127, 127);       // Zeichenfarbe hellrot für Soladin-1
  if (j == 2) TFT.setColor(127, 255, 127);       // Zeichenfarbe hellgrün für Soladin-2
  TFT.print(F("Soladin-"));
  TFT.print(j);
  TFT.print("    ");
  static int i = 0;
  for (i = (k - 1) * 5; i < (k * 5); i++) {  // loop this for the last 10 days
    if (sol.query(HSD, i)) {            // Read history data
      if (i < 5) {
        Lnx = i * 16;
        Lnx += Ln4;
      } else {
        Lnx = i - 5;
        Lnx *= 16;
        Lnx += Ln4;
      }
      TFT.setPrintPos(1, Lnx);
      TFT.print(F("-"));
      TFT.print(i);
      TFT.print(F(": "));
      char timeStr[14];
      sprintf(timeStr, "%02d:%02d  ", (sol.DailyOpTm / 12), ((sol.DailyOpTm % 12) * 5)); // so stimmt's
      TFT.print(timeStr);
      TFT.print(float(sol.Gridoutput) / 100, 1);
      TFT.print(F("kWh      "));
    }
  }
}

void DisplDS(byte j) {
  byte Pp = 1;
  if (j == 0){
    TFT.setColor(255, 127, 127);        // Zeichenfarbe hellrot für Soladin-1
    Lnx = Ln3;
  } else {
    TFT.setColor(127, 255, 127);        // Zeichenfarbe hellgrün für Soladin-2
    Lnx = Ln6;
    Pp = 89;
  }
  if (!sol_present[j]){
    TFT.setPrintPos(1, Lnx);
    TFT.print("Soladin-");
    TFT.print(j + 1);
    TFT.print(" fehlt");
    return;
  }
  TFT.setPrintPos(Pp, Ln2);
  TFT.print(F("AC"));
  TFT.print(j + 1);
  TFT.print(F(": "));
  TFT.print(sol_Gridpower[j]);
  TFT.print(F("W   "));

  TFT.setPrintPos(1, Lnx);
  TFT.print(F("PV"));
  TFT.print(j + 1);
  TFT.print(F(": "));
  TFT.print(int((sol_PVvolt[j]) / 10));
  TFT.print(F("V*"));
  TFT.print(float(sol_PVamp[j]) / 100, 1);
  TFT.print(F("A= "));
  int PVa1 = (sol_PVamp[j]);
  TFT.print(word(((sol_PVvolt[j]) / 10) * PVa1 / 100));
  TFT.print(F("W    "));

  TFT.setPrintPos(1, Lnx + 16);
  TFT.print((char)187);             // Achtung: Font; Zeichen  >>
  TFT.print(j + 1);
  TFT.print(F(": "));
  TFT.print(float(sol_Totalpower[j]) / 100, 0);
  TFT.print(F("kWh  "));
  char timeStr[14];
  sprintf(timeStr, "%04ldh ", (sol_TotalOpTime[j] / 60));
  TFT.print(timeStr);

  TFT.setColor(255, 0, 255);         // Zeichenfarbe magenta für Alarm-Flags
  TFT.setPrintPos(1, Lnx + 32);
  TFT.print(j + 1);
  TFT.print(F("! Alarmflag: "));
  TFT.print(sol_Flag[j], HEX);
}

void DispSetup1(byte j){
  Lnx = j * 32;
  TFT.setPrintPos(1, Ln4 + Lnx);
  if (j == 0) TFT.setColor(255, 127, 127);       // Zeichenfarbe hellrot für Soladin-1
  if (j == 1) TFT.setColor(127, 255, 127);       // Zeichenfarbe hellgrün für Soladin-2
  TFT.print("Soladin-");
  TFT.print(j + 1);
  if (sol_present[j]){
      TFT.print(F(" verbunden"));
      TFT.setPrintPos(1, Ln5 + Lnx);
      TFT.print(F("FW-Version: "));
      TFT.print(sol_FW_version[j], HEX);
  } else {
      TFT.print(F(" fehlt"));
  }
  if (j == 1){
      TFT.setPrintPos(1, Ln8);
      TFT.setColor(255, 255, 255);            // Zeichenfarbe weiß
      TFT.print(F("Setup ... fertig."));
  }
}

void disp_setup(){
  // initialisiere Display:
  TFT.begin(UCG_FONT_MODE_SOLID);         // damit ist überschreiben von Zeichen möglich
  //  TFT.setFont(ucg_font_unifont_mr);   // Zeile beginnt bei Pixel 0; kein °C! size: 1680
  //TFT.setFont(ucg_font_8x13_mf);          // Zeile beginnt bei Pixel 1; hat mehr Zeichen! size: 3153
  TFT.setFont(ucg_font_8x13_mr);        // Zeile beginnt bei Pixel 1; hat weniger Zeichen! size: 1519

  //TFT.clearScreen();                      // der sollte normalerweise reichen!
  TFT.setRotate90(); TFT.clearScreen();   // aber für mein Display-a brauche ich leider diesen Aufwand
  TFT.setRotate180(); TFT.clearScreen();  // aber für mein Display-a brauche ich leider diesen Aufwand
  TFT.setRotate270(); TFT.clearScreen();  // aber für mein Display-a brauche ich leider diesen Aufwand
  TFT.setColor(1, 0, 0, 0);               // Hintergrund schwarz

  DispHead();
  TFT.setColor(255, 255, 255);            // Zeichenfarbe weiß
  TFT.setPrintPos(1, Ln2);
  TFT.print("Sys.Version: ");
  TFT.print(SysVer);

  TFT.setPrintPos(1, Ln3);
  TFT.print("Verbindungsaufbau: ");
}


/*
Routinen für die Datenspeicherung im EEPROM
*/

void setup_EEPROMcheckData() {
  EEPROMwl.begin(EEPROM_LAYOUT_VERSION, AMOUNT_OF_INDEXES);
  EEPROMwl.printStatus(Serial);
  if (EEPROMwl.getCurrentIndexEEPROM(IDX_VAR1, dataLength1) < 0) EEPROMwl.put(IDX_VAR1, savedTotalPowS1);
  if (EEPROMwl.getCurrentIndexEEPROM(IDX_VAR2, dataLength2) < 0) EEPROMwl.put(IDX_VAR2, savedTotalPowS2);
  EEPROMwl.get(IDX_VAR1, sol_TotalpowerSaved[0]);
  EEPROMwl.get(IDX_VAR2, sol_TotalpowerSaved[1]);
  sol_Totalpower[0] = sol_TotalpowerSaved[0];
  sol_Totalpower[1] = sol_TotalpowerSaved[1];
}

void loop_setEEPROMdata(byte j) {
  if (sol_Totalpower[j] > (sol_TotalpowerSaved[j] + 100)) {     // schreibe in's EEPROM nur wenn +1kWh
    if (j == 0) EEPROMwl.put(IDX_VAR1, sol_Totalpower[j]);
    if (j == 1) EEPROMwl.put(IDX_VAR2, sol_Totalpower[j]);
  }
}


/*
Hauptprogramm
*/

void setup() {
  Serial.begin(57600);
  Serial.println();
  Serial.println(F("Soladin 600 Monitor"));
  Serial.print(F("## Sys.Version: "));
  Serial.println(SysVer);
  Serial.println(F("Setup..."));

  solcom1.begin(9600);
  solcom2.begin(9600);
  mySerial.begin(38400);

  // initialisiere PINs:
  // pinMode(DispA0, OUTPUT);             // braucht's das??? das ist Teil des SPI
  pinMode(HDQ, INPUT_PULLUP);
  pinMode(DispLED, OUTPUT);
  analogWrite(DispLED, 196);              // Display hell
  disp_setup();
  setup_EEPROMcheckData();

  for (iii = 0; iii < NoOfSol; iii++){
    if (iii == 0){solcom1.listen(); sol.begin(&solcom1);}
    if (iii == 1){solcom2.listen(); sol.begin(&solcom2);}
    sol_present[iii] = sol.query(FWI);
    sol_FW_ID[iii] = sol.FW_ID;
    sol_FW_version[iii] = sol.FW_version;
    sol_FW_date[iii] = sol.FW_date;
    Serial.print("Probe Soladin-");
    Serial.print(iii + 1);
    DispSetup1(iii);
    if (sol_present[iii]) {
      Serial.println(F(" Connected"));
      SPrintFW(iii, false);
    } else {
      Serial.println(F(" missing"));
    }
  }

  SPrintMenu();
  doSU = 0;
  delay(3333);  // den delay im setup lassen wir mal durchgehen  ;)
  DispHead();
  Serial.println(F("Setup ... done\n"));
} // end setup


void loop() {
  if (!mySerial.isListening()) mySerial.listen();  // muß rein sonst liest mySerial nicht mit.  echt jetzt??
  timeNow = millis();
  if (timeNow - prevQuery >= interval1) {            // Wartezeit abgelaufen; jetzt neue Abfrage der WR

    for (iii = 0; iii < NoOfSol; iii++){
      if (iii == 0){solcom1.listen(); sol.begin(&solcom1);}
      if (iii == 1){solcom2.listen(); sol.begin(&solcom2);}
      sol_present[iii] = sol.query(DVS);
      // Serial.println("\n >>> Soladin-" + (iii + 1));
      // SPrintDS(sol_present[iii]);
      if (sol_present[iii]){
        sol_Gridpower[iii] = sol.Gridpower;
        sol_PVvolt[iii] = sol.PVvolt;
        sol_PVamp[iii] = sol.PVamp;
        if (sol_Totalpower[iii] < sol.Totalpower && (sol_Totalpower[iii] + 500) > sol.Totalpower) { // keine Sprünge größer +5kWh
          sol_Totalpower[iii] = sol.Totalpower;
          loop_setEEPROMdata(iii);
        }
        sol_TotalOpTime[iii] = sol.TotalOperaTime;
        sol_Flag[iii] = sol.Flag;
      } else {
        sol_Gridpower[iii] = 0;
        sol_PVvolt[iii] = 0;
        sol_PVamp[iii] = 0.0;
      }
    }

    if (sol_present[0]){
      mySerial.print(sol_Gridpower[0]);
    } else{
      mySerial.print("0");
    }
    mySerial.print(";");
    mySerial.print(sol_Totalpower[0]);
    mySerial.print(";");
    if (sol_present[1]){
      mySerial.print(sol_Gridpower[1]);
    } else{
      mySerial.print("0");
    }
    mySerial.print(";");
    mySerial.print(sol_Totalpower[1]);
    mySerial.print(";!\n");
    // SPrintMSOut();   // debug!!

    prevQuery = millis();
  }

  if (timeNow - prevDisp >= interval1 * 5){
    if (!normalDS) {                      // hier die Ausgabe der Daten Historie am Display
      if (HDcount == 1){
        if (!sol_present[0]){
            HDcount += 2;
          } else {
            solcom1.listen();
            sol.begin(&solcom1);
            doHD2(1, 1);
            HDcount++;
          }
      } else if (HDcount == 2){
        solcom1.listen();
          sol.begin(&solcom1);
          doHD2(1, 2);
          HDcount++;
      } else if (HDcount == 3){
        if (!sol_present[1]){
          HDcount += 2;
        } else {
          solcom2.listen();
          sol.begin(&solcom2);
          doHD2(2, 1);
          HDcount++;
        }
      } else if (HDcount == 4){
        solcom2.listen();
        sol.begin(&solcom2);
        doHD2(2, 2);
        HDcount++;
      } else {
        HDcount = 1;
        normalDS = true;
      }
    } else {                              // ab hier Ausgabe der Betriebsdaten
      DispHead();
      DisplDS(0);
      DisplDS(1);
    }

    //mySerial.print("\r");                 // muß rein denn während der Routine wird das
    // empfangene Zeichen verschluckt und das Partnerprogramm wartet sonst ewig. Nur für futro!
    prevDisp = millis();
  }

  if (timeNow - prevBtn >= interval2) analogWrite(DispLED, 16);   // Display dunkler
  if (!digitalRead(HDQ)) {                // Taster wurde gedrückt (=GND)
    analogWrite(DispLED, 196);            // Display hell
    TFT.setColor(255, 0, 0);             // Zeichenfarbe rot
    TFT.setPrintPos(145, Ln8);
    TFT.print("??");
    normalDS = false;
    prevBtn = millis();
  }

  if (Serial.available() > 0) {                   // read serial comands
    char incomingByte = Serial.read();
    switch (incomingByte) {
      //case 'f': break;                          // print stored firmware data
      //case 'r': doMP(); break;                  // read max power
      case 'm':                                   // show data in memory and variables
        for (iii = 0; iii < NoOfSol; iii++){
          SPrintFW(iii, true);
          doSU = 0;
        }
        break;
      case 'd':                                   // read current device status
        solcom1.listen(); sol.begin(&solcom1);
        Serial.print("\n >>> Soladin-1");
        SPrintDS(sol.query(DVS));
        solcom2.listen(); sol.begin(&solcom2);
        Serial.print("\n >>> Soladin-2");
        SPrintDS(sol.query(DVS));
        doSU = 0;
        break;
      case 'o':                                   // show mySerial output string
        SPrintMSOut();
        doSU = 0;
        break;
      case 'h':                                   // read historical data
        solcom1.listen(); sol.begin(&solcom1);
        Serial.print("\n >>> Soladin-1");
        doHD1(sol.query(PRB));
        solcom2.listen(); sol.begin(&solcom2);
        Serial.print("\n >>> Soladin-2");
        doHD1(sol.query(PRB));
        doSU = 0;
        break;
      case 's':                                   // Software Reset
        doSU++;
        Serial.print(F("Achtung: Reset durch Aufruf von setup!  #"));
        Serial.println(doSU);
        if (doSU == 3) {delay(500); setup();}
        break;
      default:
        SPrintMenu();
        doSU = 0;
        break;
    }
  }

  if (mySerial.available() > 0 ) {        // read mySerial data for PWM
    byte incomingByte[64];
    mySerial.readBytes(incomingByte, 1);
    Serial.println(incomingByte[0]);   // debug!
    analogWrite(PVc, incomingByte[0]);
  }
} // end loop


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Universal uC Color Graphics Library

  Copyright (c) 2014, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  Redistributions of source code must retain the above copyright notice, this list
  of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and / or other
  materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
