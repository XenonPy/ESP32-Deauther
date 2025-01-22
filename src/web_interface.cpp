#include <WebServer.h>
#include "web_interface.h"
#include "definitions.h"
#include "deauth.h"

WebServer server(80);
int num_networks;

// Move the function declaration to the top
String getEncryptionType(wifi_auth_mode_t encryptionType);

void redirect_root() {
  server.sendHeader("Location", "/");
  server.send(301);
}

void handle_root() {
  String html = R"(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>XePy's Deauther</title>
    <style>
        /* General Styles */
body {
    font-family: 'Courier New', Courier, monospace;
    line-height: 1.6;
    color: #33ff33; /* Hacker green */
    max-width: 800px;
    margin: 0 auto;
    padding: 20px;
    background-color: #000; /* Black background */
}

/* Headings */
h1, h2 {
    color: #00ff00; /* Bright green */
    font-weight: bold;
}

/* Table Styling */
table {
    width: 100%;
    border-collapse: collapse;
    margin: 20px 0;
    background: #111; /* Darker background */
    border-radius: 5px;
    overflow: hidden;
    box-shadow: 0 2px 4px rgba(0, 255, 0, 0.2);
}

th, td {
    padding: 12px;
    text-align: left;
    border-bottom: 1px solid #33ff33;
    color: #33ff33;
}

th {
    background-color: #004400; /* Dark green */
    color: #00ff00;
}

/* Alternating row colors */
tr:nth-child(even) {
    background-color: #002200; /* Slightly lighter dark green */
}

/* Form Styling */
form {
    background-color: #111;
    padding: 20px;
    border-radius: 5px;
    box-shadow: 0 2px 5px rgba(0, 255, 0, 0.2);
    margin-bottom: 20px;
    border: 1px solid #33ff33;
}

/* Inputs and Buttons */
input[type="text"], input[type="submit"] {
    width: 100%;
    padding: 10px;
    margin-bottom: 10px;
    border: 1px solid #33ff33;
    border-radius: 4px;
    background-color: #000;
    color: #33ff33;
    font-size: 16px;
}

input[type="submit"] {
    background-color: #004400;
    color: #00ff00;
    border: none;
    cursor: pointer;
    transition: background-color 0.3s, box-shadow 0.3s;
}

input[type="submit"]:hover {
    background-color: #006600;
    box-shadow: 0 0 10px #00ff00;
}

    </style>
</head>
<body>
    <h1>XePy's Deauther</h1>
    
    <h2>Targets List</h2>
    <table>
        <tr>
            <th>Number</th>
            <th>SSID</th>
            <th>BSSID</th>
            <th>Channel</th>
            <th>RSSI</th>
            <th>Encryption</th>
        </tr>
)";

  for (int i = 0; i < num_networks; i++) {
    String encryption = getEncryptionType(WiFi.encryptionType(i));
    html += "<tr><td>" + String(i) + "</td><td>" + WiFi.SSID(i) + "</td><td>" + WiFi.BSSIDstr(i) + "</td><td>" + 
            String(WiFi.channel(i)) + "</td><td>" + String(WiFi.RSSI(i)) + "</td><td>" + encryption + "</td></tr>";
  }

  html += R"(
    </table>

    <form method="post" action="/rescan">
        <input type="submit" value="Rescan networks">
    </form>

    <form method="post" action="/deauth">
        <h2>Launch Deauth-Attack</h2>
        <input type="text" name="net_num" placeholder="Network Number">
        <input type="text" name="reason" placeholder="Reason code">
        <input type="submit" value="Launch Attack">
    </form>

    <p>Eliminated stations: )" + String(eliminated_stations) + R"(</p>

    <form method="post" action="/deauth_all">
        <h2>Deauth all Networks</h2>
        <input type="text" name="reason" placeholder="Reason code">
        <input type="submit" value="Deauth All">
    </form>

    <form method="post" action="/stop">
        <input type="submit" value="Stop Deauth-Attack">
    </form>

    <h2>Reason Codes</h2>
    <table>
        <tr>
            <th>Code</th>
            <th>Meaning</th>
        </tr>
        <tr><td>0</td><td>Reserved.</td></tr>
        <tr><td>1</td><td>Unspecified reason.</td></tr>
        <tr><td>2</td><td>Previous authentication no longer valid.</td></tr>
        <tr><td>3</td><td>Deauthenticated because sending station (STA) is leaving or has left Independent Basic Service Set (IBSS) or ESS.</td></tr>
        <tr><td>4</td><td>Disassociated due to inactivity.</td></tr>
        <tr><td>5</td><td>Disassociated because WAP device is unable to handle all currently associated STAs.</td></tr>
        <tr><td>6</td><td>Class 2 frame received from nonauthenticated STA.</td></tr>
        <tr><td>7</td><td>Class 3 frame received from nonassociated STA.</td></tr>
        <tr><td>8</td><td>Disassociated because sending STA is leaving or has left Basic Service Set (BSS).</td></tr>
        <tr><td>9</td><td>STA requesting (re)association is not authenticated with responding STA.</td></tr>
        <tr><td>10</td><td>Disassociated because the information in the Power Capability element is unacceptable.</td></tr>
        <tr><td>11</td><td>Disassociated because the information in the Supported Channels element is unacceptable.</td></tr>
        <tr><td>12</td><td>Disassociated due to BSS Transition Management.</td></tr>
        <tr><td>13</td><td>Invalid element, that is, an element defined in this standard for which the content does not meet the specifications in Clause 8.</td></tr>
        <tr><td>14</td><td>Message integrity code (MIC) failure.</td></tr>
        <tr><td>15</td><td>4-Way Handshake timeout.</td></tr>
        <tr><td>16</td><td>Group Key Handshake timeout.</td></tr>
        <tr><td>17</td><td>Element in 4-Way Handshake different from (Re)Association Request/ Probe Response/Beacon frame.</td></tr>
        <tr><td>18</td><td>Invalid group cipher.</td></tr>
        <tr><td>19</td><td>Invalid pairwise cipher.</td></tr>
        <tr><td>20</td><td>Invalid AKMP.</td></tr>
        <tr><td>21</td><td>Unsupported RSNE version.</td></tr>
        <tr><td>22</td><td>Invalid RSNE capabilities.</td></tr>
        <tr><td>23</td><td>IEEE 802.1X authentication failed.</td></tr>
        <tr><td>24</td><td>Cipher suite rejected because of the security policy.</td></tr>
    </table>
</body>
</html>
)";

  server.send(200, "text/html", html);
}


void handle_deauth() {
  int wifi_number = server.arg("net_num").toInt();
  uint16_t reason = server.arg("reason").toInt();

  String html = R"(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Attack</title>
    <style>
        /* General Styles */
body {
    font-family: 'Courier New', Courier, monospace;
    display: flex;
    justify-content: center;
    align-items: center;
    height: 100vh;
    margin: 0;
    background-color: #000; /* Black background */
    color: #33ff33; /* Hacker green */
}

/* Alert Box */
.alert {
    background-color: #004400; /* Dark green */
    color: #00ff00; /* Bright green text */
    padding: 20px;
    border-radius: 5px;
    box-shadow: 0 2px 5px rgba(0, 255, 0, 0.3);
    text-align: center;
    border: 1px solid #33ff33;
}

/* Error Alert */
.alert.error {
    background-color: #440000; /* Dark red */
    color: #ff3333; /* Bright red for errors */
    border: 1px solid #ff3333;
}

/* Button Styling */
.button {
    display: inline-block;
    padding: 10px 20px;
    margin-top: 20px;
    background-color: #003300; /* Dark green */
    color: #00ff00; /* Hacker green text */
    text-decoration: none;
    border-radius: 5px;
    border: 1px solid #33ff33;
    transition: background-color 0.3s, box-shadow 0.3s;
    font-size: 16px;
}

.button:hover {
    background-color: #006600;
    box-shadow: 0 0 10px #00ff00;
}

    </style>
</head>
<body>
    <div class="alert)";

  if (wifi_number < num_networks) {
    html += R"(">
        <h2>Starting Deauth-Attack!</h2>
        <p>Deauthenticating network number: )" + String(wifi_number) + R"(</p>
        <p>Reason code: )" + String(reason) + R"(</p>
    </div>)";
    start_deauth(wifi_number, DEAUTH_TYPE_SINGLE, reason);
  } else {
    html += R"( error">
        <h2>Error: Invalid Network Number</h2>
        <p>Please select a valid network number.</p>
    </div>)";
  }

  html += R"(
    <a href="/" class="button">Back to Home</a>
</body>
</html>
  )";

  server.send(200, "text/html", html);
}

void handle_deauth_all() {
  uint16_t reason = server.arg("reason").toInt();

  String html = R"(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Deauth All Networks</title>
    <style>
        /* General Styles */
body {
    font-family: 'Courier New', Courier, monospace;
    display: flex;
    justify-content: center;
    align-items: center;
    height: 100vh;
    margin: 0;
    background-color: #000; /* Black background */
    color: #33ff33; /* Hacker green text */
}

/* Alert Box */
.alert {
    background-color: #004400; /* Dark green */
    color: #00ff00; /* Bright green text */
    padding: 20px;
    border-radius: 5px;
    box-shadow: 0 2px 5px rgba(0, 255, 0, 0.3);
    text-align: center;
    border: 1px solid #33ff33;
}

/* Button Styling */
.button {
    display: inline-block;
    padding: 10px 20px;
    margin-top: 20px;
    background-color: #003300; /* Dark green */
    color: #00ff00; /* Hacker green text */
    text-decoration: none;
    border-radius: 5px;
    border: 1px solid #33ff33;
    transition: background-color 0.3s, box-shadow 0.3s;
    font-size: 16px;
}

.button:hover {
    background-color: #006600;
    box-shadow: 0 0 10px #00ff00;
}

    </style>
</head>
<body>
    <div class="alert">
        <h2>Starting Deauth-Attack on All Networks!</h2>
        <p>WiFi will shut down now. To stop the attack, please reset the ESP32.</p>
        <p>Reason code: )" + String(reason) + R"(</p>
    </div>
</body>
</html>
  )";

  server.send(200, "text/html", html);
  server.stop();
  start_deauth(0, DEAUTH_TYPE_ALL, reason);
}

void handle_rescan() {
  num_networks = WiFi.scanNetworks();
  redirect_root();
}

void handle_stop() {
  stop_deauth();
  redirect_root();
}

void start_web_interface() {
  server.on("/", handle_root);
  server.on("/deauth", handle_deauth);
  server.on("/deauth_all", handle_deauth_all);
  server.on("/rescan", handle_rescan);
  server.on("/stop", handle_stop);

  server.begin();
}

void web_interface_handle_client() {
  server.handleClient();
}

// The function implementation can stay where it is
String getEncryptionType(wifi_auth_mode_t encryptionType) {
  switch (encryptionType) {
    case WIFI_AUTH_OPEN:
      return "Open";
    case WIFI_AUTH_WEP:
      return "WEP";
    case WIFI_AUTH_WPA_PSK:
      return "WPA_PSK";
    case WIFI_AUTH_WPA2_PSK:
      return "WPA2_PSK";
    case WIFI_AUTH_WPA_WPA2_PSK:
      return "WPA_WPA2_PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE:
      return "WPA2_ENTERPRISE";
    default:
      return "UNKNOWN";
  }
}
