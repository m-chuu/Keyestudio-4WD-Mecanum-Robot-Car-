
<h1>🤖 Mobile Robot &amp; Robotic Arm</h1>
<p class="subtitle">Autonomous Pick, Sort &amp; Store System — Team Banana Robotics · Jun–Jul 2026</p>

<p>
A two-robot autonomous system: a <strong>mecanum-wheel mobile robot</strong> (Arduino UNO) follows a
line-marked course, finds and grips colour-coded blocks, and delivers them to a
<strong>6-DOF robotic arm</strong> (ESP32) that identifies each block with a HuskyLens camera and sorts
it into its storage bay. The two robots coordinate through
<a href="https://platform.favoriot.com">Favoriot</a>, Malaysia's homegrown IoT platform.
</p>

<p>
🎥 <a href="https://www.youtube.com/shorts/NQkrVoJGTZc">Demo 1 — Path Navigation</a> ·
<a href="https://www.youtube.com/shorts/l49opY_GzNk">Demo 2 — Full Pick, Sort &amp; Store Sequence</a><br>
💻 <a href="https://github.com/m-chuu/Keyestudio-4WD-Mecanum-Robot-Car-">GitHub repository</a>
</p>

<h2>System Architecture</h2>
<pre><code>IR Remote ─┐                          Favoriot Cloud (HTTPS / MQTT)
           ▼                            ▲                ▲
┌─────────────────────┐  SoftwareSerial │                │ poll streams
│  Arduino UNO        │◄───────────────►│                │
│  Mecanum Car        │        ┌────────┴───────┐  ┌─────┴──────────────┐
│  · 3× IR line       │        │ ESP32 WiFi     │  │ ESP32 Robot Arm    │
│  · Ultrasonic       │        │ Bridge         │  │ · PCA9685 6 servos │
│  · TCS3200 colour   │        │ (esp32_main)   │  │ · HuskyLens (UART) │
│  · Gripper servo    │        └────────────────┘  │ · S-curve motion   │
└─────────────────────┘                            └────────────────────┘</code></pre>

<ul>
  <li><strong>UNO mecanum car</strong> (<code>src/uno_main.cpp</code>) — line tracking, mission sequencing, object detection &amp; gripping.</li>
  <li><strong>ESP32 bridge</strong> (<code>extras/esp32_main.cpp</code>) — WiFi/Favoriot link; forwards cloud commands to the UNO and posts task-status streams.</li>
  <li><strong>ESP32 robot arm</strong> (<code>extras/robot-arm/Robot-Arm-Final.cpp</code>) — polls Favoriot for <code>*_complete</code> signals, scans the delivered block's AprilTag/QR with HuskyLens, and runs the matching pick-and-place pose sequence.</li>
</ul>

<h2>Mobile Robot Features</h2>
<ul>
  <li><strong>Line following</strong> — 3-channel IR array (left/mid/right) with proportional speed correction on each wheel's PWM; junction counting tracks progress along the course.</li>
  <li><strong>Sequence engine</strong> — missions are declarative <code>Step</code> tables stored in flash (<code>PROGMEM</code>): move-forward-N-markers, 90°/180° rotations, drifts, timed reverses, grabs, delays.</li>
  <li><strong>Fault recovery</strong> — timed moves give up after a timeout (<code>MOVE_FWD_TIMEOUT_MS</code>) and a following <code>REVERSE_IF_TIMEOUT</code> step backs the robot off obstructions.</li>
  <li><strong>Object approach</strong> — ultrasonic ranging slows the robot inside 10&nbsp;cm and stops at 4&nbsp;cm before colour checking.</li>
  <li><strong>Colour selection</strong> — TCS3200 pulse readings classify BLUE / RED / YELLOW; the robot only grips the mission's target colour.</li>
  <li><strong>Gripper care</strong> — servo detaches (goes limp) when open to reduce strain, stays powered while carrying a load.</li>
  <li><strong>Triple e-stop</strong> — IR remote ★ button, cloud STOP command, both checked inside every movement loop.</li>
</ul>

<h2>Controls</h2>
<table>
  <tr><th>Trigger</th><th>Action</th></tr>
  <tr><td>IR <strong>1</strong> / <strong>3</strong></td><td>Fixed left-side / right-side grab-and-deliver path</td></tr>
  <tr><td>IR <strong>2</strong> (or cloud n/a)</td><td>Full three-position mission: middle, left, then right grab-and-deliver runs</td></tr>
  <tr><td>IR <strong>4</strong> / cloud <code>CMD4</code></td><td>Search &amp; deliver the <strong>BLUE</strong> block (middle → left → right), then post <code>task4_complete</code></td></tr>
  <tr><td>IR <strong>5</strong> / cloud <code>CMD5</code></td><td>Search &amp; deliver the <strong>RED</strong> block, then post <code>task5_complete</code></td></tr>
  <tr><td>IR <strong>6</strong> / cloud <code>CMD6</code></td><td>Search &amp; deliver the <strong>YELLOW</strong> block, then post <code>task6_complete</code></td></tr>
  <tr><td>IR <strong>★</strong> / cloud <code>STOP</code></td><td>Emergency stop (works mid-mission)</td></tr>
</table>
<p>When the arm sees a new stream entry ending in <code>_complete</code>, it scans the block and sorts it:
AprilTag ID 1 → RED bin, ID 2 → BLUE bin, ID 3 → YELLOW bin.</p>

<h2>Pin Map (UNO)</h2>
<table>
  <tr><th>Function</th><th>Pin</th><th>Function</th><th>Pin</th></tr>
  <tr><td>IR receiver</td><td>A3</td><td>Ultrasonic TRIG / ECHO</td><td>12 / 13</td></tr>
  <tr><td>Line sensors L / M / R</td><td>A0 / A1 / A2</td><td>Gripper servo</td><td>9</td></tr>
  <tr><td>Colour OUT / S2 / S3</td><td>6 / 7 / 8</td><td>ESP32 link (RX / TX)</td><td>11 / 10</td></tr>
  <tr><td>Motor driver (I²C-style)</td><td>3 / 2</td><td></td><td></td></tr>
</table>

<h2>Project Structure</h2>
<pre><code>src/uno_main.cpp            Mecanum car mission control (env:uno)
extras/esp32_main.cpp       WiFi/Favoriot bridge (env:esp32)
extras/robot-arm/           ESP32 arm + HuskyLens sketches
extras/Laboratory Robot.pdf Banana Robotics lab-automation proposal
lib/MecanumCar_v2/          Mecanum drive library (Keyestudio KS0560)
platformio.ini              Build environments (uno, esp32)
secrets.h                   WiFi &amp; Favoriot credentials (gitignored)</code></pre>

<h2>Build &amp; Setup</h2>
<ol>
  <li>Install <a href="https://platformio.org">PlatformIO</a>.</li>
  <li>Create <code>secrets.h</code> in the project root (it is gitignored — never commit it):
<pre><code>#define WIFI_SSID       "your-primary-ssid"
#define WIFI_PASSWORD   "your-primary-password"
#define WIFI_SSID2      "your-fallback-ssid"
#define WIFI_PASSWORD2  "your-fallback-password"

#define FAVORIOT_API_URL   "https://apiv2.favoriot.com/v2/streams"
#define FAVORIOT_API_KEY   "your-favoriot-api-key-or-jwt"
#define DEVICE_ACCESS_TOKEN "your-device-access-token"   // MQTT RPC only

#define DEVICE_DEVELOPER_ID    "yourdevice@yourusername"
#define MOBILE_ROBOT_DEVICE_ID "yourdevice@yourusername" // arm polls this</code></pre></li>
  <li>Flash the car: <code>pio run -e uno -t upload</code> · Flash the bridge: <code>pio run -e esp32 -t upload</code>.</li>
  <li>Flash the arm sketch (<code>extras/robot-arm/Robot-Arm-Final.cpp</code>) to its own ESP32 with the HUSKYLENS, Adafruit PWM Servo Driver, and ArduinoJson libraries.</li>
  <li>On the HuskyLens: set Protocol Type to <strong>UART</strong> (9600 baud) and select <strong>Tag Recognition</strong>; attach AprilTags (tag36h11) 1/2/3 to the red/blue/yellow objects.</li>
  <li>On Favoriot: create devices for the robot and arm, and set the IDs above to match.</li>
</ol>

<div class="card warn">
  <strong>Security note:</strong> all credentials live only in <code>secrets.h</code>, which is excluded
  from version control. If a key is ever committed, regenerate it in the Favoriot dashboard.
</div>

<h2>Beyond the Prototype — Laboratory Automation Proposal</h2>
<p>
The concept was scaled into an industry proposal (<code>extras/Laboratory Robot.pdf</code>): a
<strong>MiR250 AMR</strong> and <strong>Stäubli TX2-60 (Striclean+)</strong> arm automating hazardous
chemical transport in accredited laboratories, with a <strong>Vision Language Model</strong> generating
inspection reports — GHS label verification, container integrity check, and visible leakage detection —
backed by TAM/SAM/SOM market analysis, a go-to-market strategy, and an MVP control dashboard.
</p>

<h2>Team</h2>
<p>Banana Robotics — Jo Shen · Matt · Sivesh · Daniel · Yihong</p>

<p>
<span class="tag">C/C++</span><span class="tag">Arduino UNO</span><span class="tag">ESP32</span>
<span class="tag">PlatformIO</span><span class="tag">IoT (Favoriot)</span><span class="tag">HuskyLens</span>
<span class="tag">Computer Vision</span><span class="tag">Mecanum Drive</span><span class="tag">Servo Control</span>
</p>

<footer>
Mobile-Robot-Final-Program · Built during a robotics training program, Jun–Jul 2026.
</footer>

</body>
</html>
