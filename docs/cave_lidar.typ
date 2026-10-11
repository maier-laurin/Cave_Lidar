// Cave Lidar - Engineering Documentation
// Build: typst compile cave_lidar.typ     Live: typst watch cave_lidar.typ
// Put images and diagrams in ./figures

#set document(title: "Cave Lidar - Engineering Documentation", author: "Laurin Maier")
#set page(paper: "a4", margin: (x: 20mm, y: 2.5cm), numbering: "1")
#set text(font: "New Computer Modern", size: 11pt, lang: "en")
#set par(justify: true)
#set heading(numbering: "1.1")
#show link: set text(fill: blue)

// Marker for things still to be filled in
#let todo(body) = text(fill: red, weight: "bold")[TODO: #body]

#align(center)[
  #text(size: 22pt, weight: "bold")[Cave Lidar] \
  #v(2pt)
  #text(size: 14pt)[Engineering Documentation] \
  #v(6pt)
  Laurin Maier \
  #datetime.today().display("[day].[month].[year]")
]

#v(1em)
#outline(indent: auto)
#pagebreak()

= Overview

A mobile rotating lidar rig for mapping caves. A spinning lidar is rotated about a second axis by a stepper drive, and the sensor pose (IMU) is fused with the lidar data to reconstruct a point cloud. This document describes the hardware, the control software, and the reconstruction and viewing software.

#todo[Project goals, requirements, intended use, system block diagram.]

= Hardware

== Lidar

Velodyne HDL lidar on a 3D-printed mount, rotated about the horizontal shaft axis.

#todo[Exact lidar model, mass, power supply.]

=== Network

The lidar is connected over Ethernet and streams its data as UDP packets. Its IP address was changed to `192.168.3.201`, so that it sits on its own subnet, separate from the home network (`192.168.1.x`). The PC reaches it through a second, dedicated Ethernet adapter, while the first adapter stays on the home network.

#table(
  columns: (auto, 1fr),
  align: (left, left),
  stroke: 0.5pt + gray,
  [*Item*], [*Value*],
  [Lidar IP], [`192.168.3.201`],
  [Lidar subnet], [`192.168.3.0/24` (mask `255.255.255.0`)],
  [Home network (first adapter)], [`192.168.1.0/24` (mask `255.255.255.0`)],
  [Lidar gateway], [`192.168.3.1` (not used)],
  [Capture PC (second adapter)], [`192.168.3.100/24`, no gateway],
  [Destination (host) address on the lidar], [`255.255.255.255` (broadcast on the lidar subnet)],
  [Data port (UDP)], [3201],
  [Telemetry port (UDP)], [8308],
)
The lidar settings can be read at `http://192.168.3.201/cgi/settings.json` (web interface at `http://192.168.3.201`). On the Windows PC, inbound UDP 3201 must be allowed in the firewall for the capture program, otherwise the packets are dropped silently.

== Rotation mechanism

The lidar is mounted on a shaft carried by two 3D-printed bearing brackets on an aluminium T-slot frame. A stepper motor with planetary gearbox drives the shaft through a toothed belt (GT2-type, marked 250-6). A slip ring at the end of the shaft carries power and data to the rotating lidar without limiting the rotation angle.

#figure(
  image("figures/Drawings-welle.pdf", width: 170mm),
  caption: [Main rotation axis, section view. Dimensions in mm. Drawn at 1:1 (170 mm wide).],
) <fig-axis>

#block(breakable: false, table(
  columns: (auto, 1fr),
  align: (center, left),
  stroke: 0.5pt + gray,
  [*Item*], [*Description*],
  [A], [Bearing at the drive end of the shaft],
  [B], [Toothed belt pulley (driven by the stepper)],
  [C], [Hollow shaft, 25 mm diameter],
  [D], [Index flag for the light barrier],
  [E], [Bearing with sleeve on the lidar side],
  [F], [Lidar mount, angled at $165 degree$ to the shaft axis],
))
#todo[Confirm item descriptions, bearing type and size, shaft material and wall thickness.]

The shaft is carried by two bearings 130 mm apart (@fig-axis). The lidar mount extends 217.1 mm beyond the lidar-side bearing.

#figure(
  rect(width: 100%, height: 5cm, stroke: 0.5pt + gray)[
    #align(center + horizon)[#todo[Insert photo of the assembled rig, e.g. `image("figures/rig_photo.jpg")`]]
  ],
  caption: [Assembled rotation mechanism.],
)

=== Drive and ratio

The total reduction from motor to lidar shaft (gearbox and belt together) was measured with the index sensor:

#table(
  columns: (auto, auto),
  align: (left, right),
  stroke: 0.5pt + gray,
  [*Quantity*], [*Value*],
  [Microstepping], [16 (3200 steps per motor revolution)],
  [Measured steps per lidar revolution], [31999 (mean of 5 revolutions)],
  [Total reduction], [10.00 : 1],
  [Revolution-to-revolution spread], [25 steps $approx 0.28 degree$],
)

#todo[Gearbox ratio, pulley tooth counts, belt length and width, motor type and rated torque.]

=== Belt tensioning

The motor sits on a slotted 3D-printed mount that slides on the extrusion, so the belt centre distance can be adjusted and locked.

#todo[Measured belt tension / pluck frequency, final motor mount design.]

== Index sensor

A light barrier detects the passage of a flag on the lidar shaft once per revolution. It sits after the belt on the lidar shaft, so skipped belt teeth appear as a changed step count per revolution. The sensor edge occurs when the lidar is $90.3 degree$ (8027 steps) before vertical in the scan direction. (The value was 93° until 2026-10-10. A scan of a flat floor showed that the zero position was 2.7° off, a roll about the shaft axis, so the offset was corrected.)

== Electronics

#todo[Teensy 4.1, MKS SERVO42C driver, BNO085 IMU, power supply, battery.]

== Rig wiring

#todo[Wiring diagram.]

The Teensy (3.3 V logic, not 5 V tolerant) is powered over USB only. Never connect it to the 12 V supply and never apply 5 V to any of its pins. Teensy GND and the 12 V supply GND must be connected.

=== Stepper driver (MKS SERVO42C)

Closed-loop driver on the back of a NEMA 17 stepper (ZT-42HS3922-1503, 1.8°, about 1.5 A). The motor coils (A+, A−, B+, B−) connect to the driver through its 4-pin connector.

#table(
  columns: (auto, auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*Driver pin*], [*Label*], [*Connect to*], [*Notes*],
  [V+], [VIN], [12 V supply +], [12–24 V allowed],
  [Gnd], [GND], [12 V supply − and Teensy GND], [Common ground required],
  [Com], [VCC], [not connected], [Reads about 3 V floating. If steps are missed, connect to Teensy 3.3 V],
  [En], [EN], [Teensy pin 4], [Ignored while menu `En = Hold`],
  [Stp], [CLK], [Teensy pin 2], [Step pulses],
  [Dir], [DIR], [Teensy pin 3], [HIGH/LOW selects direction],
)

=== IMU (BNO085, I2C)

Adafruit BNO085, mounted rigidly on the frame next to the motor mount, so it measures the frame pose. I2C address `0x4A` (`0x4B` if the address jumper is bridged). BT, P0, P1, DI, CS and 3Vo stay unconnected.

#table(
  columns: (auto, auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*BNO085 pin*], [*STEMMA QT wire*], [*Teensy*], [*Notes*],
  [VIN], [red], [3.3 V], [Board has its own regulator],
  [GND], [black], [GND], [],
  [SDA], [blue], [pin 18 (SDA0)], [`Wire`, 400 kHz],
  [SCL], [yellow], [pin 19 (SCL0)], [],
  [RST], [grey], [pin 17], [Hardware reset, used by the library],
  [INT], [purple], [pin 16], [Wired but unused, never driven],
)

*Known issue:* after uploading a new sketch the BNO085 sends no data (even though `begin_I2C` succeeds) until the USB cable is unplugged and replugged. A cold start works. The reset pin is verified working, so the cause is the BNO state after an upload. Workaround: replug USB after each upload.

=== Light barrier (index sensor)

One pulse per lidar-shaft revolution, used for homing and the index edge.

#table(
  columns: (auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*Wire*], [*Teensy*], [*Notes*],
  [red (power)], [3.3 V], [Not 5 V: the Teensy input is not 5 V tolerant],
  [black (GND)], [GND], [],
  [green (signal)], [pin 5], [`INPUT_PULLUP`, interrupt on the falling edge (output goes LOW at the index)],
)

=== Velodyne VLP-16 GPS simulation

The Teensy imitates a GPS receiver so the VLP-16 timestamps share one clock with the IMU. Simulated time starts at 2000-01-01 00:00:00 UTC at Teensy boot.

#table(
  columns: (auto, auto, auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*Signal*], [*Jumper*], [*Velodyne GPS pin*], [*Teensy 4.1*], [*Notes*],
  [PPS], [green], [1], [pin 10], [3.3 V, 10 ms pulse every second, rising edge = top of the second],
  [GND], [white], [3], [GND], [],
  [NMEA], [yellow], [4], [pin 1 (TX1, `Serial1`)], [`$GPRMC` at 9600 baud 8N1, sent 100 ms after each PPS edge],
)

`Serial1` is a hardware UART separate from the USB port, so programming and the PC data stream are not affected.

= Control software

== Overview

The rig is run by a single firmware, `firmware/lidar_ctrl/lidar_ctrl.ino`, on the Teensy 4.1. It has five jobs:

+ Rotate the lidar shaft at a constant, selectable speed (@sec:steps).
+ Know the shaft angle at all times, using a step counter that is re-zeroed by the index sensor once per revolution (@sec:angle).
+ Imitate a GPS receiver (PPS pulse and NMEA sentence) so that the VLP-16 time-stamps its packets on a clock that the rig controls (@sec:time).
+ Read the BNO085 IMU at 100 Hz (@sec:imu).
+ Send the IMU data, the shaft angle and the index events to the PC as time-stamped binary packets over USB (@sec:usb).

The firmware does not log anything itself. The PC records the USB stream, and separately the UDP packets of the VLP-16. Both carry time stamps from the same simulated clock, which is what allows the two recordings to be merged afterwards.

=== Building and uploading

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt + gray,
  [*Item*], [*Setting*],
  [Environment], [Arduino IDE with Teensyduino],
  [Board], [Teensy 4.1],
  [Library], ["Adafruit BNO08x" (Library Manager, accept the dependencies)],
  [USB type], [Serial. The baud rate setting has no effect on the USB connection; the sketch opens it at 115200],
)

After every upload the BNO085 may stay silent until the USB cable is unplugged and replugged (see the known issue in the wiring section). The firmware reports this on the USB stream as a text message.
With the Arduino command line tool: `arduino-cli compile --fqbn teensy:avr:teensy41 firmware/lidar_ctrl` and `arduino-cli upload -p COM4 --fqbn teensy:avr:teensy41 firmware/lidar_ctrl`. The serial port must not be open in another program (serial monitor, CaveScan) during the upload. The sketch `firmware/bno_calibration` is a helper for calibrating the IMU (see the reconstruction chapter), not a scan firmware.

=== Design rules

The code follows a few rules that explain most of its structure.

- *One clock.* Every time stamp, on the USB stream as well as in the PPS and NMEA signals sent to the lidar, is derived from the same simulated UTC time (@sec:time).
- *Nothing blocks.* The main loop runs freely and calls one service function per task. Writing to USB never waits: if the PC does not read fast enough, the packet is dropped and counted.
- *Interrupts only do the time-critical part.* Step pulses, the index sensor and the PPS edge are handled in interrupts. Everything else (messages, state changes, NMEA, USB output) happens in the main loop.
- *Interrupt priority.* The PPS timer has priority 8, the step timer 16 (lower number means higher priority on the Teensy 4.x). A step pulse can therefore never delay a PPS edge, and the I2C traffic for the IMU never delays a step.

== Operation

=== Normal sequence

+ *Power up.* The firmware starts in the homing mode: the shaft turns at 3000 steps/s until the index sensor is hit, then keeps turning to the vertical position and stops. The message `At 0 deg, stopped.` appears and the rig holds. The IMU is started about one second after power-up.
+ *Configuration.* The PC sends one line `C,<rpm>,<unix_seconds>`. The first PPS pulse follows one second later and carries the time label `unix_seconds`. From the first pulse on, the binary data stream starts.
+ *Start of rotation.* Ten seconds after the configuration line (and only if the shaft has been homed) the shaft ramps up to the requested speed and keeps turning. The delay makes sure the PPS signal has been running for a while before the shaft moves.
+ *Stop.* The PC sends the single character `x`. The shaft ramps down to the low homing speed, runs on to the next index edge and then to the vertical position, where it stops. PPS, NMEA and the data stream keep running. A new `C` line can be sent once the shaft has stopped.

=== Commands

Commands are sent as text over the USB serial connection. The firmware answers with text messages inside the binary stream (type `M`, @sec:usb).

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt + gray,
  [*Command*], [*Function*],
  [`C,<rpm>,<unix_seconds>` + newline], [Configure and start. `rpm` is 1 to 20 (decimals allowed). `unix_seconds` is the UTC time, in seconds since 1970, that the first PPS pulse will carry. Restarts PPS and the data stream from zero.],
  [`x` (alone, or `x` + newline)], [Stop. Ramps down and stops at 0°. Also cancels a start that has been configured but has not yet begun to turn.],
)

`x` is only recognised as the first character of a line, so it cannot be confused with the text of a `C` command. Any other text is answered with `Unknown command`.

Replies worth knowing:

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt + gray,
  [*Message*], [*Meaning*],
  [`Homing: searching edge...`], [Start-up homing has begun.],
  [`Edge found, moving to 0 deg...`], [Index edge seen during homing, now driving to vertical.],
  [`At 0 deg, stopped.`], [The shaft is stopped at vertical and holding.],
  [`Config ok: ...`], [A `C` line was accepted. Repeats rpm, the label of the first pulse and the start delay.],
  [`Running at x.x rpm`], [Rotation has started.],
  [`Stopping: decelerating, will stop at 0 deg`], [`x` received while running.],
  [`C rejected: rotation active, send x first`], [A `C` line arrived while the shaft was turning or stopping.],
  [`C rejected: rpm must be 1..20`], [Speed outside the allowed range.],
  [`C rejected: format is C,<rpm>,<unix_seconds>`], [The line could not be parsed.],
  [`Start cancelled`], [`x` received while holding, before the start delay had elapsed.],
  [`Busy homing/stopping, wait`], [`x` received during homing or while already stopping.],
  [`BNO085 ok` / `BNO085 not found`], [Result of an IMU start attempt. The firmware retries every second.],
  [`BNO085 sends no data: ...`], [The IMU has been silent for 3 s. Replug USB (known issue).],
)

To start a second measurement after `x`, send a new `C` line. This resets the PPS counter, so the stream restarts with a new header and a new time label.

=== Operating modes

The motor controller is a small state machine with five modes.

#table(
  columns: (auto, 1fr, 1fr),
  stroke: 0.5pt + gray,
  [*Mode*], [*Meaning*], [*Left when*],
  [`HOMING`], [Turning at 3000 steps/s, waiting for the first index edge. Entered at power-up.], [Index edge seen: goes to `MOVING_ZERO`.],
  [`MOVING_ZERO`], [Turning slowly to the vertical position, 8266 steps after the index edge.], [Step counter reaches the target: goes to `HOLD`.],
  [`HOLD`], [Motor not stepping. Waits for a configuration.], [Start delay elapsed and shaft homed: goes to `RUNNING`.],
  [`RUNNING`], [Turning at the configured speed.], [`x` received: goes to `STOPPING`.],
  [`STOPPING`], [Slowed to 3000 steps/s or less, waiting for the next index edge.], [Index edge seen: goes to `MOVING_ZERO`.],
)

== Parameters

The constants at the top of the sketch are the only values that need to be changed for a different rig. The measured ones come from the calibration described in the hardware chapter.

#table(
  columns: (auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*Constant*], [*Value*], [*Meaning*],
  [`STEPS_PER_REV`], [31999], [Steps per lidar revolution (measured, total ratio 10 : 1 with 3200 steps per motor revolution).],
  [`VERTICAL_AFTER_EDGE_DEG`], [90.3°], [Angle between the index edge and the vertical lidar position (corrected from 93.0° on 2026-10-10 using the floor of a scan).],
  [`HOME_STEPS_PER_S`], [3000], [Search speed during homing and the speed used for approaching and stopping at 0°.],
  [`RAMP_STEPS_PER_S2`], [8000], [Acceleration and deceleration of the step rate.],
  [`MIN_START_STEPS_PER_S`], [400], [Lowest step rate used when starting from rest.],
  [`MIN_RPM`, `MAX_RPM`], [1, 20], [Accepted range of the `C` command.],
  [`DEBOUNCE_US`], [50 000 µs], [Index edges closer together than this are ignored.],
  [`START_DELAY_MS`], [10 000 ms], [Delay between the `C` line and the start of rotation.],
  [`PPS_WIDTH_MS`], [10 ms], [Length of the PPS pulse.],
  [`NMEA_DELAY_MS`], [100 ms], [Delay between the PPS edge and the NMEA sentence.],
  [`REPORT_US`], [10 000 µs], [IMU report interval (100 Hz).],
  [`IMU_SILENT_MS`], [3000 ms], [Time without IMU data before the warning is sent.],
)

== Step generation and shaft speed <sec:steps>

The stepper driver gets one pulse per step on pin 2. The pulses are produced by a hardware timer interrupt (`IntervalTimer`), not by the main loop, so the step rate does not depend on what else the firmware is doing. Each pulse is 3 µs long and increments the step counter. The direction pin (3) is set once at start-up and never changed, so the shaft only ever turns in one direction. The enable pin (4) is set low at start-up and stays there.

The step rate follows the target rate with a constant acceleration. Every millisecond the main loop moves the current rate towards the target by `RAMP_STEPS_PER_S2` $times$ 1 ms, and then gives the new interval to the timer. When starting from rest the rate jumps straight to 400 steps/s, because the timer only takes over a new period after the current one has ended and a very long first period would make the start sluggish.

The target rate follows from the requested speed:

$ f_"step" = frac("rpm", 60) dot 31999 "steps/s" $

#table(
  columns: (auto, auto, auto),
  align: (right, right, right),
  stroke: 0.5pt + gray,
  [*Speed*], [*Step rate*], [*Time to reach it from rest*],
  [1 rpm], [533 steps/s], [under 0.1 s],
  [6 rpm], [3200 steps/s], [about 0.4 s],
  [10 rpm], [5333 steps/s], [about 0.7 s],
  [20 rpm], [10 667 steps/s], [about 1.3 s],
)

The speed is open loop: the driver is told how many steps to make, and there is no feedback from its encoder. A lost step is therefore not noticed directly. It is noticed once per revolution, by the index sensor (next section).

== Shaft angle and index sensor <sec:angle>

The step counter `axisSteps` counts the steps since the last index edge. The light barrier on pin 5 triggers an interrupt on the falling edge, and in that interrupt the counter is read and set back to zero. The two operations are done with interrupts disabled so that no step can be lost in between. The re-zeroing every turn means that an error from slipping belt teeth or lost motor steps can never add up over more than one revolution.

The edge occurs when the lidar is 90.3° before the vertical position. The angle of the lidar for a given counter value is therefore

$ theta = (frac(n dot 360 degree, 31999) - 90.3 degree) mod 360 degree $

and is taken as a positive value. $theta = 0 degree$ is vertical. This is the same formula that is given to the receiving software in the data stream.

Edges closer together than 50 ms are ignored (debounce). At the highest allowed speed the true edges are 3 s apart, so this never removes a real edge.

At every edge the firmware sends an `E` packet containing the counter value just before the re-zero. Without slip this is close to 31999. The first edge after a start carries no information, because the counter was not started on an edge. A value that differs from 31999 by more than the normal spread of about 25 steps indicates belt slip or lost steps.

The stop at vertical works in the same way: after an edge the counter is zeroed, the target is 8266 steps, and the step interrupt itself stops the motor when the counter reaches it. This avoids overshoot caused by the main-loop delay.

== Time base, PPS and NMEA <sec:time>

The VLP-16 expects a GPS receiver. It needs a once-per-second pulse (PPS) whose rising edge marks the start of a second, and a text sentence (`$GPRMC`) saying which second that was. The firmware produces both, so that the lidar time-stamps its packets with a clock that the rig also uses for the IMU.

When the `C` line arrives, a one-second interval timer is started. Each timer expiry:

- sets the PPS pin high (in the interrupt), stores the value of the microsecond counter `micros()` at that instant and counts the pulse;
- 10 ms later (main loop) sets the pin low;
- 100 ms after the edge sends the NMEA sentence for that second on `Serial1` at 9600 baud.

The first pulse comes one second after the `C` line and carries the label `unix_seconds`. Pulse $k$ (counting from 1) carries `unix_seconds` + $k - 1$. The NMEA sentence is sent after the pulse and names the pulse that just happened, as a real receiver does. Position fields are zero, the status is "valid".

Any local time can then be converted to the simulated UTC time. For a microsecond value $t$ taken at any moment after the first pulse:

$ T_"unix" = (L + k - 1) dot 10^6 + (t - t_"pps") $

with $L$ the label of the first pulse, $k$ the number of the latest pulse and $t_"pps"$ the microsecond counter at that pulse. Only the 32-bit difference $t - t_"pps"$ is used, so the 71-minute wrap-around of the counter does not matter.

The clock is not synchronised to real time after the first pulse. It runs on the Teensy crystal. The label only has to be right once, at the start, so the PC should send its current UTC time plus one second, because the first pulse comes one second after the line arrives.

== IMU <sec:imu>

The BNO085 is read over I2C at 400 kHz. Four reports are enabled, each at 100 Hz:

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt + gray,
  [*Report*], [*Content*],
  [Rotation vector], [Orientation quaternion, referenced to magnetic north (the magnetometer is used), with an accuracy estimate.],
  [Linear acceleration], [Acceleration in the sensor frame with gravity removed, in m/s#super[2].],
  [Calibrated magnetic field], [Magnetic field in the sensor frame, in µT, with a calibration status.],
  [Calibrated gyroscope], [Angular velocity in the sensor frame, in rad/s, with a calibration status.],
)

The tilt of the rig (rotation about the horizontal axes) is known accurately from gravity: the direction of "up" in the sensor frame is `R(q)^T z` of the rotation vector, and it scattered by only 0.02° in a stationary recording. The rotation about the vertical axis is only known from the magnetometer, which is badly disturbed near the motor and the lidar. The reconstruction therefore integrates the gyroscope (vertical rate = angular velocity projected on "up") for the relative heading and uses the magnetic heading only as a weak prior.

*Known issue:* when the game rotation vector is enabled in addition, the normal rotation vector stops arriving (0 reports in 5 s, while all other reports keep their 100 Hz). The firmware therefore does not enable it (tested 2026-10-10).

While streaming, the firmware sends a text message every 5 s with the number of reports received per type (about 500 each expected), which shows a stalled report.

One data packet is sent per rotation-vector report. The most recent linear acceleration, magnetic field, angular velocity and game rotation vector values are attached to it. They are not interpolated, so they can be up to one report interval (10 ms) older than the quaternion.

The time stamp of a packet is taken in the main loop when the report has been read from the sensor, not at the moment the sensor measured. It therefore includes the delay of the sensor's internal processing and of the I2C transfer. This delay has not been measured yet.

If the IMU is not found at start-up, the firmware tries again every second. If the sensor resets on its own, the reports are enabled again. If no data arrives for 3 s a warning message is sent once.

#todo[Measure the latency between the IMU measurement and the time stamp, and how large it is compared with the time stamp resolution needed for reconstruction.]

== USB data stream <sec:usb>

=== Packet format

All data from the Teensy to the PC are binary packets. Multi-byte values are little endian.

#table(
  columns: (auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*Bytes*], [*Field*], [*Meaning*],
  [1], [`0xA5`], [Start marker.],
  [1], [type], [ASCII character `H`, `S`, `E` or `M`.],
  [1], [len], [Length of the payload in bytes (0 to 255).],
  [len], [payload], [Depends on the type.],
  [2], [crc], [CRC-16/CCITT-FALSE over type, len and payload (polynomial `0x1021`, start value `0xFFFF`, no reflection, no final XOR). Low byte first.],
)

The marker byte can also occur inside the payload. A receiver must therefore check the CRC and, if it fails, search for the next `0xA5` one byte after the previous one. Packets that fail the CRC are discarded.

The Teensy never waits for the PC. If the USB buffer is full, the packet is dropped and a counter is incremented. The counter is sent with every `S` packet (field `drops`, wraps after 255). A jump in this counter tells the PC that data was lost.

=== `H` header packet (17 bytes)

Sent at the start of the stream (with the first NMEA sentence, 100 ms after the first pulse, so up to ten `S` packets can come before it) and then every 10 s, so a recording can be started at any time and still be decoded.

#table(
  columns: (auto, auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*Offset*], [*Type*], [*Field*], [*Meaning*],
  [0], [u8], [version], [Protocol version, currently 2. Version 1 had no gyroscope; the decoder still reads it.],
  [1], [f32], [steps_per_rev], [Steps per lidar revolution (31999).],
  [5], [f32], [vertical_after_edge_deg], [Angle from index edge to vertical (90.3; 93.0 in recordings made before 2026-10-10).],
  [9], [f32], [rpm], [Configured speed.],
  [13], [u32], [first_pulse_unix_s], [UTC label of the first PPS pulse.],
)

=== `S` sample packet (70 bytes, 100 per second)

#table(
  columns: (auto, auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*Offset*], [*Type*], [*Field*], [*Meaning*],
  [0], [u64], [t_us], [Time in microseconds since 1970 (simulated UTC, @sec:time).],
  [8], [i32], [axis_steps], [Step counter since the last index edge.],
  [12], [4 $times$ f32], [qw, qx, qy, qz], [Orientation quaternion of the IMU (real part first).],
  [28], [u16], [q_acc], [Estimated orientation accuracy in units of 0.01°.],
  [30], [u8], [cal], [Bits 0 and 1: calibration status of the rotation vector (0 to 3). Bits 2 and 3: calibration status of the magnetometer (0 to 3).],
  [31], [3 $times$ f32], [lax, lay, laz], [Linear acceleration in m/s#super[2], gravity removed.],
  [43], [3 $times$ f32], [mx, my, mz], [Magnetic field in µT.],
  [55], [u8], [flags], [Bit 0: the shaft has seen an index edge, so `axis_steps` is valid. Bit 1: the shaft is in the running mode.],
  [56], [u8], [drops], [Number of packets dropped because USB was full, wrapping.],
  [57], [3 $times$ f32], [gx, gy, gz], [Angular velocity in rad/s, sensor frame (calibrated gyroscope). Version 2.],
  [69], [u8], [cal2], [Bits 0 and 1: calibration status of the gyroscope (0 to 3), other bits 0. Version 2.],
)

The first 57 bytes are the same in protocol version 1.

The lidar angle follows from `axis_steps` with the formula in the section on shaft angle and index sensor. A value that is not a number (NaN) in the acceleration or magnetic field means that the IMU has not yet delivered that report.

=== `E` index edge packet (12 bytes)

Sent once per index edge, only while the stream is running.

#table(
  columns: (auto, auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*Offset*], [*Type*], [*Field*], [*Meaning*],
  [0], [u64], [t_us], [Time of the edge in microseconds since 1970. Taken inside the interrupt, so it is accurate.],
  [8], [i32], [steps_before_reset], [Step counter just before it was zeroed. Close to 31999 without slip; meaningless for the first edge.],
)

=== `M` message packet

The payload is plain text, at most 119 characters, without terminator. These are the messages listed in the section on commands.

== Limitations and known issues

- *Open-loop drive.* Lost steps are detected only at the next index edge. Between two edges the angle is only as good as the step count, and the error is removed at the next edge. Interpolating the angle linearly between edge times is a possible refinement for the reconstruction.
- *IMU after upload.* The BNO085 stays silent after programming until USB is replugged (see the wiring section).
- *Time label.* The simulated clock is set once and runs on the Teensy crystal. It does not follow real time, so absolute times drift by the crystal error over a long measurement. Relative times between the lidar and the IMU are exact, because both use this clock.
- *Reconfiguration.* A new `C` line is refused while the shaft turns. To change the speed, stop first with `x`.
- *No safety supervision.* There is no end-stop, current monitoring or watchdog. If the PC program stops, the shaft keeps turning at the last speed.
- *Stream before homing.* A start is delayed until the first index edge has been seen, but the data stream and PPS start immediately. The flag bit 0 of `flags` shows whether `axis_steps` can be used.

= Capture software

The capture software is written in C\# (.NET 10) in `capture/`. It consists of a lidar module (`LidarRecorder`, library `LidarCapture`), a rig module (`TeensyLink`, `TeensyRecording`, library `TeensyCapture`) for the Teensy stream, a small console program for testing the lidar module on its own, and the orchestrator `CaveScan` that runs both. The orchestrator asks for a project name and creates `<root>/<project name>/scan_NN/`. Each new start of the recording goes into a new `scan_NN` folder, so that a pause (e.g. to cross an obstacle) separates the data.

== Lidar recorder

The recorder listens on UDP port 3201 for the data packets (1206 bytes) of the VLP-16 and accepts only packets from the lidar's IP. A listening thread only receives and queues packets (16 MB socket buffer, queue of 20000 packets). A second thread parses them and writes the files, so that a slow disk does not lose packets. Only the strongest-return mode of the VLP-16 is supported; other packets are counted and discarded.

=== File format

The data are written as Apache Arrow IPC stream files `lidar_0001.arrow`, `lidar_0002.arrow`, ... with a new file every 60 s. Each batch of about 0.5 s is flushed to disk, so a crash loses at most that batch. Points are stored in the sensor frame in spherical coordinates, 14 bytes per point, about 4 MB/s at 300 000 points per second. Points without a return (distance 0) are not stored.

#table(
  columns: (auto, auto, 1fr),
  align: (left, left, left),
  stroke: 0.5pt + gray,
  [*Column*], [*Type*], [*Meaning*],
  [`time`], [timestamp, ns, UTC], [Absolute time of the firing, nanoseconds since 1970, interpolated per point.],
  [`azimuth`], [uint16], [Head angle in $0.01 degree$, interpolated per point (0 to 35999).],
  [`elevation`], [int8], [Laser angle in degrees ($-15$ to $15$), replaces the ring number.],
  [`distance`], [uint16], [Distance in units of 2 mm.],
  [`reflectivity`], [uint8], [Raw reflectivity, 0 to 255.],
)

Every file carries its description as schema metadata: format version, sensor, return mode, lidar IP and port, the time label and the time source (below), the units of all columns, software version, creation time of the file, the chunk number and extra entries given by the orchestrator (project, rpm, firmware version).

=== Time

The lidar reports its packet time in microseconds since the top of the hour. Its sub-second phase is locked to the PPS pulses of the rig, so a second always starts exactly at a pulse. The whole seconds are not those of the rig: the lidar does not take the hour, minute and second from the NMEA sentence (its `status.json` shows an empty `position`), and instead counts from its own start-up. The recorder therefore combines two sources:

- the *sub-second part* from the lidar, which is exact and on the same clock as the IMU and step counter of the Teensy;
- the *whole seconds* from a rough reference clock, which only has to be correct to within $plus.minus 0.5 "s"$.

The rough reference is supplied by the caller. In the orchestrator it is the time of the most recent Teensy sample plus the time since that sample arrived, which is the same clock the lidar is locked to, so it never drifts and is off only by the USB latency (a few ms). The stand-alone console program has no Teensy stream and uses the time label plus a stopwatch that was started when the `C` command was sent (the first pulse follows one second later), or the PC clock. None of these depends on the PC clock being right. The recorder compares the reference with the lidar time for the first 50 packets, takes the median and rounds the difference to whole seconds. This integer offset stays fixed for the whole scan and is written to the file metadata (`time_offset_s`), together with the deviation of the reference from the lidar phase (`time_reference_error_ms`). On the real rig this deviation was 1 to 3 ms, so the margin to the 500 ms limit is large.

The offset is *not* recalibrated during a scan, because lidar and rig share one PPS and cannot drift apart. Instead the recorder keeps checking: about every 0.7 s it compares the reference with the lidar time again and shows the result live (`time check`). A warning is logged if it exceeds 300 ms. A stopwatch on the PC would be a poor reference for this check, because its crystal drifts by tens of ppm against the Teensy (0.1 to 0.2 s per hour), which is why the orchestrator uses the Teensy clock.

The recorder continues the time across the change of the hour, and adds the offset of every block, firing sequence and laser inside the packet (110.592 µs per block, 55.296 µs per sequence, 2.304 µs per laser). The azimuth of the second sequence of a block is interpolated from the next block. The lidar counts its microseconds from its own start-up, so it can be anywhere in the hour; the wrap detection must work for all values (a first version overflowed 32 bits above 2.49 billion µs, found by the live time check and covered by a test now).

The lidar needs about 3 s after the start of the PPS pulses to lock onto them, and the time jumps by a fraction of a second at that moment. The recording therefore has to start after the lock; the console program waits 8 s after the `C` command, which is before the shaft starts to turn (10 s). If the lidar clock jumps by more than 100 ms during a recording, this is logged.

#todo[Make the lidar accept the NMEA sentence (position field stays empty, cause not yet found); then the whole seconds would come from the lidar itself.]

=== Monitoring

The recorder counts the packets received, those dropped because the queue was full, those rejected (wrong mode), and the packets missing according to small jumps (up to 100 ms) in the lidar timestamps. These numbers are printed every 2 s by the console program.

=== Test on the real rig

A 30 s recording at 6 rpm (lidar at 600 rpm) gave 5.6 million points in a single 75 MB file, which is 2.5 MB/s or 13.4 bytes per point. No packet was lost, the queue never held more than one packet, and the times were continuous and correct to the PC clock. The points show a regular gap of 20 to 40 ms every 100 ms, which is the sector of each lidar revolution in which the beam hits the rig frame.

The console program has two helper commands: `inspect <folder>` prints a summary of the files of a scan (rows, time range, gaps, distance range, azimuth coverage, metadata), and `sniff <port>` shows the packets on the lidar's telemetry port.

== Rig recorder

`TeensyLink` keeps the serial port open and decodes the binary stream (packet format in @sec:usb). Packets with a wrong CRC are dropped, and impossible type and length combinations are rejected at once, so a stray marker byte does not stall the decoder. The firmware only streams after a `C` command; between scans only text messages arrive. While a scan is running, a `TeensyRecording` saves the stream into the scan folder:

- `rig_0001.arrow`, ... : one row per sample (100 per second): `time` (absolute, ns), `axis_steps`, quaternion, `q_acc`, `cal`, linear acceleration, magnetic field, `flags`, `drops`, and since protocol version 2 the angular velocity `gx`, `gy`, `gz` and its status `cal2` (older recordings do not have these columns; the schema metadata says `cave_lidar.rig/2` for the new format).
- `edges_0001.arrow`, ... : one row per index edge: `time`, `steps_before_reset`.
- `rig_messages.log`: all text messages and headers with the PC time of arrival.

The files are Arrow IPC streams like the lidar files, with a new file every 60 s and the description in the schema metadata (column meanings, units, steps per revolution, rpm, label of the first pulse). The times are on the simulated UTC of the rig, which is also the time scale of the lidar data, so the two can be joined directly. A queue decouples the serial reader from the disk. About 0.36 MB are written per minute.

== Orchestrator `CaveScan`

The orchestrator is a command line program with a live display:

```
CaveScan --root <folder> [--project <name>] [--teensy COM4] [--rpm 6] [--lidar-ip 192.168.3.201]
         [--port 3201] [--chunk-seconds 60] [--lock-wait 8] [--auto <seconds>]
```

It asks for a project name (if not given with `--project`) and uses it exactly as typed as folder name `<root>/<project name>/`. Names that are not legal folder names (empty, leading or trailing space, trailing dot, `\ / : * ? " < > |`, reserved Windows names) are refused. A project may already exist; each scan gets the next free folder `scan_01`, `scan_02`, ... so the parts of one project stay separate, for instance before and after an obstacle.

Keys: `s` starts a scan, `x` stops it (a second `x` ends the recording at once), `q` quits when no scan is running. `Ctrl+C` closes the scan and stops the rig. Because the firmware keeps the shaft turning if the PC disappears, the program always sends the stop command when it ends.

*Sequence of a scan:*

+ A new folder `scan_NN` is created, and the rig recording starts.
+ `C,<rpm>,<label>` is sent to the rig (label = PC time plus one second) and the program waits for the answer `Config ok`. A rig that does not answer, or that is still turning, aborts the start.
+ The program waits at least 8 s (lock wait) and until the lidar's status page (`/cgi/status.json`) reports `Locked` for the PPS. The lidar needs about 3 s to lock and its time jumps then. The shaft starts to turn after 10 s.
+ The lidar recording starts, with the Teensy clock as time reference.
+ On stop the rig runs down to 0° (about 10 to 20 s) while both recordings continue, then the lidar and rig recordings are closed and `scan.json` is written.

*Live display:* state and elapsed time; for the rig the sample rate, USB drops, CRC errors, index edges, shaft state and angle, IMU quaternion with heading accuracy, calibration status of the magnetometer, rotation vector and gyroscope, field strength and angular rate, and the last message; for the lidar the status page (PPS, motor, laser), packets and points per second, lost and dropped packets, clock jumps, the time check against the rig clock and the current file; free disk space with the remaining recording time at the current data rate; and the last messages.

*Files of a scan folder:* `lidar_NNNN.arrow`, `rig_NNNN.arrow`, `edges_NNNN.arrow`, `rig_messages.log`, `orchestrator.log` (all events) and `scan.json` (project, scan, start and end time, rpm, label, software version, and the final counters of lidar and rig).

With `--auto <seconds>` a scan runs unattended for that time and the program then exits, without display and keys.

=== Test of a complete scan

An unattended scan of 30 s plus the run-down (total 51 s, `--auto 30`) recorded 8.1 million lidar points (108 MB) and 5000 rig samples. No lidar packet was lost, no packet was dropped by the Teensy, there were no CRC errors, and the time check stayed between 1.6 and 3 ms.

#todo[Long recording and disk write times on the Raspberry Pi with the USB SSD; pause and resume across several scans.]

= Reconstruction and viewing software

The reconstruction software is the Julia package `CaveLidarRecon` in the folder `reconstruction/`. It works offline on the Arrow files of a scan folder.

== Untangling the rotation (stage 1)

`julia --project=reconstruction reconstruction/scripts/untangle.jl <scan folder> [--world | --no-attitude]` (a project folder with several `scan_NN` folders works too) reads all `lidar_*`, `rig_*` and `edges_*` files of a scan and writes one combined `untangled.arrow` into the scan folder. Columns: `time` (ns since 1970, UTC, as in the recordings), `x`, `y`, `z` (m, float32) and `reflectivity`. The lidar chunks are processed one after the other. The schema metadata records the parameters, the frame and the software version.

For every point:
+ The shaft angle at the firing time is interpolated on the circle (shortest arc) from the rig samples: $theta = 2 pi n / N - "offset"$, with $N$ the step count of the revolution the sample belongs to. The index edges are exact nodes of the track. The offset from the index edge to vertical is taken from the rig file metadata (`vertical_after_edge_deg`, 90.3° since 2026-10-10); recordings without it were made with 93.0° and are processed with that value. Points closer than 0.5 m (the rig itself) and points inside a rig dropout longer than 50 ms are dropped.
+ The point is rotated by the mount (tilt $tau$ = 15.96°, yaw $psi$ = 89.855°), shifted by the optical centre (y = −1.11 mm, z = −0.48 mm) and rotated about the shaft axis X by $sigma theta$ with $sigma = -1$. The mount values were fitted with the plane thickness of a stationary scan (7.3 mm → 5.9 mm RMS).
+ The attitude of the BNO085 (rotation vector) is interpolated (slerp) between the two samples around the firing time. The BNO sits on the frame, not on the shaft, so it measures the attitude of the whole rig. Only rotations are applied, the origin stays at the rig. The BNO axes relative to the rig frame are taken from the drawing: z is the rig Z axis, y is the rig X axis (so x is the rig −Y axis). `bno_latency_ns` (default 0) shifts the attitude in time if the IMU stamp turns out to be late.

The third step has three modes:

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt + gray,
  [*Mode*], [*Result*],
  [default (gravity)], [The direction of "up" in the rig frame is taken from the quaternion, and the point is rotated by the smallest rotation that takes it onto +Z. Only the tilt is corrected; the heading about the vertical stays the rig's own (no magnetometer). Afterwards only a rotation about z is missing to align the cloud with the real world. If the rig turns about the vertical during a scan, that yaw is time dependent and is still in the cloud (the gyroscope is meant to supply it, see below).],
  [`--world`], [The full BNO attitude rotates the point into East-North-Up. The heading comes from the magnetometer, which is unreliable on this rig (see below).],
  [`--no-attitude`], [The points stay in the rig frame: X along the shaft, Z vertical at shaft angle 0.],
)

Verification: with the attitude switched off (and offset 93°) the output equals the earlier experiment script `deskew_03.jl` point for point (8 071 530 points, largest difference 0). In gravity mode the z coordinate of every point equals its z in world mode (largest difference 0 m on a scan of 7.5 million points), and the heading difference between the two modes is a constant rotation about z. `reconstruction/test/runtests.jl` contains a synthetic scan with a rotating frame and a known result, tests of the quaternion and rotation code, the mount calibration and the offset lookup.

== Checking the level with the floor

The tilt of the whole cloud can be checked on the floor. The check used so far (a script outside the package, to be added): take every 7th point of the cloud, find the most populated 2 cm height slab below the rig (more than 0.3 m away), fit a plane to the points within 15 cm of it (normal = direction of smallest spread), then repeat ten times with all points within 2 cm of the current plane. The floor tilt is the angle between the plane normal and the z axis. On the scans of 2026-10-10 the plane had 55 000 to 70 000 inliers out to 6.9 m, 6 mm residual, and lay 64.5 to 64.9 cm below the scanner origin (as measured), so it is the floor. Near and far points gave the same tilt.

#table(
  columns: (auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*Scan, setting*], [*Floor tilt*], [*Remark*],
  [`scan_02`, 93° offset, BNO mount flat], [3.45°], [normal (−0.022, −0.056)],
  [`scan_02`, 90.3°], [1.37°], [normal (−0.022, −0.009)],
  [`scan_02`, 89.8°], [1.28°], [roll about the shaft removed completely],
  [`scan_02`, 95.7° (wrong direction)], [6.0°], [],
  [`offset_90_3/scan_01`, firmware 90.3°], [1.25°], [normal (−0.020, −0.010); 89.8° gives 1.13°],
)

The floor of the first scans was tilted by about 2.7° about the shaft axis. The cause was the offset from the index edge to vertical: the 93° was 2.7° too large (measured on the rig, and confirmed by the floor). The firmware constant `VERTICAL_AFTER_EDGE_DEG` is now 90.3° (the data suggest 89.8°, not yet changed). After the correction a tilt of about 1.1 to 1.3° remains in the x component of the normal (rotation about the Y axis). It does not depend on the offset and was the same before the rig was turned square to the walls. Possible causes: the BNO mounting, the tilt of the lidar mount, the floor itself.

== BNO085 mounting, calibration and magnetic heading

*Tilt.* The BNO mounting rotation relative to the rig frame (`bno_from_frame` in `RigParams`) is nominal (see above). A tilt of 5.6° (the BNO vertical against the rig Z axis) was fitted from a level pose with `reconstruction/scripts/calibrate_bno_mount.jl` (option `--tilt-only`, from gravity alone; the up direction scattered by 0.02° during the recording) and stored in `reconstruction/calibration/bno_mount.toml`. The file is loaded automatically when it exists. It was set aside as `bno_mount.toml.disabled` after the offset error was found, because the mounting is assumed flat; renaming it back restores the calibration. With the corrected offset and the flat mounting the floor is level within 1.1 to 1.3°, so there is no evidence for a BNO mounting tilt. Why the BNO vertical was about 5° from the rig Z axis in the calibration pose, although the surface was assumed level, is not explained.

*Heading.* The magnetometer is badly disturbed near the stepper motor (a compass deflects within 10 cm) and probably by the lidar. In scans the field strength was 60 to 73 µT (earth: about 45 to 50 µT in central Europe), the calibration status of the magnetometer was 0 and once 1 of 3, and in a recording on the calibrated rig it flickered between 0 and 2 with a reported heading accuracy of 28°. The field changes by less than 0.4 µT with the shaft angle, so the disturbance is fixed to the rig and can in principle be calibrated out (figure-8 movements with everything powered). A full fit of the heading offset gave −26° and was not used, because it is mostly magnetic error. The heading is therefore only a weak prior for north. The reconstruction is meant to use the gyroscope for the relative heading.

*Calibration helper sketch.* `firmware/bno_calibration/bno_calibration.ino` (Teensy 4.1) runs the BNO085 with dynamic calibration of accelerometer, gyroscope and magnetometer switched on and prints the state twice a second on the USB serial port: calibration status of all sensors, heading accuracy, field strength, the compass heading of the BNO y axis (the rig X axis) and the tilt. It stores the calibration in the BNO flash after the magnetometer has been at 3/3 for 10 s, or on the command `s`; `C` erases it. The motor driver pins are set as in `lidar_ctrl`. After calibrating, flash `lidar_ctrl` again.

#todo[Check whether the stored BNO calibration survives a power cycle (the status dropped to 0/3 at rest after a figure-8). Source of the 1.1 to 1.3° pitch residual. Decide between 90.3° and 89.8°. Put the floor check into the package as a script. Stage 2: gyroscope integration for the relative heading, translation and registration. Viewer.]

== Path estimation while moving (stage 2) <sec:odometry>

Untangling assumes that the rig stands still. This stage estimates the path of the rig while it is carried, so that the points of a moving scan can be put into one frame. Only four degrees of freedom are estimated: the position x, y, z and the yaw about the vertical. Roll and pitch come from the BNO085 gravity direction (see above) and are taken as exact. The frame is the rig frame at the start (z up, heading as the rig's own), the origin is the lidar at the start. Code: `reconstruction/src/` (`trajectory.jl`, `imu.jl`, `registration.jl`, `spline_odometry.jl`, `simulate.jl`) and `reconstruction/scripts/` (`simulate_path.jl`, `odometry.jl`, `spline_odometry.jl`, `plot_path.jl`).

*Principle.* The path is the IMU path plus a correction that is fitted to the lidar points: `p(t) = p_imu(t) + c(t)`. The IMU supplies the fast detail (strides, turns), the lidar supplies the slow drift. The BNO085 linear acceleration is integrated twice only over a short time, because over the whole recording it drifts to 100 m (bias, gravity leakage from a small tilt error, yaw error). The yaw rate of the gyroscope is good: the integrated yaw stayed within about 1° of the lidar-fitted yaw.

*Input.* The untangled points (`untangled.arrow`, gravity mode) and the rig files. `imu_track` turns the BNO linear acceleration and gyroscope into the same levelled rig frame as the points (rotation of the sensor into the rig frame, then the smallest rotation that takes up onto +Z, the same as for the lidar points). The recording must start with the rig standing still: the first seconds (`seed_s`, 31.5 s in the test scan) become the first map, and the IMU biases are the mean of the IMU at standstill.

*Map and loss.* The map is a voxel map (5 cm, one centroid per voxel) of all points placed so far, with a KD-tree. Each new point is matched to a local plane (the 8 nearest map centroids, all within 0.4 m, planar if the smallest eigenvalue is below 0.2 of the middle one) and gives a point-to-plane residual with Huber weighting (0.1 m), noise 3 cm. The solver is a hand-written Gauss-Newton with the correspondences found again in each iteration. The new points are fitted only against older points; they enter the map after the fit.

*Version 1: one correction per window* (`run_odometry`, `scripts/odometry.jl`). Windows of 1 s. Per window four unknowns: the displacement `dp` over the window (3) and a yaw offset `dpsi`, growing linearly over the window (a constant velocity and yaw rate offset), with a Gaussian prior around the IMU prediction (10 cm, 2°). The start pose of a window is the end pose of the previous one, and the velocity offset stays in the state for all later windows. The correction is continuous in position but has a velocity step at every window boundary. Windows of 0.5 s gave a path differing by 6 cm median, windows of 2 s failed (134 cm median difference).

*Version 2: cubic B-spline correction* (`run_spline_odometry`, `scripts/spline_odometry.jl`, the version to use). The correction `c(t) = (x, y, z, yaw)` is a uniform cubic B-spline with control points every `knot_s` (0.1 s or 0.2 s, one or two lidar head rotations), so a control point acts over four knot intervals only and the correction has continuous acceleration. The fit goes step by step: each step adds `step_s` of new points and the matching new control points, and keeps only these and the last `refine_knots` older control points free; everything older is frozen. The points enter the map only when all their control points are frozen. New control points start as the linear continuation of the previous ones (the velocity correction carries on). A smoothness prior on the second difference of the control points (the acceleration of the correction, 1 m/s#super[2] for the position and 0.3 rad/s#super[2] for the yaw) replaces the prior of version 1.

*Thinning.* The fit points are thinned to one per 20 cm voxel in the sensor frame and per 0.1 s slice (one lidar sweep). Thinning per voxel only, as in version 1, keeps just the earliest sweep of a window and leaves the rest unused; slicing raised the number of matched points per fit from about 260 to over 3000.

*Moving person.* A person who carries the rig moves with it. Two measures: the fit uses only points at least 1.5 m from the lidar (`fit_min_range`), and every map voxel logs the number of different 0.1 s segments (`segment_s`) that put points into it; only voxels hit in at least 3 segments (`min_segments`) are matched. A wall is hit again and again, a person who walks with the rig crosses a 5 cm voxel in about 0.2 s. A person who stands still counts as static.

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt + gray,
  [*Setting*], [*Default, meaning*],
  [`knot_s`], [0.2 s (tested 0.1 s and 0.2 s with the same result); spacing of the control points],
  [`step_s`, `refine_knots`], [0.4 s and 4 (0.2 s and 8 for 0.1 s knots); new data per step, older control points that stay free],
  [`sigma_acc`, `sigma_yaw_acc`], [1.0 m/s#super[2], 0.3 rad/s#super[2]; smoothness. 0.3 and 1.0 work, 3.0 diverged (path of 140 m): there is no safeguard against a failed step yet],
  [`map_voxel`, `fit_voxel`, `slice_s`], [5 cm, 20 cm, 0.1 s],
  [`fit_min_range`, `min_segments`, `segment_s`], [1.5 m, 3, 0.1 s in the scripts (the library defaults are 0 and 1, which switches the person filter off)],
)

*Synthetic test* (`scripts/simulate_path.jl`). The standstill scan `offset_90_3` was moved along a known path (8 s standstill, then about 2 m with stride bounce and a yaw swing of about 30°) and an IMU with noise and bias was simulated for it. The path was recovered with a median point error of 0.9 cm against 28 cm for the IMU alone (about 3 s run time for 7.8 M points). Not modelled: occlusion and the changed sampling density at the new viewpoint, the lever arm, tilt, and the noise values are guesses.

*Real moving scan* (`first_move/scan_01`, 20.3 M points, 6 rpm). The rig stands still until about 32 s, moves until about 80 s (yaw rate up to 40°/s, speed median 0.23 m/s, at most 0.46 m/s) and stands still again. Results of the spline version:

#table(
  columns: (auto, auto, auto, auto),
  stroke: 0.5pt + gray,
  [*Result*], [*Window (v1)*], [*Spline 0.2 s*], [*Spline 0.1 s*],
  [Moving points (from 1.5 m) to the planes of the standstill map, median / 90 %], [2.1 / 8.7 cm], [1.1 / 6.6 cm], [1.1 / 6.6 cm],
  [Fit error while moving (median, standstill: 1.4 cm)], [3.8 cm], [3.3 cm], [3.2 cm],
  [Run time], [6 s], [39 s], [64 s],
)

The path stays inside x −1.6 to 1.6 m, y −0.5 to 1.2 m, z 0 to 1.4 m and ends within 2 cm and 0.3° of the start. The path acceleration is 0.4 m/s#super[2] median. The spline path differs from the window path by 5 cm median and 18 cm at most (1 s steps). The BNO linear acceleration alone drifts to 100 m. The person filter changed the path by less than a centimetre, so the person was not what spoiled the fit in this scan. Changing the other settings (number of free older knots, step size, sigma 0.3 to 1.0) changed the result by very little.

*Viewing.* `scripts/plot_path.jl` writes `odometry_view.html`, a self-contained page (plotly.js from cdnjs) with the path in 3D, optional overlays of other paths, yaw and fit charts, and with `--points` the still map and the moving points coloured by time. `--main=NAME` reads `NAME_path.csv` (written every 0.1 s by the odometry scripts). The page was also published as an Artifact.

*Limits.*
- There is no ground truth. The room is small and the standstill map is visible all the time, so this is closer to localisation in a known room than to odometry without drift. The return to the start is therefore weak evidence. A cave passage that is new at every step will be harder.
- The result still looks like a double exposure to the eye. Suspects, not yet tested: the dynamic tilt (the BNO attitude lags or errs while the rig moves, and the fit cannot correct roll and pitch), the lever arm of the BNO (about −27 cm in x, 19 cm in y, −4.5 cm in z from the optical centre, not compensated), the timing between the shaft angle and the points.
- Along the axis of a uniform tunnel the geometry cannot determine the position; the result falls back to the IMU and the smoothness prior.
- Julia pitfall met on the way: `4f+1` is read as a `Float32` literal; write `4*f+1`.

#todo[Check the floor tilt over time with the spline path (dynamic tilt of the BNO). Compensate the BNO lever arm (`w` squared times `r` plus `alpha` times `r`). Safeguard for a failed fit step (fall back to the prior). Unit tests for `fit_window`, `fit_spline_step!` and `imu_track`. Test with a recording that leaves the standstill map (passage). Learn the biases online (`bias_gain`) on a longer scan. Global final pass over all control points. Measure how many points belong to the person. Decide voxel sizes (5 cm and 20 cm are untuned).]

= Appendix

== Change log

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt + gray,
  [*Date*], [*Change*],
  [2026-10-04], [Index sensor, step calibration (10:1) and vertical offset (93°) measured.],
  [2026-10-08], [Repository restructured. Wiring and control software chapters written for the final firmware `lidar_ctrl`.],
  [2026-10-09], [Lidar IP changed to 192.168.3.201 (separate /24 subnet, second Ethernet adapter on the PC). Lidar recorder written (Arrow stream files), tested on the rig. Lidar time: sub-second from the PPS-locked lidar, whole seconds from a rough reference. Teensy recorder and orchestrator CaveScan (CLI with live display) written and tested on the rig.],
  [2026-10-10], [Offset from the index edge to vertical corrected from 93.0° to 90.3° (the floor of a scan was tilted by 2.7° about the shaft axis). The reconstruction takes the value from the rig file metadata (`vertical_after_edge_deg`); recordings without it are processed with 93.0°. The recorder now waits for the stream header before it writes the first rig chunk, so the metadata is always there.],
  [2026-10-10], [Firmware protocol version 2: calibrated gyroscope added to the sample packet and the rig Arrow files (the game rotation vector was tried and dropped: it stalls the normal rotation vector). BNO calibration helper sketch `bno_calibration`; tilt of the BNO mounting calibrated (5.6°).],
  [2026-10-10], [Reconstruction package `CaveLidarRecon`: untangling of the shaft rotation plus BNO085 frame attitude, combined Arrow output.],
  [2026-10-10], [Path estimation while moving (stage 2): IMU path plus lidar-fitted correction, first as one correction per 1 s window, then as a cubic B-spline (knots 0.1 or 0.2 s) with a person filter (fit points from 1.5 m, map voxels hit in 3 time segments). Tested on a synthetic path and on the first moving scan; viewer page `odometry_view.html`.],
)
