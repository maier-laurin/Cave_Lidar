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

#todo[Exact lidar model, mass, power supply, interface.]

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

A light barrier detects the passage of a flag on the lidar shaft once per revolution. It sits after the belt on the lidar shaft, so skipped belt teeth appear as a changed step count per revolution. The sensor edge occurs when the lidar is $93 degree$ (8266 steps) before vertical in the scan direction.

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
  [`VERTICAL_AFTER_EDGE_DEG`], [93.0°], [Angle between the index edge and the vertical lidar position (measured).],
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

The edge occurs when the lidar is 93° before the vertical position. The angle of the lidar for a given counter value is therefore

$ theta = (frac(n dot 360 degree, 31999) - 93 degree) mod 360 degree $

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

The BNO085 is read over I2C at 400 kHz. Three reports are enabled, each at 100 Hz:

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt + gray,
  [*Report*], [*Content*],
  [Rotation vector], [Orientation quaternion, referenced to magnetic north (the magnetometer is used), with an accuracy estimate.],
  [Linear acceleration], [Acceleration in the sensor frame with gravity removed, in m/s#super[2].],
  [Calibrated magnetic field], [Magnetic field in the sensor frame, in µT, with a calibration status.],
)

One data packet is sent per rotation-vector report. The most recent linear acceleration and magnetic field values are attached to it. They are not interpolated, so they can be up to one report interval (10 ms) older than the quaternion.

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
  [0], [u8], [version], [Protocol version, currently 1.],
  [1], [f32], [steps_per_rev], [Steps per lidar revolution (31999).],
  [5], [f32], [vertical_after_edge_deg], [Angle from index edge to vertical (93.0).],
  [9], [f32], [rpm], [Configured speed.],
  [13], [u32], [first_pulse_unix_s], [UTC label of the first PPS pulse.],
)

=== `S` sample packet (57 bytes, 100 per second)

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
)

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

= Reconstruction and viewing software

This part of the software is being rewritten and is not documented yet.

#todo[Coordinate frames, calibration of the lidar-to-IMU transform, fusion of IMU orientation with the shaft angle, point cloud reconstruction pipeline, viewer.]

= Appendix

== Change log

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt + gray,
  [*Date*], [*Change*],
  [2026-10-04], [Index sensor, step calibration (10:1) and vertical offset (93°) measured.],
  [2026-10-08], [Repository restructured. Wiring and control software chapters written for the final firmware `lidar_ctrl`.],
)
