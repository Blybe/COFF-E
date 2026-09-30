# COFF-E Robotic Project Kickoff

> Working document based on *COFF-E - Plan van Aanpak*, version 1.3, dated
> 29 September 2026. Fill in the questions marked **ANSWER NEEDED** before
> committing to a final architecture.
>
> Updated after reviewing the previous-project archive
> `Koffie-halen-autonoom--main`. Findings from that archive are marked
> **PREVIOUS PROJECT**. They describe an earlier setup and must be checked on
> the current hardware before use.

## 1. Project idea

COFF-E should combine an Avular Origin One mobile platform and a UFactory xArm 6
robot arm into one safe, repeatable coffee collection workflow.

The intended high-level flow is:

1. Start in a known home position.
2. Confirm that the platform, arm, gripper, sensors and emergency stops are ready.
3. Drive to a known position near the coffee machine.
4. Align the platform accurately enough for the arm to work.
5. Use the arm to scan an access card if required.
6. Place a cup and operate or wait for the coffee machine.
7. Pick up the filled cup.
8. Put the arm and cup in a safe transport pose.
9. Drive back to the home position.
10. Deliver the cup and return the system to a neutral state.

The coffee machine itself is outside the planned error-handling scope. The
system should still stop safely and report a failure when the expected coffee
machine result does not occur.

## 2. Recommended first scope

Do not start with the complete coffee workflow. Prove these building blocks
independently:

### Building block A: xArm 6 connection

- Connect to the arm from a small Python program.
- Read the arm state and current pose.
- Enable motion only after an explicit operator command.
- Move between two safe poses at low speed.
- Stop and recover from a controlled error.
- Open and close the gripper without a cup.

**Done when:** ten repeated low-speed runs complete without unexpected motion,
and an operator can stop the arm immediately.

### Building block B: Origin One connection

- Connect to the mobile platform using its supported interface.
- Read platform, battery, localization and sensor health.
- Command a short movement in a cleared test area.
- Send the platform to one saved waypoint and back.
- Stop when an obstacle or software timeout is introduced.

**Done when:** the platform reaches a test waypoint and returns repeatedly
within a measured position and heading tolerance.

### Building block C: dry cup manipulation

- Use an empty, unbreakable test cup.
- Define approach, grasp, lift, transport and place poses.
- Establish gripper force and cup-size limits.
- Test loss-of-grip detection if the available hardware supports it.

**Done when:** at least 20 pick-and-place cycles complete without dropping or
damaging the cup.

### Building block D: platform-to-arm handoff

- Park the Origin One at a repeatable test marker.
- Confirm that the platform is stationary.
- Confirm that the arm workspace is clear.
- Allow arm motion only while the base remains locked/stationary.
- Test how parking errors affect the arm target.

**Done when:** the arm can reach a fixed test object after repeated parking
runs, or the software reliably refuses when alignment is outside tolerance.

Only after these blocks work should the team add card scanning, machine
interaction, liquid and the complete route.

## 3. Proposed software structure

Start with one Python repository but keep hardware-specific code behind small
interfaces. This allows development with mocks when the real robots are not
available.

```text
Project-COFF-E/
├── README.md
├── PROJECT_KICKOFF.md
├── pyproject.toml
├── config/
│   ├── development.yaml
│   └── robot.yaml
├── docs/
│   ├── architecture.md
│   ├── safety.md
│   ├── setup.md
│   └── operator-guide.md
├── src/coffe/
│   ├── main.py
│   ├── workflow.py
│   ├── state_machine.py
│   ├── config.py
│   ├── safety.py
│   ├── platforms/
│   │   ├── base.py
│   │   ├── avular.py
│   │   └── mock.py
│   ├── arms/
│   │   ├── base.py
│   │   ├── xarm6.py
│   │   └── mock.py
│   └── perception/
│       ├── alignment.py
│       └── cup_detection.py
├── scripts/
│   ├── test_xarm_connection.py
│   └── test_avular_connection.py
└── tests/
```

This structure is a proposal. The previous project confirms that the Origin One
used ROS 2, but the ROS 2 distribution, messages and command interface still
need to be identified on the current platform.

## 4. Integration concept

Use a state machine rather than one long script. A possible first version is:

```text
IDLE
  -> PRE_FLIGHT_CHECK
  -> NAVIGATE_TO_MACHINE
  -> ALIGN_WITH_MACHINE
  -> SCAN_ACCESS_CARD
  -> PLACE_CUP
  -> WAIT_FOR_COFFEE
  -> PICK_UP_CUP
  -> STOW_ARM
  -> NAVIGATE_HOME
  -> DELIVER_CUP
  -> RETURN_TO_NEUTRAL
  -> COMPLETE
```

Every active state should also be able to transition to:

- `PAUSED`: recoverable interruption or manual hold;
- `FAILED`: task failed but the system is in a controlled state;
- `EMERGENCY_STOPPED`: immediate safety stop requiring operator inspection.

The orchestrator should issue high-level commands such as `navigate_to()`,
`move_to_safe_pose()` and `grasp_cup()`. Hardware adapters should translate
these into the official Avular and UFactory APIs.

### Important coordination rule

The xArm must not move while the Origin One is driving in the first prototype.
Before arm motion:

1. the platform reports zero motion;
2. the platform is inside the allowed parking tolerance;
3. navigation commands are blocked;
4. arm and gripper health are valid;
5. the operator safety zone is clear.

## 5. Communication and computing needs

These details must be confirmed on the real hardware:

- Network layout and IP addresses of the Origin One, xArm controller and
  development computer.
- Whether the external NVIDIA Jetson Orin Nano Super from the previous project
  is still available and should run the main application.
- Supported Origin One API, SDK, ROS/ROS 2 distribution and message/action
  definitions.
- UFactory SDK version, xArm firmware version and supported Python version.
- Gripper model and its control/feedback capabilities.
- Available time synchronization between computers.
- Coordinate frames and calibration method between the mobile base, arm,
  gripper, cameras and coffee machine.
- Logging and telemetry storage location.

**PREVIOUS PROJECT findings:**

- The application ran on an external NVIDIA Jetson Orin Nano Super with Linux
  installed on an NVMe SSD. The Origin One's internal computers were
  deliberately not modified.
- ROS 2 communication required
  `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- The previous student could read Origin One data but had not yet succeeded in
  sending movement commands.
- Cerebra Studio 3.0.2 installers for Windows and Linux are included in the
  archive. The archive also links Origin One manuals 1.0-3 and mentions 1.0-4.
- The xArm was controlled in one experiment through control-box digital inputs
  using Jetson GPIO and relay/transistor circuitry. The control-box inputs were
  documented as 24 V inputs switched to ground. This is not a replacement for
  confirming the official UFactory Python API and must not be rewired or reused
  without checking the current schematic, isolation and electrical ratings.

Keep credentials, access-card data and fixed network secrets out of Git.
Configuration should come from local environment variables or untracked
configuration files.

## 6. Positioning and perception thoughts

Normal mobile navigation accuracy may not be enough for cup manipulation. The
project probably needs two levels of positioning:

1. **Coarse navigation:** Origin One drives to a waypoint near the machine.
2. **Fine alignment:** a camera, LiDAR feature, fiducial marker or docking
   fixture determines the final relative pose.

For a first proof of concept, a physical alignment aid or an AprilTag/ArUco
marker near the machine is simpler and more repeatable than general object
detection. This must be approved for the lab environment.

Measure the actual error instead of choosing a tolerance in advance:

- repeat parking at least 20 times;
- record position and heading error;
- compare the worst-case error with the xArm's safe reachable workspace;
- add a refusal threshold when the target pose is unsafe or unreachable.

## 7. Safety needs before hardware testing

The combined mobile base, six-axis arm, gripper and hot liquid create more risk
than either robot alone. Before powered testing, agree on:

- a responsible supervisor for the first motion tests;
- locations and behavior of all emergency-stop controls;
- a cleared and marked test area;
- low initial speed, acceleration, joint and workspace limits;
- collision detection settings;
- safe arm poses for driving and shutdown;
- maximum cup mass, dimensions, fill level and liquid temperature;
- what happens after network loss, localization loss or software timeout;
- battery and power distribution limits with the arm mounted;
- payload, center-of-gravity and stability limits for the Origin One;
- a no-liquid test phase using an empty, unbreakable cup;
- an operator checklist and incident log.

Do not test with hot liquid until dry manipulation, navigation, braking and
emergency-stop behavior have passed agreed acceptance tests.

## 8. Information needed from the team

Answer these questions in this document. Short answers are enough for the next
design iteration.

### Hardware

1. **PARTLY ANSWERED:** The xArm 6 was previously mounted on the Origin One.
   An unstable wooden mount was replaced by a laser-cut aluminium plate mounted
   high enough not to obstruct the LiDAR. The archive also contains CAD/STL
   files for a newer 80-20 aluminium frame. **ANSWER NEEDED:** Which mount is
   currently fitted, is it approved, and what is the measured base-to-arm
   transform?
2. **ANSWER NEEDED:** Which exact xArm gripper/tool is installed?
3. **PARTLY ANSWERED:** The project plan lists LiDAR, Intel RealSense and
   ultrasonic sensors on the Origin One, plus RealSense and Raspberry Pi
   cameras available in the lab. The previous vision test used an external
   camera as OpenCV camera index 1. **ANSWER NEEDED:** Which sensors are
   currently fitted and what are their mounting transforms?
4. **ANSWER NEEDED:** Is an emergency stop available that stops both systems?
   The xArm's built-in collision/contact stop was mentioned, but that does not
   confirm a combined emergency stop.
5. **PARTLY ANSWERED:** The old wiring supplied 24 V to the xArm control box and
   used a 24-to-19 V converter for the external Jetson. **ANSWER NEEDED:** Did
   the 24 V originate from the Origin One battery or a separate supply, and is
   this wiring still present and approved?
6. **ANSWER NEEDED:** What cup sizes, materials and maximum filled mass must be
   supported?

### Software and access

7. **PARTLY ANSWERED:** The archive contains Cerebra Studio 3.0.2 installers,
   an Origin One 1.0-3 manual link and a reference to 1.0-4 documentation.
   **ANSWER NEEDED:** Which manual matches the current platform and where are
   the ROS 2 command/message examples?
8. **PARTLY ANSWERED:** The Origin One uses ROS 2. The previous Jetson setup
   required `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. **ANSWER NEEDED:** Record
   the current ROS distribution using `echo $ROS_DISTRO`, then identify the
   topics, services and actions for movement.
9. **PARTLY ANSWERED:** The plan states that the arm worked with UFactory's
   Python API, but the archive contains no SDK code or version information.
   **ANSWER NEEDED:** Confirm a current connection and record the SDK and
   firmware versions.
10. **PROVISIONAL ANSWER:** Run the main application on the external NVIDIA
    Jetson Orin Nano Super, as the previous project did. Do not modify the
    Origin One's internal computers until Avular or the lab confirms that it is
    supported. **ANSWER NEEDED:** Confirm that this Jetson is available and
    approved by Timo.
11. **PARTLY ANSWERED:** There is no source code in the archive. Its PDF only
    describes a YOLOv8/OpenCV camera test and a Jetson GPIO control test.
    **ANSWER NEEDED:** Ask Chris or Timo whether the original scripts exist
    elsewhere. Reuse the concepts only after checking them against current
    documentation.
12. **PARTLY ANSWERED:** The previous external Jetson used Linux, Python,
    PyTorch, YOLOv8, OpenCV, Jetson GPIO and ROS 2. Development/flashing was
    done from Windows. **ANSWER NEEDED:** Record the exact JetPack/Ubuntu,
    Python, ROS 2 and package versions on the current system.

### Environment and workflow

13. **PARTLY ANSWERED:** The coffee machine is in the lab and the home location
    is described only as a lab start position. **ANSWER NEEDED:** Mark the exact
    start, machine, access-panel and delivery poses on a map.
14. **PARTLY ANSWERED:** The plan identifies doors, a standard 90 cm opening and
    floor seam/threshold height as navigation risks. **ANSWER NEEDED:** Walk the
    complete route and record all door widths, thresholds, ramps, lifts and
    public corridors.
15. **ANSWER NEEDED:** Who opens doors, or must this be autonomous?
16. **PARTLY ANSWERED:** The planned workflow has the arm present an access card
    to a fixed access panel and wait for approval. A risk section also mentions
    an RFID/NFC sensor. **ANSWER NEEDED:** Confirm whether the reader is the
    existing access panel or a sensor mounted on the robot.
17. **PARTLY ANSWERED:** The plan says the robot scans the card and takes a cup
    from the control panel. It does not say whether the robot presses buttons
    or places the cup under the machine. **ANSWER NEEDED:** Describe every
    physical action and which actions remain manual.
18. **ANSWER NEEDED:** How can the software detect that the cup is present and
    that coffee preparation is complete? The previous YOLOv8 test is only a
    possible starting point and was not a completed detector.
19. **ANSWER NEEDED:** Is adding a marker or docking aid near the machine
    allowed?
20. **PROVISIONAL ANSWER:** Yes. The previous project was explicitly designed
    for potentially busy shared spaces. Confirm the allowed operating times,
    exclusion zones and required human-detection behavior with the lab.

### Acceptance and planning

21. **PARTLY ANSWERED:** The plan targets a working project and final report by
    mid-December, before the Christmas break, but does not define the minimum
    demonstration. **ANSWER NEEDED:** Agree on the exact demonstration with
    Timo.
22. **ANSWER NEEDED:** What success rate and positioning tolerance are required?
23. **PARTLY ANSWERED:** The plan proposes retry plus manual override for a
    failed card scan. **ANSWER NEEDED:** Define recovery behavior for all other
    failures.
24. **PROVISIONAL ANSWER:** The plan schedules two days per week in
    September–October and three to four days per week from November, with a
    Friday 14:00 feedback session. Confirm current access because the previous
    Origin One also lost significant time to hardware repairs.
25. **PROVISIONAL ANSWER:** Timo, the lab manager/project supervisor, sets the
    acceptance criteria and supervises first tests. Confirm that Timo also
    signs the final safety checklist.
26. **PROPOSED ANSWER:** Keep code and technical documentation in English and
    make the operator manual bilingual (Dutch and English). Confirm with Timo;
    the plan currently says both English-only documentation and a Dutch/English
    user manual.

## 9. Previous-project assets and limits

The archive is useful as background, but it is not a runnable software project.

### Archive sources reviewed

- `Koffie-halen-autonoom--main/Documentatie/HVAfvalruim bot Chris2 (1).pdf`
- `Koffie-halen-autonoom--main/Documentatie/Student PvA's/20-09-26_COFF•E-PvA_IanK_RobinvdD.pdf`
- `Koffie-halen-autonoom--main/Documentatie/Source Links`
- `Koffie-halen-autonoom--main/Softwares/Software_links`
- `Koffie-halen-autonoom--main/CAD files/`

### Available

- A report by Chris Kuijper describing the previous xArm/Origin experiments.
- Origin One and UFactory documentation links.
- Cerebra Studio 3.0.2 installers for Windows and Linux.
- Onshape link, SolidWorks model and STL files for mounting structures.
- Notes about Jetson setup, ROS 2 middleware, vision and GPIO experiments.

### Not available

- Python source code or ROS 2 launch/configuration files.
- Dependency files or pinned versions.
- Automated tests.
- Robot IP addresses, ROS topic names or movement examples.
- xArm SDK and firmware versions.
- Base-to-arm or camera calibration data.
- A documented, tested combined emergency-stop design.

### Reuse decision

Reuse the mechanical models and documented setup clues only after inspecting the
current hardware. Ask the previous team for the original scripts. Prefer the
official ROS 2 and UFactory interfaces over rebuilding the experimental GPIO
control path. If GPIO remains necessary, treat it as a separate electrical
safety task requiring a reviewed schematic and controlled bench test.

## 10. Suggested milestones

| Milestone | Result |
|---|---|
| M0: Inventory | Versions, interfaces, wiring, network and safety controls documented |
| M1: Arm prototype | Safe connect, status, low-speed motion, gripper and stop |
| M2: Base prototype | Safe connect, status, waypoint navigation and stop |
| M3: Dry manipulation | Reliable empty-cup pick, transport and place |
| M4: Repeatable docking | Base parks within measured arm-working tolerance |
| M5: Integrated dry run | Complete workflow with no liquid and manual machine trigger |
| M6: Machine interaction | Card/button/cup operations integrated |
| M7: Controlled liquid run | Cold liquid first, then hot liquid after safety approval |
| M8: Final demonstration | Agreed success criteria met and documentation delivered |

For every milestone, save:

- test setup and software version;
- expected result;
- measured result;
- failures and observations;
- video or photos when permitted;
- decision for the next iteration.

## 11. First lab session checklist

- [ ] Photograph and identify every hardware component and connector.
- [ ] Record serial numbers, firmware, SDK, OS and Python versions privately.
- [ ] Obtain official Avular and UFactory documentation.
- [ ] Draw the network and power layout.
- [ ] Locate and test emergency stops with the supervisor.
- [ ] Confirm arm payload, reach and platform stability limits.
- [ ] Confirm the gripper model and feedback.
- [ ] Establish a marked, empty test zone.
- [ ] Connect to the xArm and read status without commanding motion.
- [ ] Connect to the Origin One and read status without commanding motion.
- [ ] Agree on the first low-speed motion test.
- [ ] Record answers to Section 8.
- [ ] Check whether Chris or Timo has the missing Python/ROS 2 source code.
- [ ] Run `echo $ROS_DISTRO` on the external Jetson.
- [ ] Inventory ROS 2 nodes, topics, services and actions without commanding motion.
- [ ] Confirm whether the external Jetson Orin Nano Super is still available.
- [ ] Identify which Origin One manual version matches the current robot.
- [ ] Inspect the current arm mount and measure the base-to-arm transform.
- [ ] Review the old 24 V/GPIO interface before applying power.

## 12. Decisions to avoid making too early

Do not yet assume:

- that both devices can be controlled by the same Python process;
- that navigation accuracy is sufficient for manipulation;
- that camera-based object detection is necessary;
- that an RFID/NFC reader is part of the robot;
- that the xArm may safely move while the platform is moving;
- that hot coffee is acceptable for the first integrated test;
- that previous code or unofficial APIs are reliable.

Confirm these points with documentation and small hardware experiments first.
