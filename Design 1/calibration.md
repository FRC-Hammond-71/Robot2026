# Swerve Calibration Guide

Hey! If you're reading this, you're about to calibrate the swerve drive. Do these steps **in order** — each one builds on the last. You'll want to run through this whenever you set up the robot fresh, after big mechanical changes, or before a competition.

Grab a laptop with Phoenix Tuner X and Driver Station ready to go.

---

## 1. CANcoder Offsets

This is how the robot knows which way each module is pointing. If these are off, the robot will crab-walk or drift sideways.

**You need to redo this if:**
- A module was removed, swapped, or rebuilt
- The robot took a hard hit that could've shifted something
- It drifts sideways when you push the stick straight forward (in robot-centric mode)
- A wheel is visibly crooked at startup

**What you need:** a straightedge (longer than the wheel), a laptop, USB to the roboRIO.

**Steps:**

1. Power on the robot, connect your laptop, and **disable** it in Driver Station. You need the modules to spin freely by hand.

2. Grab your straightedge and hold it along the side of the chassis, front to back. Rotate each module by hand until the wheel lines up perfectly parallel with the chassis edge. Do this for all 4 modules.
   - On MK4i modules, the bevel gear should face **inward** (toward the center) on every module.
   - Look at it from above. Even being off by a degree or two will cause noticeable drift — take your time here.

3. Open Phoenix Tuner X, select each CANcoder one at a time, and go to the Configs tab. Find "Magnet Offset." You want the reported position to read **0** (or very close) when the module is pointed straight forward. Note the current absolute position and adjust the offset accordingly.

4. Update the offsets in code. Open `src/main/java/frc/robot/generated/TunerConstants.java` and change these lines (values are in rotations):
   ```
   Line 135: kFrontLeftEncoderOffset   (CANcoder ID 21)
   Line 146: kFrontRightEncoderOffset  (CANcoder ID 24)
   Line 157: kBackLeftEncoderOffset    (CANcoder ID 22)
   Line 168: kBackRightEncoderOffset   (CANcoder ID 23)
   ```
   **Heads up:** TunerConstants.java gets overwritten if you re-run the Tuner X swerve generator. You can also set offsets through the Tuner X GUI instead if you prefer.

5. Deploy and test. Put the robot in **robot-centric mode**, push the stick straight forward, and watch. It should drive dead straight with no sideways drift. If it still drifts, at least one offset is wrong — go back and redo the module that looks off.

---

## 2. Wheel Radius (Spin-in-Place Method)

This is the single most important calibration for odometry. If the wheel radius is wrong, *every* distance measurement is wrong — and that error adds up fast. A 1% mistake here means 1% error on every single cycle.

The tricky part is that the "real" radius isn't the same as what's printed on the wheel. It depends on how much the tread compresses under the robot's weight, what surface you're on, and how worn the wheels are. So we measure it directly.

**How it works:**

The robot spins slowly in place. The gyro tells us exactly how far it actually rotated. The wheel encoders tell us how far the wheels think they traveled. Since we know how far each module sits from the center of the robot, we can solve for the wheel radius:

```
wheelRadius = (gyroDelta x driveBaseRadius) / wheelDelta
```

For our robot, the modules are at (10.375", 10.375") from center, so:
`driveBaseRadius = sqrt(10.375^2 + 10.375^2) = 14.672 inches = 0.3727 meters`

No tape measure needed. The gyro is our ground truth.

**Steps:**

1. Put the robot on the surface you'll actually compete on. Carpet compresses the wheel differently than concrete — always calibrate on the real surface. Clear about 4 feet of space around it.

2. On SmartDashboard, pick **"Wheel Radius Characterization"** from the Auto Mode chooser.

3. Enable autonomous. The robot will start spinning slowly in place (it ramps up gently, don't worry). You'll see the calculated radius updating live on SmartDashboard under `WheelRadiusCharacterization/WheelRadius`.

4. Let it go for at least **10 full rotations** — that's roughly 4 minutes. More is better. The number should settle down and stop jumping around. If it's still bouncing after 10 rotations, something might be off (see troubleshooting below).

5. Disable the robot. The final value gets printed to the console. Write it down.

6. You should get something in the range of **1.93" to 2.00"** depending on wheel wear and surface.

7. Update the value in `src/main/java/frc/robot/generated/TunerConstants.java`, line 88:
   ```java
   public static final Distance kWheelRadius = Inches.of(YOUR_VALUE_HERE);
   ```
   Use all the decimal places — don't round.

8. After updating, run the characterization one more time to double-check. The new measurement should land on the same number (confirming it's consistent).

**When to redo this:**
- Before every competition (wheels wear down)
- After swapping wheels or treads
- If the robot's been sitting for a while (tread gets flat spots)
- Anytime odometry seems off — undershooting or overshooting distances

**If something seems wrong:**
- Value won't settle? The robot might be slipping. Make sure the floor is clean and grippy.
- Way off from 1.985"? Double-check that the drive gear ratio (8.142857) is right for MK4i L1 in TunerConstants.
- Different on carpet vs concrete? That's totally normal. Use the carpet number.

---

## 3. Weight Distribution

If one side of the robot is heavier, those wheels compress more and grip differently. This makes the robot pull to one side when driving straight, and it means the effective wheel radius isn't the same on every corner — which quietly ruins your odometry.

**What you need:** 4 bathroom scales (matching is ideal), a level surface.

**Steps:**

1. Put one scale under each wheel. Make sure the robot is sitting naturally on a level surface — don't prop anything up weirdly.

2. Write down the weights:

| Corner | Weight (lbs) |
|--------|-------------|
| Front Left | _____ |
| Front Right | _____ |
| Back Left | _____ |
| Back Right | _____ |

3. Add up left vs right:
   - Left = Front Left + Back Left
   - Right = Front Right + Back Right
   - How far apart are they?

4. Add up front vs back too:
   - Front = Front Left + Front Right
   - Back = Back Left + Back Right

**What's acceptable:**
- Left-right difference should be **under 5 lbs**. Bigger than that and you'll get noticeable drift.
- Front-back can be a bit more relaxed — **under 10 lbs** is fine.

**How to fix it:**
- The battery is your best friend here — it's heavy and easy to move. Shift it toward the light side.
- You can also add ballast (steel plates, etc.) to the light corner.
- Rearranging electronics can help too, but that's usually more work than it's worth unless you're close.

---

## 4. Pigeon2 Gyro Check

The gyro is the backbone of your heading. If it's loose, miscalibrated, or drifting, everything downstream suffers — field-centric driving, auto paths, vision fusion, all of it.

**Steps:**

1. Power on and check SmartDashboard for "Pigeon Connected" = `true`. In Tuner X, confirm the Pigeon2 (CAN ID 20) shows up and the firmware is current.

2. With the robot on level ground, go to the Pigeon2 in Tuner X and run a Self-Test. You should see:
   - Pitch around 0
   - Roll around 0
   - Yaw increases when you rotate the robot counter-clockwise (looking down from above)

   If pitch and roll are way off, the Pigeon isn't mounted flat. Fix the mounting.

3. **Drift test:** Set the robot on a level, stable surface. Note the yaw. Walk away for 60 seconds. Don't touch it. Come back and check — the yaw should have moved less than **0.1 degrees**. If it drifted more, the mounting might be loose or the Pigeon might need recalibration (check CTRE's docs).

4. **Rotation test:** Put a piece of tape on the floor marking the robot's orientation. Carefully rotate the robot exactly 360 degrees by hand until it's lined back up with the tape. The yaw should read within **1 degree** of where it started.

---

## 5. Drive Motor SysId

This characterizes how your drive motors actually behave — how much voltage it takes to overcome friction, how voltage maps to speed, etc. You need this for accurate feedforward and PID tuning.

**When to do this:**
- After replacing a drive motor or gearbox
- If the robot feels sluggish or different than before
- When you're tuning kS, kV, or kP for drive

**Steps:**

1. Get the robot's wheels off the ground (put it on blocks or a stand) or find a big open space.

2. In Tuner X, open SignalLogger and hit "Start."

3. Run the SysId commands (they're already set up in the code for translation characterization):
   - **Quasistatic** — slow voltage ramp, forward and reverse
   - **Dynamic** — step voltage (4V), forward and reverse

4. Export the log from SignalLogger and load it into the WPILib SysId analysis tool.

5. The tool gives you `kS` (static friction voltage) and `kV` (volts per meter/second). Plug those into TunerConstants.java, lines 35-37:
   ```java
   .withKS(YOUR_KS).withKV(YOUR_KV)
   ```

6. You may need to retune `kP` after this. Start at 0.1 (what we have now). Bump it up if the robot isn't tracking velocity well. Back it down if modules start vibrating or oscillating.

---

## 6. Pre-Match Checklist

Run through this before every single match. No exceptions.

- [ ] Battery voltage > 12.5V
- [ ] Pigeon2 connected (SmartDashboard shows "Pigeon Connected" = true)
- [ ] All 4 CANcoders reading reasonable angles (no NaN, no stuck at 0)
- [ ] All 4 drive motors respond (quick enable in test mode)
- [ ] Correct starting pose selected in SmartDashboard
- [ ] Correct auto routine selected
- [ ] Limelight is on and pipeline is active
- [ ] No CAN errors in Driver Station diagnostics
- [ ] Robot is physically placed at the right starting spot on the field
- [ ] Bumpers on, robot is within starting config

---

## 7. Troubleshooting

| What's happening | Probably this | Do this |
|---|---|---|
| Robot drifts sideways driving straight | CANcoder offsets are stale | Redo Section 1 |
| Drifts in robot-centric but not field-centric | Weight imbalance + open-loop voltage | We switched to closed-loop velocity; also check Section 3 |
| Odometry undershoots distances | Wheel radius in code is too big | Redo Section 2 on carpet |
| Odometry overshoots distances | Wheel radius in code is too small | Redo Section 2 |
| Pose jumps around when vision kicks in | Odometry std devs are too loose | Tighten values in Constants.java (Odometry class) |
| Auto starts in the wrong spot | Starting pose not set properly | Check SmartDashboard pose chooser; verify resetPose is working |
| Modules twitch at low speed | Steer PID is too aggressive, or open-loop voltage mode | Check steer gains; make sure we're on closed-loop velocity |
| One module won't track its angle | Bad CANcoder offset or a wiring issue | Redo Section 1 for just that module; check wiring |
| Robot veers off during auto paths | Combination of offset + radius errors | Run Sections 1, 2, and 3 in order |
| Battery browns out mid-match | Drawing too much current | Lower the slip current limit or add current limiting |
