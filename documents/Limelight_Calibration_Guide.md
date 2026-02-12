# Team 1197 Torbots: Limelight Field Calibration Guide (MegaTag2)

**Goal:** Ensure the Limelight knows *exactly* where it is on the robot so it can tell the robot *exactly* where it is on the field.

## Why Calibrate?
If your Limelight thinks it is 1 inch forward of where it actually is, your robot's entire positioning system will be off by 1 inch (or more as you rotate). MegaTag2 relies heavily on accurate "3D Offset" transforms.

## Prerequisite: The "Field-Relative" Method
The most accurate way to do this is NOT with a tape measure. It is to use the field itself.

1.  **Place the Robot:** Put the robot on the field in a known location (e.g., directly in front of the Speaker or Amp).
2.  **Align Perfectly:** Square the robot up against a field element so you know its rotation is perfectly 0, 90, or 180.
3.  **Open Limelight UI:** Connect to your Limelights via web browser:
    *   **Front:** `http://10.11.97.11:5801`
    *   **Left:**  `http://10.11.97.12:5801`
    *   **Right:** `http://10.11.97.13:5801`
    *(Note: adjust IPs if we used .14, .15, etc)*

## Step-by-Step Calibration

### 1. The "Tape Measure" Baseline
*   **Measure:** Use a tape measure to find the camera's position relative to the **center of the robot's wheels**.
    *   **Forward (X):** Positive = Front of robot.
    *   **Right (Y):** Positive = Right of robot.
    *   **Up (Z):** Positive = Up from floor.
*   **Input:** Enter these numbers into the "3D Offset" tab in Limelight. These are your starting points.

### 2. The "MegaTag2" Tune (The Magic Step)
1.  **Enable Robot Code:** Ensure the Pigeon2 Gyro is working (Robot rotation on dashboard matches reality).
2.  **Look at a Tag:** Point the robot so the Limelight sees an AprilTag.
3.  **Check "BotPose":** Look at the **Field Space** visualizer in the Limelight UI.
4.  **Compare & Tweak:**
    *   **Does the visualizer show the robot clipped inside the Speaker?**
        *   -> Adjust your **Forward (X)** offset until it pops out to the correct spot.
    *   **Does the visualizer show the robot floating in the air?**
        *   -> Adjust your **Up (Z)** offset.
    *   **Does the visualizer show the robot shifted to the left/right?**
        *   -> Adjust your **Right (Y)** offset.
5.  **Rotate Test:** Spin the robot 90 degrees. Does the position "slide" or "jump"?
    *   -> If yes, your **Yaw (Rotation)** offset is wrong. Tweak it until the robot spins in place on the map without translating.

## Verification
*   Drive the robot to the middle of the field (e.g. Center Line).
*   Check your driver station Shuffleboard.
*   Does the field map show the robot on the line?
*   If yes, **SAVE** the Limelight configuration!

> [!TIP]
> **Cover the Lenses!** When calibrating one camera, cover the other two with a Torbots shirt or rag. This ensures you are only seeing data from the specific camera you are tuning.
