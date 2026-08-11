# AGROBOT Navigation V1 Status

Date: 2026-08-11

## Final status

Navigation V1 is validated for the prototype.

## Validated features

- ROS2 motor bridge works.
- ESP32 receives movement commands.
- Encoder odometry /odom works.
- LiDAR /scan works.
- AMCL localization works with the saved map.
- Safe forward movement works using /odom.
- Calibrated turn commands work.
- goto X Y map navigation command works.
- LiDAR obstacle safety works during navigation.

## Important tests

Forward distance test:
- Command: forward 1.00
- Result: Forward reached, moved=1.01 m, target=1.00 m.
- Real movement was approximately 0.85 m to 1.00 m, acceptable for the prototype.

Map goto test:
- Command: goto 0.30 0.00
- Result: Goto reached enough, distance=0.08 m, tolerance=0.18 m.
- This validated map-based navigation using AMCL position, odometry movement, and LiDAR safety.

Goto with obstacle safety:
- Command: goto 0.30 0.30
- Result: Turn reached, turned=42.3 deg, target=42.1 deg.
- Result: Goto cancelled because obstacle ahead was detected at 0.31 m.
- The obstacle was physically real, so LiDAR safety behavior was correct.

## Practical calibration

For this robot prototype:

- forward 1.00 is about 0.85 to 1.00 m real movement.
- right 45 is about real 90 degrees.
- left 45 is about real 90 degrees, depending on floor slip.

## Conclusion

Navigation V1 is good enough for the AGROBOT prototype.

The robot can use a saved map, AMCL localization, odometry movement, calibrated turns, and LiDAR safety to execute simple goto X Y navigation commands.

Full industrial-level autonomous navigation remains a future improvement because wheel slip affects exact turning accuracy.

## Next step

Move to AI vision and plant detection.
