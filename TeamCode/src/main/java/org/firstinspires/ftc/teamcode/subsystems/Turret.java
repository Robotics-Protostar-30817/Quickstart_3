package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;


/**
 * Controls the horizontal rotation of the shooter turret using an
 * Axon MINI Servo MK2.
 *
 * <p>The turret rotates the shooter horizontally so that the shooter
 * can point toward the center of a detected CELL.</p>
 *
 * <p>The servo is controlled using FTC normalized servo positions in
 * the range {@code [0.0, 1.0]}. A calibrated center servo position
 * corresponds to a turret angle of 0 degrees. Positive and negative
 * turret angles represent rotation to opposite sides of the robot.</p>
 *
 * <p>The conversion between turret angle and servo position depends
 * on the mechanical linkage and usable servo rotation range and must
 * therefore be calibrated on the physical robot.</p>
 */
public class Turret implements Subsystem {

    /**
     * Singleton turret subsystem instance.
     */
    public static final Turret INSTANCE = new Turret();

    /**
     * Servo that rotates the turret.
     */
    private Servo turretServo;

    /**
     * Servo position corresponding to the turret pointing straight
     * forward.
     *
     * <p>This is an initial value and should be calibrated on the
     * physical turret.</p>
     */
    private double centerPosition = 0.50;

    /**
     * Turret angle represented by the full usable servo-position range.
     *
     * <p>This placeholder assumes that changing the servo position by
     * 1.0 corresponds to 180 degrees of turret rotation. Replace this
     * value after measuring the actual mechanism.</p>
     */
    private double degreesPerServoRange = 180.0;

    /**
     * Minimum allowed turret angle in degrees.
     *
     * <p>The value should be adjusted after determining the safe
     * mechanical range of the turret.</p>
     */
    private double minAngleDegrees = -90.0;

    /**
     * Maximum allowed turret angle in degrees.
     *
     * <p>The value should be adjusted after determining the safe
     * mechanical range of the turret.</p>
     */
    private double maxAngleDegrees = 90.0;

    /**
     * Last requested turret angle in degrees.
     */
    private double targetAngleDegrees = 0.0;

    /**
     * <p>Positive y is to the robot's right. The current turret is mounted
     * approximately  the same forward/backward position to the robot center.</p>
     *
     * <p>This value should be replaced with the measured offset to center offset.</p>
     */
    private static final double TURRET_TO_CENTER_Y_INCHES = 0.0;
    /**
     * <p>Positive x is to the robot's right.
     * This assumes the turret rotation axis is located at the center.</p>
     *
     */
    private static final double TURRET_TO_CENTER_X_INCHES = 0.0;

    /**
     * Creates the singleton turret subsystem.
     */
    private Turret() {
    }


    /**
     * Initializes the Axon MINI Servo MK2 used to rotate the turret.
     *
     * <p>After initialization, the turret is commanded to the calibrated
     * center position, corresponding to a turret angle of 0 degrees.</p>
     *
     * @param servo turret servo obtained from the FTC hardware map
     */
    public void init(Servo servo) {

        turretServo = servo;

        targetAngleDegrees = 0.0;

        turretServo.setPosition(centerPosition);
    }


    /**
     * Sets the desired turret angle relative to the robot's forward
     * direction.
     *
     * <p>The requested angle is limited to the configured safe mechanical
     * range and converted to an FTC servo position.</p>
     *
     * @param angleDegrees requested turret angle in degrees
     */
    public void setTargetAngle(double angleDegrees) {

        if (turretServo == null) {
            return;
        }

        targetAngleDegrees = clamp(
                angleDegrees,
                minAngleDegrees,
                maxAngleDegrees
        );

        double servoPosition =
                centerPosition
                        + targetAngleDegrees
                        / degreesPerServoRange;

        servoPosition = clamp(
                servoPosition,
                0.0,
                1.0
        );

        turretServo.setPosition(servoPosition);
    }


    /**
     * Returns the most recently requested turret angle.
     *
     * <p>This is the commanded angle rather than an independently
     * measured physical turret angle.</p>
     *
     * @return requested turret angle in degrees
     */
    public double getTargetAngle() {
        return targetAngleDegrees;
    }


    /**
     * Returns the currently commanded FTC servo position.
     *
     * @return normalized servo position in the range {@code [0.0, 1.0]}
     */
    public double getServoPosition() {

        if (turretServo == null) {
            return centerPosition;
        }

        return turretServo.getPosition();
    }


    /**
     * Commands the turret to point straight forward.
     *
     * <p>The forward direction corresponds to a turret angle of
     * 0 degrees.</p>
     */
    public void center() {
        setTargetAngle(0.0);
    }


    /**
     * Sets the calibrated servo position corresponding to a turret
     * angle of 0 degrees.
     *
     * @param position normalized FTC servo position in the range
     *                 {@code [0.0, 1.0]}
     */
    public void setCenterPosition(double position) {

        centerPosition = clamp(
                position,
                0.0,
                1.0
        );
    }


    /**
     * Sets the measured turret angular range represented by a servo
     * position change of 1.0.
     *
     * <p>For example, if the physical mechanism rotates 180 degrees
     * while the servo command changes by 1.0, this value should be
     * 180 degrees.</p>
     *
     * @param degrees angular range in degrees per full servo-position range
     */
    public void setDegreesPerServoRange(double degrees) {

        if (degrees > 0.0) {
            degreesPerServoRange = degrees;
        }
    }


    /**
     * Sets the safe mechanical angular limits of the turret.
     *
     * @param minimumDegrees minimum permitted turret angle in degrees
     * @param maximumDegrees maximum permitted turret angle in degrees
     */
    public void setAngleLimits(
            double minimumDegrees,
            double maximumDegrees) {

        if (minimumDegrees < maximumDegrees) {
            minAngleDegrees = minimumDegrees;
            maxAngleDegrees = maximumDegrees;
        }
    }

    /**
     * Calculates the horizontal turret angle required to point toward the center
     * of the cell.
     *
     * <p>The supplied cell X and Y coordinates describe the detected cell
     * position relative to the robot center. The coordinates use the convention: </p>
     *
     * <ul>
     *     <li>+X = right</li>
     *     <li>+Y = forward</li>
     *     <li>+Z = up</li>
     * </ul>
     *
     * <p>The returned angle uses the turret convention:</p>
     *
     * <ul>
     *     <li>0 degrees = straight forward</li>
     *     <li>positive angle = toward robot right</li>
     *     <li>negative angle = toward robot left</li>
     * </ul>
     *
     * <p>The Cell Z coordinate is not used directly for horizontal turret rotation.</p>
     *
     * @param cell detected Cell containing camera-relative position
     * @return required horizontal turret angle in degrees.
     */
    public double calculateTurretAngle(Vision.Cell cell){
        if (cell == null){
            return 0.0;
        }

        return Math.toDegrees(Math.atan2(cell.getX(),cell.getY()));
    }

    /**
     * Performs periodic turret processing.
     *
     * <p>No periodic controller is required because the servo's internal
     * controller moves toward the commanded servo position.</p>
     */
    @Override
    public void periodic() {
        // Position control is performed internally by the servo.
    }


    /**
     * Limits a value to the specified inclusive range.
     *
     * @param value value to limit
     * @param minimum minimum permitted value
     * @param maximum maximum permitted value
     * @return limited value
     */
    private double clamp(
            double value,
            double minimum,
            double maximum) {

        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}