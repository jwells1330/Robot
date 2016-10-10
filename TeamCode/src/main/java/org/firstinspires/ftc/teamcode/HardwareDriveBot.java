package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *  @author Jochen Fischer
 *  @version 1.0
 *
 *  Hardware definition for the Elon Drivebot.
 *
 *  The robot has a simple drive base with 2 motors:
 *      motorLeft
 *      motorRight
 *
 */
public class HardwareDriveBot
{
    /* Public OpMode members. */
    public DcMotor  motorLeft   = null;
    public DcMotor  motorRight  = null;
    public TouchSensor touchSensor = null;
    public ColorSensor colorSensor = null;
    public GyroSensor gyroSensor = null;

    // useful constants:
    public static final double SLOW_POWER = 0.2;
    public static final double POWER = 1.0;
    public static final double STOP = 0.0;
    public static final int ENC_ROTATION = 1120;
    public static final double WHEEL_DIAMETER = 4.0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareDriveBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeft   = hwMap.dcMotor.get("motorLeft");
        motorRight  = hwMap.dcMotor.get("motorRight");
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);

        // reset encoders
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run with encoders.
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /**
         * Setup Sensors
         */
        touchSensor = hwMap.touchSensor.get("touchSensor");
        colorSensor = hwMap.colorSensor.get("colorSensor");
        gyroSensor = hwMap.gyroSensor.get("gyroSensor");

        colorSensor.enableLed(true);
    }

    // Stop both motors
    void stop() {
        motorLeft.setPower(HardwareDriveBot.STOP);
        motorRight.setPower(HardwareDriveBot.STOP);

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

