package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderDriveArm extends LinearOpMode {
    HardwarePushbot_TC robot = new HardwarePushbot_TC();
    static final double COUNTS_PER_MOTOR_REV = 537.6;  // 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;   // 1  // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 5.25;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.3;
    private ElapsedTime runtime =  new ElapsedTime();
    double speed;
    double arvmovement;
    double timeoutS;

    public void encoderDriveArm(double speed, double armmovement, double timeoutS)
    {
        int newArmTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newArmTarget = robot.ArmMotor.getCurrentPosition() + (int) (armmovement * COUNTS_PER_INCH);

            robot.ArmMotor.setTargetPosition(newArmTarget);


            // Turn On RUN_TO_POSITION
            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.ArmMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&  robot.ArmMotor.isBusy() ) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d ", newArmTarget );
                telemetry.addData("Path2", "Running at %7d ",  robot.ArmMotor.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            robot.ArmMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
