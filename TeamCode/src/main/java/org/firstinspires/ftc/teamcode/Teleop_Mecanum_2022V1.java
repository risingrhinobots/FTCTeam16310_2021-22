/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleopMecanum_2022_V3", group="2022")
//@Disabled
public class Teleop_Mecanum_2022V1 extends LinearOpMode {

    // Declare OpMode members.
    EncoderDriveArm encoderDriveArm = new EncoderDriveArm();
    HardwarePushbot_TC robot = new HardwarePushbot_TC();
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private CRServo CarouselServo = null;
    private DcMotor ArmMotor = null;
    private Servo ClawServo = null;
    private Servo ClawReachServo = null;
    static final double COUNTS_PER_MOTOR_REV = 537.6;  // 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;   // 1  // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 5.25;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    static final double INCREMENT = 0.005;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 1000;     // period of each cycle
    static final double MAX_POS = 0.6;     // Maximum rotational position
    static final double MIN_POS = 0.4;     // Minimum rotational position
    static final double CLAW_MAX_POS = 0.75;     // Maximum rotational position
    static final double CLAW_MIN_POS = 0.95;     // Minimum rotational position
    // Define class members

    double position = 0.6;
    double ClawReachPosition = 0.3;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRight");
        ClawServo = hardwareMap.get(Servo.class, "Claw");
        ClawReachServo = hardwareMap.get(Servo.class, "ClawReach");
        CarouselServo = hardwareMap.get(CRServo.class, "Carousel");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);

        ArmMotor.setDirection(DcMotor.Direction.REVERSE);
        ClawReachServo.setPosition(ClawReachPosition);
        ClawServo.setPosition(position);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double FrontLeftPower;
            double FrontRightPower;
            double BackLeftPower;
            double BackRightPower;
            double ArmPower;

            double drive= 0;
            double turn =0;
            double strafe = 0;


            drive =   (-gamepad1.left_stick_y);
            turn  =  (gamepad1.left_stick_x);
            strafe =  (gamepad1.right_stick_x);


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.
            double denominator = Math.max(Math.abs(drive)+Math.abs(turn)+Math.abs(strafe),1);

            FrontLeftPower    = Range.clip(drive + turn + strafe, -1, 1) ;
            BackLeftPower    = Range.clip(drive - turn + strafe, -1, 1) ;
            FrontRightPower   = Range.clip(drive - turn - strafe, -1, 1) ;
            BackRightPower   = Range.clip(drive + turn - strafe, -1, 1) ;


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            FrontLeftDrive.setPower(FrontLeftPower);
            FrontRightDrive.setPower(FrontRightPower);
            BackLeftDrive.setPower(BackLeftPower);
            BackRightDrive.setPower(BackRightPower);


            if (gamepad1.dpad_up) {
                InLineEncoderDriveArm(ArmMotor, 0.1, -2, 5);
            }
            if (gamepad1.dpad_down) {
                InLineEncoderDriveArm(ArmMotor, 0.1, 2, 5);
            }

            // position = servoPower;
            if (gamepad1.a) {
                position = 0.65;
                ClawServo.setPosition(position);

                // Display the current value
                telemetry.addData("Servo Position", "%5.2f", position);
                telemetry.addData(">", "Press Stop to end test.");
                telemetry.update();
            }

            if (gamepad1.b) {
                position = 0.40;
                ClawServo.setPosition(position);

                // Display the current value
                telemetry.addData("Servo Position", "%5.2f", position);
                telemetry.addData(">", "Press Stop to end test.");
                telemetry.update();
            }


            if (gamepad2.x) {
                ClawReachPosition = 0.80;
                ClawReachServo.setPosition(ClawReachPosition);
            }
            if (gamepad2.y) {
                ClawReachPosition = 0.75;
                ClawReachServo.setPosition(ClawReachPosition);
            }

            if (gamepad2.dpad_up) {
                // Keep stepping up until we hit the max value.
                // Keep stepping up until we hit the max value.
                ClawReachPosition += INCREMENT;
                if (ClawReachPosition >= CLAW_MAX_POS) {
                    ClawReachPosition = CLAW_MAX_POS;
                }
                ClawReachServo.setPosition(ClawReachPosition);
            }
            else if (gamepad2.dpad_down) {
                ClawReachPosition -= INCREMENT;
                if (ClawReachPosition <= CLAW_MIN_POS) {
                    ClawReachPosition = CLAW_MIN_POS;
                }
                ClawReachServo.setPosition(ClawReachPosition);
            }



            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", ClawReachPosition);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();
            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.y) {
                // Keep stepping up until we hit the max value.
                position = 0; //stop
                CarouselServo.setPower(position);
            }

            if (gamepad1.x) {
                // Keep stepping up until we hit the max value.
                position = -1; //rotate right
                CarouselServo.setPower(position);

            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Frontleft (%.2f), Frontright (%.2f), Backleft (%.2f), Backright (%.2f)", FrontLeftPower, FrontRightPower, BackLeftPower, BackRightPower);
            telemetry.update();
        }
    }


    public void InLineEncoderDriveArm(DcMotor ArmMotor, double speed, double armmovement, double timeoutS) {
        int newArmTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newArmTarget = ArmMotor.getCurrentPosition() + (int) (armmovement * COUNTS_PER_INCH);
            ArmMotor.setPower(Math.abs(speed));
            ArmMotor.setTargetPosition(newArmTarget);

            // Turn On RUN_TO_POSITION
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && ArmMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d ", newArmTarget);
                telemetry.addData("Path2", "Running at %7d ", ArmMotor.getCurrentPosition());

                telemetry.update();
            }

            sleep(250);   // optional pause after each move

        }
    }
}


