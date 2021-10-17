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

import android.provider.Telephony;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwarePushbot_TC;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: AutoTest1", group="FreightFrenzy")
//@Disabled
public class AutoTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot_TC robot   = new HardwarePushbot_TC();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;  // 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;   // 1  // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 5.25 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.9;
    static final double     TURN_SPEED              = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        double armPosition=0.09;
        double gripPosition=0.95;
        double guidePosition= 0.2;

        robot.init(hardwareMap);

        //  robot.gripServo.setPosition(gripPosition);
        //    robot.armServo.setPosition(armPosition);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // FORWARD DRIVE SAMPLE. reverse drive will be all negative values

        //Drive forward to the target BOX
/*
        class Carousel implements Runnable{
            ElapsedTime runElapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            public void run(){
                runElapsedTime.reset();
                runElapsedTime.startTime();
                if(runElapsedTime.time() == 0.1){
                    robot.CarouselDrive.setPower(0);
                }

            }
        }
        Thread thread1 = new Thread(new Carousel());
        ElapsedTime runElapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        robot.CarouselDrive.setPower(0.2);
        encoderDrive(0.2, -18,-18,-18,-18,2);
        sleep(1000);
        thread1.start();
        sleep(5000);
        encoderDrive(0.7, 111,111,111,111,7);
        encoderDrive(0.2, -1, -1,-1,-1,2);
        encoderDrive(0.2, 0, 4,4,0,2);
        robot.ArmMotor.setTargetPosition(50);
        //
        */

        encoderDrive(0.4,50,50,50,50,4);





        // S1: Forward 50 Inches with 5 Sec timeout
        //following is an example of left turn
        // encoderDrive(TURN_SPEED,-10,10,-10,10,4);
        //following is an example of right turn
        //turn right since this is blue corner to drop the wobble from the back of the robot
        //encoderDrive(TURN_SPEED,-12,12,-12,12,4);

        // S2: Turn Right 12 Inches with 4 Sec timeout
        //after turning drive backwords so that the wobble can be dropped in the target
//        encoderDrive(DRIVE_SPEED,-20,-20,-20,-20,4);  // S2: Turn Right 12 Inches with 4 Sec timeout
/*
        //get the arm and gripper to drop the wobble
        armPosition = 0.2;
        gripPosition = 0.55;
        robot.armServo.setPosition(armPosition);
        sleep(2500);
        robot.gripServo.setPosition(gripPosition);
        sleep(2000);
*/
        //move forward to clear away from wobble
        //encoderDrive(DRIVE_SPEED, 12,12,12,12, 2);
/*
        //reset the arm and gripper to starting position
        armPosition = 0.09;
        gripPosition = 0.95;
        robot.armServo.setPosition(armPosition);
        robot.gripServo.setPosition(gripPosition);
*/
        //turn left towards the shooting target zone
        //  encoderDrive(TURN_SPEED, -11.5,11.5,-11.5,11.5, 5);
        //back towards out of the launch line

        //encoderDrive(DRIVE_SPEED, -10,-10,-10,-10, 5);

        //encoderDrive(TURN_SPEED,-2,2,-2,2,4);
/*
        int i = 0;
            robot.guideServo.setPosition(guidePosition);
            robot.conveyor.setPower(1);
            robot.leftShooter.setPower(1);
            robot.rightShooter.setPower(1);
            robot.intake.setPower(1);
            sleep(6000);

        robot.conveyor.setPower(0);
        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.intake.setPower(0);
        //drive to position on the launch line
  */
        //    encoderDrive(DRIVE_SPEED, 10,10,10,10, 5);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double frontleftInches, double frontrightInches,
                             double backleftInches, double backrightInches,
                             double timeoutS) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(frontleftInches * COUNTS_PER_INCH);
            newfrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(frontrightInches * COUNTS_PER_INCH);
            newbackLeftTarget = robot.backLeft.getCurrentPosition() + (int)(backleftInches * COUNTS_PER_INCH);
            newbackRightTarget = robot.backRight.getCurrentPosition() + (int)(backrightInches * COUNTS_PER_INCH);

            robot.frontLeft.setTargetPosition(newfrontLeftTarget);
            robot.frontRight.setTargetPosition(newfrontRightTarget);
            robot.backLeft.setTargetPosition(newbackLeftTarget);
            robot.backRight.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() &&
                            robot.backRight.isBusy() && robot.backLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newfrontLeftTarget,  newfrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
}