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

import static org.firstinspires.ftc.teamcode.HardwarePushbot_TC.*;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import android.provider.Telephony;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DuckPosDetermination.*;
import org.firstinspires.ftc.teamcode.HardwarePushbot_TC;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


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

@Autonomous(name="Pushbot: RedLeftAuto_Warehouse", group="FreightFrenzy")
@Disabled
public class AutoRedLeft_Warehouse extends LinearOpMode {
   // private DistanceSensor sensorRange;

    /* Declare OpMode members. */
    HardwarePushbot_TC robot   = new HardwarePushbot_TC();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    double ArmMovement;
    double ArmMovementTimeout;
    double CarouselPosition;

    OpenCvWebcam webcam;
    InLineDuckPosDeterminationPipeline pipeline;
   // EncoderDrive encoderDrive = new EncoderDrive();
    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new InLineDuckPosDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
       // sensorRange = hardwareMap.get(DistanceSensor .class, "DistanceSensor");
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Status", "Error: %7d", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Analysis", pipeline.getAnalysis());
       // telemetry.update();
        telemetry.addData("Running Program", "Red Left");
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders and Setting up webcam pipeline");    //
        telemetry.update();

       /* robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
    /*    telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition());
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.update();*/

        robot.ClawServo.setPosition(CLAW_CLOSE_POS);
        sleep(500);
        robot.ClawReachServo.setPosition(CLAWREACH_PULLIN_P0S);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        //so stop() actually works
        if(isStopRequested()){
            return;
        }

        //  pipeline = new DuckPositionDetermination();
        while (opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();
            break;

        }

        InLineDuckPosDeterminationPipeline.DuckPositionInLine position1;
        {
            position1 = InLineDuckPosDeterminationPipeline.DuckPositionInLine.LEFT;
            position1 =pipeline.getAnalysis();
        }
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // FORWARD DRIVE SAMPLE. reverse drive will be all negative values
        robot.ArmMotor.setDirection(DcMotor.Direction.REVERSE);


        //straffe towards the inside of the field before moving to the carousel
        encoderDriveInLine(0.2,-5,5,5,-5,2);
        //position arm for delivery based on duck position
      //  encoderDrive.encoderDrive(robot,0.2,-5,5,5,-5,2);
        //Drive backward to the carousel
        encoderDriveInLine(0.4,-19,-19,-19,-19,9);
/*        while(sensorRange.getDistance(DistanceUnit.INCH) >= distance){
            drive(0.4);
        }*/

        ElapsedTime carouselTimer = new ElapsedTime();
        carouselTimer.reset();
        carouselTimer.startTime();

        while(carouselTimer.seconds() <= 4) {
            CarouselPosition=-1;
            robot.CarouselServo.setPower(CarouselPosition);
        }

        sleep(500);
        CarouselPosition=0;
        robot.CarouselServo.setPower(CarouselPosition);

        //straffe left towards the middle of the field to position to move towards alliance hub
        encoderDriveInLine(0.5,-10,10,10,-10,5);

        // move back a little bit more
        encoderDriveInLine(0.5,-5,-5,-5,-5,5);


        //straffe left towards the middle of the field to position to move towards alliance hub
        encoderDriveInLine(0.5,-30,30,30,-30,5);

        sleep(200);

        robot.ClawReachServo.setPosition(CLAWREACH_PICK_POS);
        sleep(200);
        if (position1 == InLineDuckPosDeterminationPipeline.DuckPositionInLine.LEFT) {
            ArmMovement = ARMMOVEMENT_LOW;
            ArmMovementTimeout = 5;
        } else if (position1 == InLineDuckPosDeterminationPipeline.DuckPositionInLine.CENTER) {
            ArmMovement = ARMMOVEMENT_MID;
            ArmMovementTimeout = 7;
            //move towards the alliance hub
        } else if (position1 == InLineDuckPosDeterminationPipeline.DuckPositionInLine.RIGHT) {
            ArmMovement = ARMMOVEMENT_HIGH;
            ArmMovementTimeout = 9;
        }

        //raise the arm according the duck position
        encoderDriveArmInLine(robot.ArmMotor, 0.1, -ArmMovement, ArmMovementTimeout);
        sleep(200);

        //move towards the alliance hub
        encoderDriveInLine(0.5,24,24,24,24,5);

        //baesd on the level adjust any driving forward movement
        if (position1 == InLineDuckPosDeterminationPipeline.DuckPositionInLine.LEFT) {
            encoderDriveInLine(0.1,3.2,3.2,3.2,3.2,5);
            //robot.ClawReachServo.setPosition(CLAWREACH_PICK_POS);
        } else if (position1 == InLineDuckPosDeterminationPipeline.DuckPositionInLine.CENTER) {
            encoderDriveInLine(0.2,3.5,3.5,3.5,3,5);
            //robot.ClawReachServo.setPosition(CLAWREACH_MAX_POS);
            //move towards the alliance hub
        } else if (position1 == InLineDuckPosDeterminationPipeline.DuckPositionInLine.RIGHT) {
            encoderDriveInLine(0.2,7,7,7,7,5);
            robot.ClawReachServo.setPosition(CLAWREACH_MAX_POS);
        }


        //open out the claw to open position
      //  ClawReachPosition = 0.90;
       // robot.ClawReachServo.setPosition(CLAWREACH_MAX_POS);
        sleep(200);

        //open the claw up so that the frieght drops on the alliance hub

        robot.ClawServo.setPosition(CLAW_OPEN_POS);

        sleep(500);
        //close the claw and pull back the claw reach servo

        //move back towards the storage unit
        encoderDriveInLine(0.5,-30,-30,-30,-30,5);


        robot.ClawServo.setPosition(CLAW_CLOSE_POS);
        sleep(500);

        robot.ClawReachServo.setPosition(CLAWREACH_PULLIN_P0S);

        //robot.ClawServo.setPosition(CLAW_CLOSE_POS);

        //strafe right towards the inside of the storage unit
       // encoderDriveInLine(0.2,12,-12,-12,12,2);
        //strafe right towards the inside of the storage unit
        encoderDriveInLine(0.2,58,-58,-58,58,2);

/*
        //based on the level adjust any driving backword movement
        if (position1 == InLineDuckPosDeterminationPipeline.DuckPositionInLine.LEFT) {
            encoderDriveInLine(0.1,-1,-1,-1,-1,5);
        } else if (position1 == InLineDuckPosDeterminationPipeline.DuckPositionInLine.CENTER) {
            encoderDriveInLine(0.2,-2,-2,-2,-2,5);
            //move towards the alliance hub
        } else if (position1 == InLineDuckPosDeterminationPipeline.DuckPositionInLine.RIGHT) {
            encoderDriveInLine(0.2,-4,-4,-4,-4,5);
        }
*/
        encoderDriveInLine(0.6,120,120,120,120,5);
        encoderDriveArmInLine(robot.ArmMotor, 0.1, ArmMovement, 5);

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
    public void encoderDriveInLine(double speed,
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
            sleep(250);   // optional pause after each move
        }
    }

    public void drive(double speed){
        robot.frontLeft.setPower(-speed);
        robot.frontRight.setPower(-speed);
        robot.backLeft.setPower(-speed);
        robot.backRight.setPower(-speed);
    }
    public void encoderDriveArmInLine(DcMotor ArmMotor, double speed, double armmovement, double timeoutS) {
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

    public static class InLineDuckPosDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the freight position
         */
        public Telemetry telemetry;
        public enum DuckPositionInLine
        {
            LEFT,
            CENTER,
            RIGHT,
            NODUCK
        }

        /*
         * Some color constants
         */
        final Scalar BLUE = new Scalar(0, 0, 255);
        final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         coordinates for c930E camera*/
        final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(35,30);
        final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(130,30);
        final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(235,30);
        final int REGION_WIDTH = 50;
        final int REGION_HEIGHT = 50;

        /* coordinates for c270 camera
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(50,55);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(150,55);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(250,55);
        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 50;
        */
        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;


        // Volatile since accessed by OpMode thread w/o synchronization
        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile InLineDuckPosDeterminationPipeline.DuckPositionInLine position;

        {
            position = InLineDuckPosDeterminationPipeline.DuckPositionInLine.LEFT;
        }

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];



            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the min of the 3 averages
             */
            int minOneTwo = Math.min(avg1, avg2);
            int min = Math.min(minOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(min == avg1) // Was it from region 1?
            {
                position = InLineDuckPosDeterminationPipeline.DuckPositionInLine.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(min == avg2) // Was it from region 2?
            {
                position = InLineDuckPosDeterminationPipeline.DuckPositionInLine.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(min == avg3) // Was it from region 3?
            {
                position = InLineDuckPosDeterminationPipeline.DuckPositionInLine.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(min < avg1) // Was it from region 3?
            {
                position = InLineDuckPosDeterminationPipeline.DuckPositionInLine.NODUCK; // Record our analysis

            }


            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public InLineDuckPosDeterminationPipeline.DuckPositionInLine getAnalysis()
        {
            return position;
        }
    }
}
