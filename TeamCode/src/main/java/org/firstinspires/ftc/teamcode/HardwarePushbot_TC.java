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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot_TC
{
    /* Public OpMode members. */
    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  backLeft  = null;
    public DcMotor  backRight  = null;


    public DcMotor ArmMotor = null;
    public DcMotor ArmReach = null;
    public CRServo CarouselServo=null;
    public Servo ClawReachServo=null;
    public Servo   ClawServo=null;
    public Servo   CapPickServo=null;
    public Servo   CapGrabClawServo=null;


    static final double     COUNTS_PER_MOTOR_REV    = 537.6;  // 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;   // 1  // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.9;
    static final double     TURN_SPEED              = 0.3;
    static final double CLAW_OPEN_POS = 0.43;
    static final double CLAW_CLOSE_POS = 0.10;
    static final double CLAWREACH_MAX_POS = 0.1;
    static final double CLAWREACH_PICK_POS = 0.25;
    static final double CLAWREACH_PULLIN_P0S = 0.75;
    static final double ARMMOVEMENT_LOW = 5.75;
    static final double ARMMOVEMENT_MID = 9.3;
    static final double ARMMOVEMENT_HIGH = 15.3;
    static final double distance = 9;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot_TC(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft  = hwMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "FrontRight");
        backLeft =  hwMap.get(DcMotorEx.class, "BackLeft");
        backRight = hwMap.get(DcMotorEx.class, "BackRight");
        ArmMotor = hwMap.get(DcMotorEx.class, "ArmMotor");
        ArmReach = hwMap.get(DcMotorEx.class, "ArmReach");
        ClawServo = hwMap.get(Servo.class, "Claw");
        ClawReachServo = hwMap.get(Servo.class, "ClawReach");
        CarouselServo = hwMap.get(CRServo.class, "Carousel");
        CapPickServo = hwMap.get(Servo.class, "CapPick");
        CapGrabClawServo= hwMap.get(Servo.class, "CapGrabClaw");


        frontLeft.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        ArmMotor.setDirection(DcMotorEx.Direction.REVERSE);
        ArmReach.setDirection(DcMotorEx.Direction.REVERSE);



        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ArmReach.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        ArmMotor.setPower(0);
        ArmReach.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmReach.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    }
}


