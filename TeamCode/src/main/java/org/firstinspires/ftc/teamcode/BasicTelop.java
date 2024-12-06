/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Telop", group="Linear OpMode")
public class BasicTelop extends LinearOpMode {

    // Declare OpMode members for each motor
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor lslidesMotor = null;
    private DcMotor rslidesMotor = null;
    private DcMotor hslidesMotor = null;

    private CRServo transfer = null;
    private Servo rpivot = null;
    private Servo lpivot = null;

    private Servo intakeDropM = null;
    private Servo hlockM = null;
    private Servo larmM = null;
    private Servo rarmM = null;
    private Servo elbowM = null;
    private Servo claw = null;
    private Servo wrist = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        intakeMotor  = hardwareMap.get(DcMotor.class, "intake");
        lslidesMotor  = hardwareMap.get(DcMotor.class, "lslides");
        rslidesMotor  = hardwareMap.get(DcMotor.class, "rslides");
        //hslidesMotor  = hardwareMap.get(DcMotor.class, "hslides");
        //vslidesMotor = hardwareMap.get(DcMotor.class, "vslides");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        rpivot = hardwareMap.get(Servo.class, "rpivot");
        lpivot = hardwareMap.get(Servo.class, "lpivot");
        intakeDropM = hardwareMap.get(Servo.class, "intakeDrop");
        hlockM = hardwareMap.get(Servo.class, "hlock");
        larmM = hardwareMap.get(Servo.class, "larm");
        rarmM = hardwareMap.get(Servo.class, "rarm");
        elbowM = hardwareMap.get(Servo.class, "elbow");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        //initialize motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        rslidesMotor.setDirection(DcMotor.Direction.FORWARD);
        lslidesMotor.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

       hslidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initialize toggle servos (servos that go between angles at the press of a button)
        ToggleServo intakeDrop = new ToggleServo(intakeDropM, new int[]{0, 65}, Servo.Direction.FORWARD);
        ToggleServo hlock = new ToggleServo(hlockM, new int[]{120, 40}, Servo.Direction.REVERSE);
        ToggleServo larm = new ToggleServo(larmM, new int[]{250, 245, 225, 120, 110}, Servo.Direction.REVERSE, 340);
        ToggleServo rarm = new ToggleServo(rarmM, new int[]{250, 245, 225, 120, 110}, Servo.Direction.FORWARD, 340);
        ToggleServo elbow = new ToggleServo(elbowM, new int[]{350, 0, 75, 65, 225}, Servo.Direction.FORWARD, 270);
        ToggleServo rpivotM = new ToggleServo(rpivot, new int[]{0, 60}, Servo.Direction.FORWARD);
        ToggleServo lpivotM = new ToggleServo(lpivot, new int[]{0, 60}, Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        //previous button states
        boolean lb1Pressed = false;
        boolean rb1Pressed = false;
        boolean b1Pressed = false;
        boolean a1Pressed = false;
        boolean x1Pressed = false;
        boolean y1Pressed = false;
        boolean down1Pressed = false;
        boolean up1Pressed = false;
        boolean right1Pressed = false;
        boolean left1Pressed = false;

        boolean lb2Pressed = false;
        boolean rb2Pressed = false;
        boolean b2Pressed = false;
        boolean a2Pressed = false;
        boolean x2Pressed = false;
        boolean y2Pressed = false;
        boolean down2Pressed = false;
        boolean up2Pressed = false;
        boolean right2Pressed = false;
        boolean left2Pressed = false;

        //other variables used in teleop
        // added rslidesPower and lslidesPower
        double rslidesPower= 0;
        double lslidesPower = 0;
        double hslidesPower = 0;
        double handPower = 0;
        int intakeDirection = -1;
        boolean intakeOn = false;
        boolean intakeSuckOn = false;
        double driveSensitivity = 1;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            if(gamepad1.left_bumper && gamepad1.right_bumper) driveSensitivity = 0.4;
            else driveSensitivity = 1;
            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * driveSensitivity);
            rightFrontDrive.setPower(rightFrontPower * driveSensitivity);
            leftBackDrive.setPower(leftBackPower * driveSensitivity);
            rightBackDrive.setPower(rightBackPower * driveSensitivity);

            //vertical slides control
            if(gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0){
                rslidesPower = 0.1;
                lslidesPower = 0.1;
            }
            else if(gamepad2.left_trigger > 0){
                rslidesPower = gamepad2.left_trigger;
                lslidesPower = gamepad2.left_trigger;
            }

            else {
                rslidesPower = 0.1;
                lslidesPower = 0.1;
            }

            //horizontal slides control
            if(gamepad1.left_trigger > 0.01 && gamepad1.right_trigger > 0.01) hslidesPower = 0;
            else if(gamepad1.left_trigger > 0.01) hslidesPower = gamepad1.left_trigger;
            else if(gamepad1.right_trigger > 0.01) hslidesPower = -gamepad1.right_trigger;
            else hslidesPower = 0;

            //slides power
            hslidesMotor.setPower(hslidesPower);

            //motor intake
            double intakeSpeed = 1.0;
            if(gamepad1.left_bumper && !lb1Pressed) intakeDirection *= -1;
            if(gamepad1.right_bumper && !rb1Pressed) intakeOn = !intakeOn;

            if(intakeOn){
                intakeMotor.setPower(intakeDirection < 0 ? intakeDirection * intakeSpeed : intakeDirection * 0.6);
                transfer.setPower(1.0);
            }
            else{
                intakeMotor.setPower(0);
                transfer.setPower(0);
            }

            //toggle servos inputs
            if(gamepad1.a && !a1Pressed) intakeDrop.toggle();
            if(gamepad1.x && !x1Pressed) hlock.toggleRight();
            if(gamepad2.dpad_left && !left2Pressed){
                larm.toggleLeft();
                rarm.toggleLeft();
                elbow.toggleLeft();
            }
            if(gamepad2.dpad_right && !right2Pressed){
                larm.toggleRight();
                rarm.toggleRight();
                elbow.toggleRight();
            }
            if(gamepad2.x && !x2Pressed) {
            }
            if(gamepad2.a && !a2Pressed){
            }

            if(gamepad2.dpad_down && !down2Pressed) {
                lpivotM.toggleRight();
                rpivotM.toggleRight();
//                lpivot.setPosition(0);
//                rpivot.setPosition(0);
            }
            if(gamepad2.dpad_up && !up2Pressed) {
                rpivotM.toggleLeft();
                lpivotM.toggleLeft();
//                lpivot.setPosition(90);
//                rpivot.setPosition(0);
            }

            //update all button states
            b1Pressed = gamepad1.b;
            a1Pressed = gamepad1.a;
            x1Pressed = gamepad1.x;
            y1Pressed = gamepad1.y;
            down1Pressed = gamepad1.dpad_down;
            up1Pressed = gamepad1.dpad_up;
            left1Pressed = gamepad1.dpad_left;
            right1Pressed = gamepad1.dpad_right;
            lb1Pressed = gamepad1.left_bumper;
            rb1Pressed = gamepad1.right_bumper;

            b2Pressed = gamepad2.b;
            a2Pressed = gamepad2.a;
            x2Pressed = gamepad2.x;
            y2Pressed = gamepad2.y;
            down2Pressed = gamepad2.dpad_down;
            up2Pressed = gamepad2.dpad_up;
            left2Pressed = gamepad2.dpad_left;
            right2Pressed = gamepad2.dpad_right;
            lb2Pressed = gamepad2.left_bumper;
            rb2Pressed = gamepad2.right_bumper;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("ArmL", larm.getServo().getPosition());
            telemetry.addData("ArmR", rarm.getServo().getPosition());
            telemetry.addData("Elbow", elbow.getServo().getPosition());
            telemetry.addData("larm", larm.pos);
            telemetry.addData("rarm", elbow.pos);
            telemetry.addData("elbownum", elbow.pos);
            telemetry.update();
        }
    }}
