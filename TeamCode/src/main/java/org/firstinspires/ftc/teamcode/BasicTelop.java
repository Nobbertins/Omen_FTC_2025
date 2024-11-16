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
    private DcMotor hslidesMotor = null;
    private DcMotor vslidesMotor = null;
    private DcMotor pivotMotor = null;

    private CRServo transfer = null;
    private CRServo hand1 = null;

    private CRServo hand2 = null;
    private Servo intakeDropM = null;
    private Servo hlockM = null;
    private Servo larmM = null;
    private Servo rarmM = null;
    private Servo elbowM = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        intakeMotor  = hardwareMap.get(DcMotor.class, "intake");
        hslidesMotor  = hardwareMap.get(DcMotor.class, "hslides");
        vslidesMotor = hardwareMap.get(DcMotor.class, "vslides");
        pivotMotor = hardwareMap.get(DcMotor.class, "pivot");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        hand1 = hardwareMap.get(CRServo.class, "hand1");
        hand2 = hardwareMap.get(CRServo.class, "hand2");
        intakeDropM = hardwareMap.get(Servo.class, "intakeDrop");
        hlockM = hardwareMap.get(Servo.class, "hlock");
        larmM = hardwareMap.get(Servo.class, "larm");
        rarmM = hardwareMap.get(Servo.class, "rarm");
        elbowM = hardwareMap.get(Servo.class, "elbow");

        //initialize motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        vslidesMotor.setDirection(DcMotor.Direction.FORWARD);
        hslidesMotor.setDirection(DcMotor.Direction.FORWARD);
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        hand1.setDirection(DcMotorSimple.Direction.FORWARD);
        hand2.setDirection(DcMotorSimple.Direction.REVERSE);

        //initialize toggle servos (servos that go between angles at the press of a button)
        ToggleServo intakeDrop = new ToggleServo(intakeDropM, new int[]{0, 40}, Servo.Direction.FORWARD);
        ToggleServo hlock = new ToggleServo(hlockM, new int[]{0, 40}, Servo.Direction.REVERSE);
        ToggleServo larm = new ToggleServo(larmM, new int[]{340, 225}, Servo.Direction.REVERSE);
        ToggleServo rarm = new ToggleServo(rarmM, new int[]{340, 225}, Servo.Direction.FORWARD);
        ToggleServo elbow = new ToggleServo(elbowM, new int[]{270, 75}, Servo.Direction.FORWARD);

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
        double vslidesPower = 0;
        double hslidesPower = 0;
        double handPower = 0;
        int intakeDirection = 1;
        boolean intakeOn = false;
        boolean intakeSuckOn = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
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

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //vertical slides control
            if(gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) vslidesPower = 0;
            else if(gamepad2.left_trigger > 0) vslidesPower = gamepad2.left_trigger;
            else if(gamepad2.right_trigger > 0) vslidesPower = -gamepad2.right_trigger;
            else vslidesPower = 0;

            //horizontal slides control
            if(gamepad1.left_trigger > 0.01 && gamepad1.right_trigger > 0.01) hslidesPower = 0;
            else if(gamepad1.left_trigger > 0.01) hslidesPower = gamepad1.left_trigger;
            else if(gamepad1.right_trigger > 0.01) hslidesPower = -gamepad1.right_trigger;
            else hslidesPower = 0;

            //slides power
            hslidesMotor.setPower(hslidesPower);
            vslidesMotor.setPower(vslidesPower);

            //pivot motor
            double pivotSpeed = 0.5;
            if(gamepad2.dpad_up) pivotMotor.setPower(pivotSpeed);
            if(gamepad2.dpad_down) pivotMotor.setPower(-pivotSpeed);
            if(!gamepad2.dpad_down && !gamepad2.dpad_up) pivotMotor.setPower(0);

            //motor intake
            double intakeSpeed = 1.0;
            if(gamepad1.left_bumper && !lb1Pressed) intakeDirection *= -1;
            if(gamepad1.right_bumper && !rb1Pressed) intakeOn = !intakeOn;

            if(intakeOn){
                intakeMotor.setPower(intakeDirection > 0 ? intakeDirection * intakeSpeed : intakeDirection * 0.6);
                transfer.setPower(1.0);
            }
            else{
                intakeMotor.setPower(0);
                transfer.setPower(0);
            }

            //toggle servos inputs
            if(gamepad1.a && !a1Pressed) intakeDrop.toggle();
            if(gamepad1.x && !x1Pressed) hlock.toggle();
            if(gamepad2.y && !y2Pressed){
                larm.toggle();
                rarm.toggle();
            }
            if(gamepad2.b && !b2Pressed) elbow.toggle();
            if(gamepad2.x && !x2Pressed) {
                if(handPower != 1.0) handPower = 1.0;
                else handPower = 0.0;
            }
            if(gamepad2.a && !a2Pressed){
                if(handPower != -1.0) handPower = -1.0;
                else handPower = 0.0;
            }
            hand1.setPower(handPower);
            hand2.setPower(handPower);
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
            telemetry.update();
        }
    }}
