package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
@Autonomous(name = "Sample Collection Auto")
public class SampleCollectionAuto extends LinearOpMode {
    // Define your robot's starting position
    private static final Pose2d STARTING_POSE = new Pose2d(-64, 0, Math.toRadians(0));
    // Sample positions (adjust these based on your field measurements)
    private static final Vector2d SPECIMEN_DROP = new Vector2d(-35, 0);
    private static final Vector2d SPECIMEN_DROP_PRE = new Vector2d(-40, 0);
    private static final Vector2d afterSPECIMEN_DROP = new Vector2d(-51, -26);
    private static final Vector2d SAMPLE_1_PRE = new Vector2d(-34, -36);
    private static final Vector2d SAMPLE_1= new Vector2d(-15, -45);
    private static final Vector2d SAMPLE_2_PRE = new Vector2d(-15, -44);
    private static final Vector2d SAMPLE_2 = new Vector2d(-15, -52);
    private static final Vector2d SAMPLE_3_PRE = new Vector2d(-15, -51);
    private static final Vector2d SAMPLE_3 = new Vector2d(-15, -60);
    private static final Vector2d HUMAN_ZONE_1 = new Vector2d(-59, -45);
    private static final Vector2d HUMAN_ZONE_2 = new Vector2d(-59, -52);
    private static final Vector2d HUMAN_ZONE_3 = new Vector2d(-59, -60);

    private static final Vector2d GRAB_POS = new Vector2d(-60, -42);
    //pivot class with raise and drop methods
    public class Arm{
        private ToggleServo rArm = null;
        private ToggleServo lArm = null;
        private ToggleServo elbow = null;
        public Arm(HardwareMap hardwareMap){
            Servo rMotor = hardwareMap.get(Servo.class, "rarm");
            Servo lMotor = hardwareMap.get(Servo.class, "larm");
            Servo elbowMotor = hardwareMap.get(Servo.class, "elbow");
            lArm = new ToggleServo(lMotor, new int[]{20, 70, 230,220, 220}, Servo.Direction.FORWARD, 0);
            rArm = new ToggleServo(rMotor, new int[]{20, 70, 230, 220, 220}, Servo.Direction.REVERSE, 0);
            elbow = new ToggleServo(elbowMotor, new int[]{10, 130, 0, 60, 210}, Servo.Direction.REVERSE, 0);
        }
        //raise action class for raise method
        public class Right implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rArm.toggleRight();
                lArm.toggleRight();
                elbow.toggleRight();
                return false;
            }
        }
        //raise method for easy calls
        public Action rightToggle() {
            return new Right();
        }
        //drop action class for drop method
        public class Left implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rArm.toggleLeft();
                lArm.toggleLeft();
                elbow.toggleLeft();
                return false;
            }
        }
        //drop class for easy calls
        public Action leftToggle() {
            return new Left();
        }
    }
    public class Claw{
        private ToggleServo claw = null;
        public Claw(HardwareMap hardwareMap){
            Servo clawMotor = hardwareMap.get(Servo.class, "claw");
            claw = new ToggleServo(clawMotor, new int[]{0, 200}, Servo.Direction.FORWARD);
        }
        //raise action class for raise method
        public class ToggleClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.toggle();
                return false;
            }
        }
        //raise method for easy calls
        public Action toggle() {
            return new ToggleClaw();
        }
    }
    public class Pivot{
        private ToggleServo rPivot = null;
        private ToggleServo lPivot = null;
        public Pivot(HardwareMap hardwareMap){
            Servo rMotor = hardwareMap.get(Servo.class, "rpivot");
            Servo lMotor = hardwareMap.get(Servo.class, "lpivot");
            rPivot = new ToggleServo(rMotor, new int[]{0, 50, 80}, Servo.Direction.FORWARD, 80);
            lPivot = new ToggleServo(lMotor, new int[]{0, 50, 80}, Servo.Direction.REVERSE, 80);
        }
        //raise action class for raise method
        public class Raise implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rPivot.toggleLeft();
                lPivot.toggleLeft();
                return false;
            }
        }
        //raise method for easy calls
        public Action raise() {
            return new Raise();
        }
        //drop action class for drop method
        public class Drop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rPivot.toggleRight();
                lPivot.toggleRight();
                return false;
            }
        }
        //drop class for easy calls
        public Action drop() {
            return new Drop();
        }
    }

    public class IntakePivot{
        private ToggleServo intakePivA = null;
        private ToggleServo intakePivB = null;
        public IntakePivot(HardwareMap hardwareMap){
            Servo intakePivAM = hardwareMap.get(Servo.class, "intakePivA");
            Servo intakePivBM = hardwareMap.get(Servo.class, "intakePivB");
            intakePivA = new ToggleServo(intakePivAM, new int[]{15, 25, 130, 215}, Servo.Direction.FORWARD, 25);
            intakePivB = new ToggleServo(intakePivBM, new int[]{15, 25, 130, 215}, Servo.Direction.REVERSE, 25);
        }
        //raise action class for raise method
        public class Raise implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePivA.toggleLeft();
                intakePivB.toggleLeft();
                return false;
            }
        }
        //raise method for easy calls
        public Action raise() {
            return new Raise();
        }
        //drop action class for drop method
        public class Drop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePivA.toggleRight();
                intakePivB.toggleRight();
                return false;
            }
        }
        //drop class for easy calls
        public Action drop() {
            return new Drop();
        }
    }
    //vertical slides class with raise and drop methods
    public class VerticalSlides{
        private DcMotor rightSlides = null;
        private DcMotor leftSlides = null;
        public VerticalSlides(HardwareMap hardwareMap){
            leftSlides  = hardwareMap.get(DcMotor.class, "lslides");
            rightSlides  = hardwareMap.get(DcMotor.class, "rslides");
        }
        //raise action class for raise method
        public class Raise implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double liftPower = 0.9;
                leftSlides.setPower(liftPower);
                rightSlides.setPower(liftPower);
                sleep(800);
                leftSlides.setPower(0);
                rightSlides.setPower(0);
                return false;
            }
        }
        //raise method for easy calls
        public Action raise() {
            return new Raise();
        }
        //drop action class for drop method
        public class Drop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double dropPower = -0.7;
                leftSlides.setPower(dropPower);
                rightSlides.setPower(dropPower);
                sleep(600);
                leftSlides.setPower(0);
                rightSlides.setPower(0);
                return false;
            }
        }
        //drop class for easy calls
        public Action drop() {
            return new Drop();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, STARTING_POSE);

        // TODO: Initialize your intake system
        // TODO: Initialize your lift system

        TrajectoryActionBuilder driveToSpecimenDrop = drive.actionBuilder(STARTING_POSE)
                .strafeTo(SPECIMEN_DROP_PRE);
        TrajectoryActionBuilder WAIT = drive.actionBuilder(STARTING_POSE)
                .waitSeconds(3);

        TrajectoryActionBuilder moveBack= drive.actionBuilder(new Pose2d(SPECIMEN_DROP.x, SPECIMEN_DROP.y, STARTING_POSE.heading.toDouble()))
                .strafeTo(new Vector2d(-45, 0));
        TrajectoryActionBuilder specPose = drive.actionBuilder(new Pose2d(SPECIMEN_DROP_PRE.x, SPECIMEN_DROP_PRE.y, Math.toRadians(180)))
                .strafeTo(SPECIMEN_DROP);
        TrajectoryActionBuilder smallWait = drive.actionBuilder(STARTING_POSE)
                .waitSeconds(0.5);
        // Build the entire action sequence
        TrajectoryActionBuilder pushSequence = drive.actionBuilder(new Pose2d(SPECIMEN_DROP.x, SPECIMEN_DROP.y, STARTING_POSE.heading.toDouble()))
                // Move to specimen drop

                .strafeToLinearHeading(afterSPECIMEN_DROP, Math.toRadians(0))
                .splineToConstantHeading(SAMPLE_1_PRE, Math.toRadians(0))
                // Move to first sample
                .splineToConstantHeading(SAMPLE_1, Math.toRadians(0))

                // Move to bucket
                .strafeTo(HUMAN_ZONE_1)
                // Move to second sample
                .strafeToConstantHeading(SAMPLE_2_PRE)
                .splineToConstantHeading(SAMPLE_2, Math.toRadians(0)) // Time for intake

                .strafeTo(HUMAN_ZONE_2)
                // Back to bucket

                .strafeToConstantHeading(SAMPLE_3_PRE)
                .splineToConstantHeading(SAMPLE_3, Math.toRadians(0))
                .strafeTo(HUMAN_ZONE_3)
                .splineToConstantHeading(GRAB_POS, Math.toRadians(0))
                .waitSeconds(5);
        VerticalSlides vslides = new VerticalSlides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        //claw.toggle
        Pivot pivot = new Pivot(hardwareMap);
        //pivot.raise, pivot.drop
        Arm arm = new Arm(hardwareMap);
        IntakePivot intakePiv = new IntakePivot(hardwareMap);

        Actions.runBlocking(
                new SequentialAction(
                        pivot.drop(),
                        pivot.drop(),
                        pivot.drop(),
                        claw.toggle()
                )
        );
        waitForStart();

        if (isStopRequested()) return;
        Action startTraj = driveToSpecimenDrop.build();
        Action pushTraj = pushSequence.build();

        Action waitTraj = WAIT.build();
        Action smallWaitTraj = smallWait.build();
        Action specPoseTraj = specPose.build();
        Action moveBackTraj = moveBack.build();
        //arm.rightToggle, arm.leftToggle
        // TODO: Set lift to specimen height
        //Actions.runBlocking(autoSequence);
        Actions.runBlocking(
                new SequentialAction(
                        pivot.raise(),
                        smallWaitTraj,
                        intakePiv.drop(),
                        intakePiv.drop(),
                        smallWaitTraj,
                        arm.rightToggle(),
                        arm.rightToggle(),
                        pivot.drop(),
                        startTraj,
                        vslides.raise(),
                        specPoseTraj,
                        arm.rightToggle(),
                        vslides.drop(),
                        waitTraj,
                        pivot.drop(),
                        waitTraj,
                        claw.toggle(),
                        waitTraj,
                        moveBackTraj,
                        arm.leftToggle(),
                        arm.leftToggle()

//                        startTraj,
//                        vslides.raise(),
//                        pivot.raise(),
//                        vslides.drop(),
//                        pushTraj
                )
        );
        // Throughout the sequence, you'll need to add your intake/lift controls
        // at the appropriate times
    }

}