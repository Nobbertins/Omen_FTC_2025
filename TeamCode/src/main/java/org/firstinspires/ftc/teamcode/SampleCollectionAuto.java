package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.GraphUtils;

@Config
@Autonomous(name = "Sample Collection Auto")
public class SampleCollectionAuto extends LinearOpMode {
    //start timer
    private ElapsedTime runtime = new ElapsedTime();
    // Define your robot's starting position
    private static final Pose2d STARTING_POSE = new Pose2d(-64, -5, Math.toRadians(0));
    // Sample positions (adjust these based on your field measurements)
    private static final Vector2d SPECIMEN_DROP = new Vector2d(-46.2, 0);
    private static final Vector2d SPECIMEN_DROP_PRE = new Vector2d(-48.3, 0);
    private static final Vector2d afterSPECIMEN_DROP = new Vector2d(-51, -26);
    private static final Vector2d SAMPLE_1_PRE = new Vector2d(-33, -34);
    private static final Vector2d SAMPLE_1= new Vector2d(-17, -42);
    private static final Vector2d SAMPLE_2_PRE = new Vector2d(-18, -53);
    private static final Vector2d SAMPLE_2 = new Vector2d(-18, -57);
    private static final Vector2d SAMPLE_3_PRE = new Vector2d(-15, -58);
    private static final Vector2d SAMPLE_3 = new Vector2d(-15, -62.5);
    private static final Vector2d HUMAN_ZONE_1 = new Vector2d(-49, -45);
    private static final Vector2d HUMAN_ZONE_2 = new Vector2d(-49, -52);
    private static final Vector2d HUMAN_ZONE_3 = new Vector2d(-49, -61);

    private static final Vector2d GRAB_POS = new Vector2d(-59, -42);
    private static final Vector2d GRAB_POS_POST = new Vector2d(-63, -42);
    //pivot class with raise and drop methods
    public class Arm{
        private ToggleServo rArm = null;
        private ToggleServo lArm = null;
        private ToggleServo elbow = null;
        public Arm(HardwareMap hardwareMap){
            Servo rMotor = hardwareMap.get(Servo.class, "rarm");
            Servo lMotor = hardwareMap.get(Servo.class, "larm");
            Servo elbowMotor = hardwareMap.get(Servo.class, "elbow");
            lArm = new ToggleServo(lMotor, new int[]{20, 65, 230}, Servo.Direction.FORWARD, 20);
            rArm = new ToggleServo(rMotor, new int[]{20, 65, 230}, Servo.Direction.REVERSE, 20);
            elbow = new ToggleServo(elbowMotor, new int[]{10, 135, 0}, Servo.Direction.REVERSE, 10);
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
            claw = new ToggleServo(clawMotor, new int[]{2, 200}, Servo.Direction.REVERSE, 2);
        }
        //raise action class for raise method
        public class ToggleClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.toggle();
                sleep(500);
                return false;
            }
        }
        //raise method for easy calls
        public Action toggle() {
            return new ToggleClaw();
        }
    }

    public class Transfer{
        private ToggleServo transfer = null;
        public Transfer(HardwareMap hardwareMap){
            Servo transferM = hardwareMap.get(Servo.class, "claw");
            ToggleServo transfer = new ToggleServo(transferM, new int[]{0, 90}, Servo.Direction.FORWARD, 2);
        }
        //raise action class for raise method
        public class Toggle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                transfer.toggle();
                return false;
            }
        }
        //raise method for easy calls
        public Action toggle() {
            return new Toggle();
        }
    }

    public class HLock{
        private ToggleServo hlock = null;
        public HLock(HardwareMap hardwareMap){
            Servo hlockMotor = hardwareMap.get(Servo.class, "hlock");
            hlock = new ToggleServo(hlockMotor, new int[]{120, 40}, Servo.Direction.REVERSE, 120);
        }
        //raise action class for raise method
        public class Toggle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hlock.toggle();
                return false;
            }
        }
        //raise method for easy calls
        public Action toggle() {
            return new Toggle();
        }
    }
    public class Pivot{
        private ToggleServo rPivot = null;
        private ToggleServo lPivot = null;
        public Pivot(HardwareMap hardwareMap){
            Servo rMotor = hardwareMap.get(Servo.class, "rpivot");
            Servo lMotor = hardwareMap.get(Servo.class, "lpivot");
            rPivot = new ToggleServo(rMotor, new int[]{0, 30, 80}, Servo.Direction.FORWARD, 80);
            lPivot = new ToggleServo(lMotor, new int[]{0, 30, 80}, Servo.Direction.REVERSE, 80);
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
            intakePivA = new ToggleServo(intakePivAM, new int[]{15, 130, 215}, Servo.Direction.FORWARD, 25);
            intakePivB = new ToggleServo(intakePivBM, new int[]{15, 130, 215}, Servo.Direction.REVERSE, 25);
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
                return false;
            }
        }
        //raise method for easy calls
        public Action raise() {
            return new Raise();
        }
        public class Stop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftSlides.setPower(0);
                rightSlides.setPower(0);
                return false;
            }
        }
        //raise method for easy calls
        public Action stop() {
            return new Stop();
        }
        //drop action class for drop method
        public class Drop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double dropPower = -0.5;
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
    public class HorizontalSlides{
        private DcMotor hSlides = null;

        public HorizontalSlides(HardwareMap hardwareMap){
            hSlides  = hardwareMap.get(DcMotor.class, "hslides");
        }
        //raise action class for raise method
        public class Extend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double power = -0.8;
                hSlides.setPower(power);
                return false;
            }
        }
        //raise method for easy calls
        public Action extend() {
            return new Extend();
        }
        public class Stop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hSlides.setPower(0);
                return false;
            }
        }
        //raise method for easy calls
        public Action stop() {
            return new Stop();
        }
        //drop action class for drop method
        public class Retract implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double power = 0.5;
                hSlides.setPower(power);
                return false;
            }
        }
        //drop class for easy calls
        public Action retract() {
            return new Retract();
        }
    }

    public class Intake{
        private DcMotor intake = null;

        public Intake(HardwareMap hardwareMap){
            intake  = hardwareMap.get(DcMotor.class, "intake");
        }
        //raise action class for raise method
        public class In implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double power = -1.0;
                intake.setPower(power);
                return false;
            }
        }
        //raise method for easy calls
        public Action In() {
            return new In();
        }
        public class Stop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0);
                return false;
            }
        }
        //raise method for easy calls
        public Action stop() {
            return new Stop();
        }
        //drop action class for drop method
        public class Out implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double power = 0.6;
                intake.setPower(power);
                return false;
            }
        }
        //drop class for easy calls
        public Action Out() {
            return new Out();
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
                .waitSeconds(1);

        TrajectoryActionBuilder moveBack= drive.actionBuilder(new Pose2d(SPECIMEN_DROP.x, SPECIMEN_DROP.y, STARTING_POSE.heading.toDouble()))
                .strafeTo(new Vector2d(-45, 0));
        TrajectoryActionBuilder specPose = drive.actionBuilder(new Pose2d(SPECIMEN_DROP_PRE.x, SPECIMEN_DROP_PRE.y, Math.toRadians(0)))
                .strafeTo(SPECIMEN_DROP);

        TrajectoryActionBuilder specPosePostPush1 = drive.actionBuilder(new Pose2d(GRAB_POS_POST.x, GRAB_POS_POST.y, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-51.4, 10), Math.toRadians(60));

        TrajectoryActionBuilder specPosePostPush1Post = specPosePostPush1.fresh()
                .strafeTo(new Vector2d(-44, 10));

        TrajectoryActionBuilder grabPosTwo = specPosePostPush1Post.fresh()
                .splineToConstantHeading(new Vector2d(-54.3, -42), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-61.2, -42));

        TrajectoryActionBuilder specPosePostPush2 = grabPosTwo.fresh()
                .splineToConstantHeading(new Vector2d(-53.2, 15), Math.toRadians(70));

        TrajectoryActionBuilder specPosePostPush2Post = specPosePostPush2.fresh()
                .strafeTo(new Vector2d(-47.5 , 15));

        TrajectoryActionBuilder smallWait = drive.actionBuilder(STARTING_POSE)
                .waitSeconds(0.25);
        // Build the entire action sequence
        TrajectoryActionBuilder pushSequence = drive.actionBuilder(new Pose2d(SPECIMEN_DROP.x, SPECIMEN_DROP.y, STARTING_POSE.heading.toDouble()))
                // Move to specimen drop

                .splineToConstantHeading(afterSPECIMEN_DROP, Math.toRadians(0))
                .splineToConstantHeading(SAMPLE_1_PRE, Math.toRadians(0))
                // Move to first sample
                .splineToConstantHeading(SAMPLE_1, Math.toRadians(0))

                // Move to bucket
                .splineToConstantHeading(HUMAN_ZONE_1, Math.toRadians(0))
                // Move to second sample
                .splineToConstantHeading(SAMPLE_2_PRE, Math.toRadians(0))
                .splineToConstantHeading(SAMPLE_2, Math.toRadians(0)) // Time for intake

                .splineToConstantHeading(HUMAN_ZONE_2, Math.toRadians(0))
                // Back to bucket

                .splineToConstantHeading(SAMPLE_3_PRE, Math.toRadians(0))
                .splineToConstantHeading(SAMPLE_3, Math.toRadians(0))
                .splineToConstantHeading(HUMAN_ZONE_3, Math.toRadians(0))

                .strafeToConstantHeading(GRAB_POS)
                .strafeToConstantHeading(GRAB_POS_POST);

        VerticalSlides vslides = new VerticalSlides(hardwareMap);
        HorizontalSlides hslides = new HorizontalSlides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        HLock hlock = new HLock(hardwareMap);
        IntakePivot intakePiv = new IntakePivot(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);

        Actions.runBlocking(
                new SequentialAction(
                        pivot.drop(),
                        pivot.drop(),
                        pivot.drop()
                )
        );
        waitForStart();
        runtime.reset();
        if (isStopRequested()) return;
        Action startTraj = driveToSpecimenDrop.build();
        Action pushTraj = pushSequence.build();

        Action waitTraj = WAIT.build();
        Action smallWaitTraj = smallWait.build();
        Action specPoseTraj = specPose.build();
        Action specPosePostPush1Traj = specPosePostPush1.build();

        Action specPosePostPush1PostTraj = specPosePostPush1Post.build();
        Action moveBackTraj = moveBack.build();
        //arm.rightToggle, arm.leftToggle
        // TODO: Set lift to specimen height
        //Actions.runBlocking(autoSequence);
        Actions.runBlocking(
                new SequentialAction(
                        pivot.raise(),
                        smallWaitTraj,
                        new ParallelAction(
                        intakePiv.drop(),
                        intakePiv.drop()
                        ),
                        smallWaitTraj,
                        new ParallelAction(
                        arm.rightToggle(),
                        arm.rightToggle()
                        ),
                        smallWaitTraj,
                        new ParallelAction(
                        startTraj, pivot.drop()
                                ),
                        new ParallelAction(
                        specPoseTraj,
                        vslides.raise()),
                        waitTraj,
                        smallWaitTraj,
                        vslides.stop(),
                        new ParallelAction(
                        claw.toggle(),
                        vslides.drop()),
                        smallWaitTraj,
                        new ParallelAction(
                        pivot.raise(),
                        pivot.raise(),
                        moveBackTraj,
                        arm.leftToggle(),
                        arm.leftToggle()
                        ),
                        pushTraj,
                        vslides.drop(),
                        new ParallelAction(
                        claw.toggle(),
                        vslides.raise()),
                        waitTraj,
                        vslides.stop(),
                        new ParallelAction(
                        pivot.drop(),
                        pivot.drop(),
                        arm.rightToggle(),
                        arm.rightToggle()
                        ),
                        specPosePostPush1Traj,
                        new ParallelAction(
                                specPosePostPush1PostTraj,
                        vslides.raise()
                                ),
                        waitTraj,
                        vslides.stop(),
                        new ParallelAction(
                        claw.toggle(),
                        vslides.drop()),
                        new ParallelAction(
                        grabPosTwo.build(),
                                pivot.raise(),
                                pivot.raise(),
                                arm.leftToggle(),
                                arm.leftToggle()
                                ),
                        waitTraj,
                        new ParallelAction(
                                claw.toggle(),
                                vslides.raise()),
                        waitTraj,
                        vslides.stop(),
                        smallWaitTraj,
                        new ParallelAction(
                        specPosePostPush2.build(),
                                new SequentialAction(
                                arm.rightToggle(),
                                        arm.rightToggle(),
                                        pivot.drop(),
                                        pivot.drop()
                                )
                                ),
                        new ParallelAction(
                                specPosePostPush2Post.build(),
                                vslides.raise()
                        ),
                        waitTraj,
                        smallWaitTraj,
                        smallWaitTraj,
                        vslides.stop(),
                        waitTraj,
                        claw.toggle(),
                        vslides.drop()
                )
        );
        // Throughout the sequence, you'll need to add your intake/lift controls
        // at the appropriate times
    }

}