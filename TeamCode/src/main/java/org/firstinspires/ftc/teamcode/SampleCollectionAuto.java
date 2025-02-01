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
    private static final Pose2d STARTING_POSE = new Pose2d(-64, 0, Math.toRadians(0));
    // Sample positions (adjust these based on your field measurements)
    private static final Vector2d SPECIMEN_DROP = new Vector2d(-46.2, 0);
    private static final Vector2d SPECIMEN_DROP_PRE = new Vector2d(-48.3, 0);
    private static final Vector2d afterSPECIMEN_DROP = new Vector2d(-51, -26);
    private static final Vector2d SAMPLE_1_PRE = new Vector2d(-34, -34);
    private static final Vector2d SAMPLE_1= new Vector2d(-15, -39);
    private static final Vector2d SAMPLE_2_PRE = new Vector2d(-15, -44);
    private static final Vector2d SAMPLE_2 = new Vector2d(-15, -54);
    private static final Vector2d SAMPLE_3_PRE = new Vector2d(-15, -51);
    private static final Vector2d SAMPLE_3 = new Vector2d(-15, -62);
    private static final Vector2d HUMAN_ZONE_1 = new Vector2d(-53, -45);
    private static final Vector2d HUMAN_ZONE_2 = new Vector2d(-53, -52);
    private static final Vector2d HUMAN_ZONE_3 = new Vector2d(-50, -62);

    private static final Vector2d GRAB_POS = new Vector2d(-59, -42);
    private static final Vector2d GRAB_POS_POST = new Vector2d(-63.5, -42);
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
                .splineToConstantHeading(new Vector2d(-55, -1), Math.toRadians(60));

        TrajectoryActionBuilder specPosePostPush1Post = specPosePostPush1.fresh()
                .strafeTo(new Vector2d(-50.3 , -1));

        TrajectoryActionBuilder smallWait = drive.actionBuilder(STARTING_POSE)
                .waitSeconds(0.25);
        TrajectoryActionBuilder grabPosTwo = specPosePostPush1Post.fresh()
                .splineToConstantHeading(GRAB_POS, Math.toRadians(0))
                .strafeToConstantHeading(GRAB_POS_POST);
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
        Claw claw = new Claw(hardwareMap);
        //claw.toggle
        Pivot pivot = new Pivot(hardwareMap);
        //pivot.raise, pivot.drop
        Arm arm = new Arm(hardwareMap);
        HLock hlock = new HLock(hardwareMap);
        IntakePivot intakePiv = new IntakePivot(hardwareMap);

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
                        smallWaitTraj,
                        intakePiv.drop(),
                        intakePiv.drop(),
                        smallWaitTraj,
                        smallWaitTraj,
                        arm.rightToggle(),
                        arm.rightToggle(),
                        pivot.drop(),
                        startTraj,
                        new ParallelAction(
                        specPoseTraj,
                        vslides.raise()),
                        waitTraj,
                        smallWaitTraj,
                        smallWaitTraj,
                        vslides.stop(),
                        smallWaitTraj,
                        smallWaitTraj,
                        claw.toggle(),
                        vslides.drop(),
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
                        claw.toggle(),
                        waitTraj,
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
                        waitTraj,
                        smallWaitTraj,
                        smallWaitTraj,
                        vslides.stop(),
                        waitTraj,
                        claw.toggle(),
                        vslides.drop(),
                        grabPosTwo.build(),
                        specPosePostPush1Traj,
                        new ParallelAction(
                                specPosePostPush1PostTraj,
                                vslides.raise()
                        ),
                        waitTraj,
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