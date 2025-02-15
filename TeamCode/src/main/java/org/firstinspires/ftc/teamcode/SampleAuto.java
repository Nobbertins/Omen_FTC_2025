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
@Autonomous(name = "Sample Auto")
public class SampleAuto extends LinearOpMode {
    //start timer
    private ElapsedTime runtime = new ElapsedTime();
    // Define your robot's starting position
    private static final Pose2d STARTING_POSE = new Pose2d(-64, 32, Math.toRadians(0));
    // Sample positions (adjust these based on your field measurements)
    private static final Vector2d SPECIMEN_DROP = new Vector2d(-46.2, 0);
    private static final Vector2d SPECIMEN_DROP_PRE = new Vector2d(-48.3, 0);
    private static final Vector2d SAMPLE_1= new Vector2d(-54.2, -42);
    private static final Vector2d SAMPLE_2 = new Vector2d(-50.7, -45);
    private static final Vector2d SAMPLE_3 = new Vector2d(-47.2, -45);
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
            rArm = new ToggleServo(lMotor, new int[]{20, 79, 230, 220, 220}, Servo.Direction.FORWARD, 20);
            lArm = new ToggleServo(rMotor, new int[]{20, 79, 230, 220, 220}, Servo.Direction.REVERSE, 20);
            elbow = new ToggleServo(elbowMotor, new int[]{10, 131, 0, 100, 210}, Servo.Direction.REVERSE, 10);
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
            Servo transferM = hardwareMap.get(Servo.class, "transfer");
            transfer = new ToggleServo(transferM, new int[]{0, 90}, Servo.Direction.FORWARD, 2);
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
        public class LowRaise implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double liftPower = 0.3;
                leftSlides.setPower(liftPower);
                rightSlides.setPower(liftPower);
                return false;
            }
        }
        //raise method for easy calls
        public Action lowRaise() {
            return new LowRaise();
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
        public class LowDrop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double dropPower = -0.6;
                leftSlides.setPower(dropPower);
                rightSlides.setPower(dropPower);
                sleep(100);
                leftSlides.setPower(0);
                rightSlides.setPower(0);
                return false;
            }
        }
        //drop class for easy calls
        public Action lowDrop() {
            return new LowDrop();
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
                double power = -0.7;
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
                double power = 0.7;
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
        public class InHalf implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double power = -0.6;
                intake.setPower(power);
                return false;
            }
        }
        //raise method for easy calls
        public Action InHalf() {
            return new InHalf();
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

        VerticalSlides vslides = new VerticalSlides(hardwareMap);
        HorizontalSlides hslides = new HorizontalSlides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        HLock hlock = new HLock(hardwareMap);
        IntakePivot intakePiv = new IntakePivot(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        TrajectoryActionBuilder driveToSpecimenDrop = drive.actionBuilder(STARTING_POSE)
                .strafeToLinearHeading(new Vector2d(-53.7, 53.7), Math.toRadians(337))
                .afterTime(0, vslides.raise())
                .afterTime(0.2, arm.rightToggle())
                .afterTime(0.6, new SequentialAction(arm.rightToggle(), arm.rightToggle(), arm.rightToggle(), arm.rightToggle()))
                .afterTime(1.0, new SequentialAction(vslides.lowRaise(), claw.toggle()))
                .afterTime(1.2, arm.leftToggle())
                .afterTime(1.5, vslides.drop());

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
        TrajectoryActionBuilder sampleBack4 = drive.actionBuilder(new Pose2d(new Vector2d(-54, 54), Math.toRadians(17)))
                .strafeToLinearHeading(new Vector2d(-55, 55), Math.toRadians(315))
                .afterTime(0.2, arm.leftToggle())
                .afterTime(0.5, vslides.drop());
        // Build the entire action sequence
        TrajectoryActionBuilder collectSequence1 = driveToSpecimenDrop.fresh()
                // Move to specimen drop
                // Move to first sample
                .strafeToLinearHeading(new Vector2d(-51.7, 53.7), Math.toRadians(338.5))
                .afterTime(0, hlock.toggle())
                .afterTime(0.05, new ParallelAction(hslides.extend(), intakePiv.drop(), intakePiv.drop(), intakePiv.drop()))
                .afterTime(0.25, new SequentialAction(arm.leftToggle(), arm.leftToggle(), hslides.stop(), intake.In()))
                .afterTime(0.6, hslides.extend())
                .afterTime(0.8, hslides.stop())
                .afterTime(1.2, intake.stop())
                .afterTime(1.3, new ParallelAction(hslides.retract(), intakePiv.raise(), intakePiv.raise(), intake.In(), intakePiv.raise()))
                .afterTime(2.0, intake.stop())
                .afterTime(2.1, new ParallelAction(hslides.stop(), hlock.toggle()))
                .afterTime(2.3, new ParallelAction(intake.InHalf(), transfer.toggle()))
                .afterTime(2.7, intake.stop())
                .afterTime(2.7, new ParallelAction(claw.toggle()));

        TrajectoryActionBuilder collectSeq2 = collectSequence1.fresh()
                .strafeToLinearHeading(new Vector2d(-54.2, 53.7), Math.toRadians(337))
                .afterTime(0.2, vslides.raise())
                .afterTime(0.4, arm.rightToggle())
                .afterTime(0.9, new SequentialAction(arm.rightToggle(), arm.rightToggle(), arm.rightToggle()))
                .afterTime(1.9, new SequentialAction(vslides.lowRaise(), claw.toggle()))
                .afterTime(2.1, arm.leftToggle())
                .afterTime(2.4, vslides.drop());

        TrajectoryActionBuilder collectSequence2 = collectSeq2.fresh()

                .strafeToLinearHeading(new Vector2d(-51, 53.7), Math.toRadians(360))
                .afterTime(0, hlock.toggle())
                .afterTime(0.05, new ParallelAction(hslides.extend(), intakePiv.drop(), intakePiv.drop(), intakePiv.drop(), transfer.toggle()))
                .afterTime(0.2, new SequentialAction(arm.leftToggle(), arm.leftToggle(), hslides.stop(), intake.In()))
                .afterTime(0.55, hslides.extend())
                .afterTime(0.75, hslides.stop())
                .afterTime(1.25, intake.stop())
                .afterTime(1.75, new ParallelAction(hslides.retract(), intakePiv.raise(), intakePiv.raise(), intake.In(), intakePiv.raise()))
                .afterTime(2.15, intake.stop())
                .afterTime(2.85, new ParallelAction(hslides.stop(), hlock.toggle()))
                .afterTime(2.95, new ParallelAction(intake.InHalf(), transfer.toggle()))
                .afterTime(3.05, intake.stop())
                .afterTime(3.05, new ParallelAction(claw.toggle()));

        TrajectoryActionBuilder sampleBack2 = collectSequence2.fresh()
                .strafeToLinearHeading(new Vector2d(-54.7, 53.7), Math.toRadians(337))
                .afterTime(0.25, vslides.raise())
                .afterTime(0.45, arm.rightToggle())
                .afterTime(0.85, new SequentialAction(arm.rightToggle(), arm.rightToggle(), arm.rightToggle()))
                .afterTime(1.75, new SequentialAction(vslides.lowRaise(), claw.toggle()))
                .afterTime(1.95, arm.leftToggle())
                .afterTime(2.25, vslides.drop());
        TrajectoryActionBuilder collectSequence3 = sampleBack2.fresh()
                // Move to specimen drop
                // Move to first sample
                .strafeToLinearHeading(new Vector2d(-46.7, 52.7), Math.toRadians(18))
                .afterTime(0, hlock.toggle())
                .afterTime(0.05, new ParallelAction(hslides.extend(), intakePiv.drop(), intakePiv.drop(), intakePiv.drop(), transfer.toggle()))
                .afterTime(0.2, new SequentialAction(arm.leftToggle(), arm.leftToggle(), hslides.stop(), intake.In()))
                .afterTime(0.55, hslides.extend())
                .afterTime(0.75, hslides.stop())
                .afterTime(1.25, intake.stop())
                .afterTime(1.75, new ParallelAction(hslides.retract(), intakePiv.raise(), intakePiv.raise(), intake.In(), intakePiv.raise()))
                .afterTime(2.45, intake.stop())
                .afterTime(3.05, new ParallelAction(hslides.stop(), hlock.toggle()))
                .afterTime(3.15, new ParallelAction(intake.InHalf(), transfer.toggle()))
                .afterTime(3.25, intake.stop())
                .afterTime(3.25, new ParallelAction( claw.toggle()));

        TrajectoryActionBuilder sampleBack3 = collectSequence3.fresh()
                .strafeToLinearHeading(new Vector2d(-53.7, 53.7), Math.toRadians(337))
                .afterTime(0.25, vslides.raise())
                .afterTime(0.45, arm.rightToggle())
                .afterTime(0.85, new SequentialAction(arm.rightToggle(), arm.rightToggle(), arm.rightToggle()))
                .afterTime(1.15, new SequentialAction(vslides.lowRaise(), claw.toggle()))
                .afterTime(1.35, arm.leftToggle())
                .afterTime(1.65, vslides.drop());
        waitForStart();
        runtime.reset();
        if (isStopRequested()) return;
        Action startTraj = driveToSpecimenDrop.build();
        Action collectTraj1 = collectSequence1.build();

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
                        new ParallelAction(
                        pivot.raise(), pivot.raise(),
                        startTraj),
                        collectSequence1.build(),
                        collectSeq2.build(),
                        collectSequence2.build(),
                        sampleBack2.build(),
                        collectSequence3.build(),
                        sampleBack3.build()
                )
        );
        // Throughout the sequence, you'll need to add your intake/lift controls
        // at the appropriate times
    }

}