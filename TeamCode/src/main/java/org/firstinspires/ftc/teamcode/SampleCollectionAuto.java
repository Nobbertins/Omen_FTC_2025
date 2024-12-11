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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Sample Collection Auto")
public class SampleCollectionAuto extends LinearOpMode {
    // Define your robot's starting position
    private static final Pose2d STARTING_POSE = new Pose2d(8, -64, Math.toRadians(180));

    // Sample positions (adjust these based on your field measurements)
    private static final Vector2d SPECIMEN_DROP = new Vector2d(36, -64);
    private static final Vector2d afterSPECIMEN_DROP = new Vector2d(30, -50);
    private static final Vector2d SAMPLE_1 = new Vector2d(44.5, -44);
    private static final Vector2d SAMPLE_2 = new Vector2d(44.5, -14);
    private static final Vector2d SAMPLE_3 = new Vector2d(44.5, -3);
    private static final Vector2d BUCKET_POS = new Vector2d(10, -12);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, STARTING_POSE);

        // TODO: Initialize your intake system
        // TODO: Initialize your lift system

        // Build the entire action sequence
        TrajectoryActionBuilder autoSequence = drive.actionBuilder(STARTING_POSE)
                // Move to specimen drop
                .strafeTo(SPECIMEN_DROP)
                .waitSeconds(5)
                .strafeTo(afterSPECIMEN_DROP)
                .waitSeconds(5)
                // Move to first sample
                .strafeTo(SAMPLE_1)
                .waitSeconds(5)  // Time for intake

                // Move to bucket
                .strafeTo(BUCKET_POS)
                .waitSeconds(5)  // Time for outtake

                // Move to second sample
                .strafeTo(SAMPLE_2)
                .waitSeconds(5)  // Time for intake

                // Back to bucket
                .strafeTo(BUCKET_POS)
                .waitSeconds(0.5)  // Time for outtake

                // Move to third sample
                .strafeTo(SAMPLE_3)
                .waitSeconds(0.5)  // Time for intake

                // Back to bucket one last time
                .strafeTo(BUCKET_POS)
                .waitSeconds(0.5)  // Time for outtake

                // Park
                .strafeTo(BUCKET_POS);

        waitForStart();

        if (isStopRequested()) return;
        Action desiredTraj = autoSequence.build();
        // TODO: Set lift to specimen height
        //Actions.runBlocking(autoSequence);
        Actions.runBlocking(
                new SequentialAction(
                        desiredTraj
                )
        );
        // Throughout the sequence, you'll need to add your intake/lift controls
        // at the appropriate times
    }
}