package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="trajectoriesAuto", group="Linear Opmode")
public class trajectoriesAuto extends LinearOpMode {

    public static int strafeLeft = 67;
    public static int strafeX = -71;
    public static int strafeY = -68;
    public static int splineX = -71;
    public static double splineY = 7.5;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(strafeX, strafeY))
                .strafeLeft(strafeLeft)
                .waitSeconds(0.1)
                .splineToSplineHeading(new Pose2d(-80, -60, Math.toRadians(270)), Math.toRadians(0))
                .waitSeconds(0.1)
                .splineToSplineHeading(new Pose2d(splineX, splineY, Math.toRadians(182)), Math.toRadians(0))
                .waitSeconds(0.1)
                .back(70)
                .turn(Math.toRadians(180))
                .build();

        waitForStart();
        drive.followTrajectorySequence(trajSeq);

        while (opModeIsActive()) {

        }
    }
}
