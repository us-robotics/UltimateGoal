package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public abstract class MainAuto extends LinearOpMode
{
	public abstract Vector2d getTargetCenter();

	public abstract Pose2d getEndPose();

	@Override
	public void runOpMode()
	{
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

		Pose2d startPose = new Pose2d(-63d, 53d, 0d);
		Pose2d endPose = getEndPose();
		Vector2d center = getTargetCenter();

		drive.setPoseEstimate(startPose);

		Trajectory trajectory0 = drive.trajectoryBuilder(startPose)
				.splineTo(new Vector2d(0d, -8d).plus(center), Math.toRadians(0d)).build();

		Trajectory trajectory1 = drive.trajectoryBuilder(trajectory0.end(), true)
				.splineToSplineHeading(new Pose2d(-36d, 34d, Math.toRadians(-45d)), Math.toRadians(-135d)).build();

		Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
				.splineToSplineHeading(new Pose2d(new Vector2d(-4d, -16d).plus(center), Math.toRadians(180d)), Math.toRadians(0d)).build();

		Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
				.splineTo(new Vector2d(-10d, 24d), Math.toRadians(180d)).build();

		Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
				.strafeTo(new Vector2d(-10d, 14d)).build();

		Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
				.strafeTo(new Vector2d(-10d, 4d)).build();

		Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
				.splineTo(endPose.vec(), endPose.getHeading()).build();

		waitForStart();

		if (isStopRequested()) return;

		int delay = 300;

		drive.followTrajectory(trajectory0);

		sleep(delay);

		drive.followTrajectory(trajectory1);

		sleep(delay);

		drive.followTrajectory(trajectory2);

		sleep(delay);

		drive.followTrajectory(trajectory3);

		sleep(delay);

		drive.followTrajectory(trajectory4);

		sleep(delay);

		drive.followTrajectory(trajectory5);

		sleep(delay);

		drive.followTrajectory(trajectory6);
	}
}
