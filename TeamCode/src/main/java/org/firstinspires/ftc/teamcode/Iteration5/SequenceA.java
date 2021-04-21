package org.firstinspires.ftc.teamcode.Iteration5;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Behaviors.DrivetrainI5;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleDraggerI5;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabberI5;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.OpModeBase;

public class SequenceA extends JobSequence
{
	public SequenceA(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	protected void queueJobs()
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		WobbleGrabberI5 grabber = opMode.getBehavior(WobbleGrabberI5.class);
		WobbleDraggerI5 dragger = opMode.getBehavior(WobbleDraggerI5.class);

		SampleMecanumDrive drive = drivetrain.getDrive();

		Pose2d startPose = new Pose2d(-63d, 53d, 0d);
		Vector2d center = new Vector2d(12d, 60d);

		drive.setPoseEstimate(startPose);

		Trajectory wobbleDropFirst = drive.trajectoryBuilder(startPose)
				.splineTo(new Vector2d(6d, -12d).plus(center), Math.toRadians(0d)).build();

		Trajectory pickUpWobbleSecond = drive.trajectoryBuilder(wobbleDropFirst.end(), true)
				.splineToSplineHeading(new Pose2d(-38.5d, 34.5d, Math.toRadians(-45d)), Math.toRadians(-135d)).build();

		Trajectory wobbleDropSecond = drive.trajectoryBuilder(pickUpWobbleSecond.end())
				.splineToSplineHeading(new Pose2d(new Vector2d(-4d, -16d).plus(center), Math.toRadians(180d)), Math.toRadians(0d)).build();

		Trajectory goLaunchOne = drive.trajectoryBuilder(wobbleDropSecond.end())
				.splineTo(new Vector2d(2d, 28d), Math.toRadians(180d)).build();

		Trajectory goLaunchTwo = drive.trajectoryBuilder(goLaunchOne.end())
				.strafeTo(new Vector2d(2d, 20.5d)).build();

		Trajectory goLaunchThree = drive.trajectoryBuilder(goLaunchTwo.end())
				.strafeTo(new Vector2d(2d, 13d)).build();

		Trajectory goPark = drive.trajectoryBuilder(goLaunchThree.end(), true)
				.splineTo(new Vector2d(12d, 32d), Math.toRadians(150d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(wobbleDropFirst));
		execute(dragger, new WobbleDraggerI5.Grab(false));

		wait(0.5f);

		execute(grabber, new WobbleGrabberI5.Grab(false));

		buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.GRAB));
		buffer(drivetrain, new DrivetrainI5.Follow(pickUpWobbleSecond));

		execute();

		execute(dragger, new WobbleDraggerI5.Grab(true));
		execute(grabber, new WobbleGrabberI5.Grab(true));

		wait(0.3f);
		execute(drivetrain, new DrivetrainI5.Follow(wobbleDropSecond));
		execute(grabber, new WobbleGrabberI5.Grab(false));

		wait(0.5f);

		execute(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.HIGH));
		execute(grabber, new WobbleGrabberI5.Grab(true));

		buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.FOLD));
		buffer(drivetrain, new DrivetrainI5.Follow(goLaunchOne));

		execute();

		wait(0.3f);
		execute(drivetrain, new DrivetrainI5.Follow(goLaunchTwo));

		wait(0.3f);
		execute(drivetrain, new DrivetrainI5.Follow(goLaunchThree));

		wait(0.3f);
		execute(drivetrain, new DrivetrainI5.Follow(goPark));
	}
}
