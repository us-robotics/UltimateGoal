package org.firstinspires.ftc.teamcode.Iteration5;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Behaviors.DrivetrainI5;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleDraggerI5;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabberI5;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.OpModeBase;

public abstract class CommonSequence extends JobSequence
{
	public CommonSequence(OpModeBase opMode)
	{
		super(opMode);
	}

	protected Trajectory dropWobbles(Pose2d start, Vector2d center)
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		WobbleGrabberI5 grabber = opMode.getBehavior(WobbleGrabberI5.class);
		WobbleDraggerI5 dragger = opMode.getBehavior(WobbleDraggerI5.class);

		SampleMecanumDrive drive = drivetrain.getDrive();

		Trajectory wobbleDrop1 = drive.trajectoryBuilder(start)
				.splineTo(new Vector2d(6d, -12d).plus(center), Math.toRadians(0d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(wobbleDrop1));

		execute(dragger, new WobbleDraggerI5.Drag(false));
		execute(grabber, new WobbleGrabberI5.Grab(false));

		wait(0.3f);

		Trajectory pickUpWobble = drive.trajectoryBuilder(wobbleDrop1.end(), true)
				.splineToSplineHeading(new Pose2d(-38.5d, 34.5d, Math.toRadians(-45d)), Math.toRadians(-135d)).build();

		buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.GRAB));
		buffer(drivetrain, new DrivetrainI5.Follow(pickUpWobble));

		execute();

		execute(dragger, new WobbleDraggerI5.Drag(true));
		execute(grabber, new WobbleGrabberI5.Grab(true));

		Trajectory wobbleDrop2 = drive.trajectoryBuilder(pickUpWobble.end())
				.splineToSplineHeading(new Pose2d(new Vector2d(-4d, -16d).plus(center), Math.toRadians(180d)), Math.toRadians(0d)).build();

		wait(0.3f);

		execute(drivetrain, new DrivetrainI5.Follow(wobbleDrop2));
		execute(grabber, new WobbleGrabberI5.Grab(false));

		wait(0.3f);

		execute(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.HIGH));
		execute(grabber, new WobbleGrabberI5.Grab(true));

		buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.HIGH));

		return wobbleDrop2;
	}

	protected Trajectory powerShots(Pose2d start)
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		Launcher launcher = opMode.getBehavior(Launcher.class);

		SampleMecanumDrive drive = drivetrain.getDrive();

		execute(launcher, new Launcher.Prime(Launcher.SHOT_POWER, true));
		execute(launcher, new Launcher.Lift(-1));

		Trajectory goLaunch1 = drive.trajectoryBuilder(start)
				.splineTo(new Vector2d(2d, 28d), Math.toRadians(180d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(goLaunch1));
		execute(launcher, new Launcher.Lift(1));

		wait(0.5f);

		execute(launcher, new Launcher.Lift(0));

		Trajectory goLaunch2 = drive.trajectoryBuilder(goLaunch1.end())
				.strafeTo(new Vector2d(2d, 20.5d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(goLaunch2));
		execute(launcher, new Launcher.Lift(1));

		wait(0.5f);

		execute(launcher, new Launcher.Lift(0));

		Trajectory goLaunch3 = drive.trajectoryBuilder(goLaunch2.end())
				.strafeTo(new Vector2d(2d, 13d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(goLaunch3));
		execute(launcher, new Launcher.Lift(1));

		wait(0.5f);

		execute(launcher, new Launcher.Lift(-1));

		return goLaunch3;
	}
}
