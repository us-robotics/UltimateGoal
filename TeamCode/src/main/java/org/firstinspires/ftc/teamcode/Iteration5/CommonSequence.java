package org.firstinspires.ftc.teamcode.Iteration5;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;

import org.firstinspires.ftc.teamcode.Behaviors.DrivetrainI5;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
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

	protected final static Pose2d LAUNCH_POINT = new Pose2d(0d, 39.5d, Math.toRadians(180d));

//	protected Pose2d dropWobbles(Pose2d start, Vector2d center, Vector2d grabPoint)
//	{
//		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
//		WobbleGrabberI5 grabber = opMode.getBehavior(WobbleGrabberI5.class);
//		WobbleDraggerI5 dragger = opMode.getBehavior(WobbleDraggerI5.class);
//
//		SampleMecanumDrive drive = drivetrain.getDrive();
//
//		Trajectory wobbleDrop1 = drive.trajectoryBuilder(start)
//				.splineTo(new Vector2d(-3d, -7d).plus(center), Math.toRadians(0d)).build();
//
//		execute(drivetrain, new DrivetrainI5.Follow(wobbleDrop1));
//
//		wait(0.1f);
//
//		execute(dragger, new WobbleDraggerI5.Drag(false));
//		execute(grabber, new WobbleGrabberI5.Grab(false));
//
//		wait(0.5f);
//
//		Trajectory pickUpWobble = drive.trajectoryBuilder(wobbleDrop1.end(), true)
//				.splineToLinearHeading(new Pose2d(grabPoint, Math.toRadians(-45d)), Math.toRadians(135.0f)).build();
//
//		buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.GRAB));
//		buffer(drivetrain, new DrivetrainI5.Follow(pickUpWobble));
//
//		execute();
//		wait(0.1f);
//
//		execute(dragger, new WobbleDraggerI5.Drag(true));
//		execute(grabber, new WobbleGrabberI5.Grab(true));
//
//		wait(0.5f);
//
//		Trajectory wobbleDrop2 = drive.trajectoryBuilder(pickUpWobble.end())
//				.splineToSplineHeading(new Pose2d(new Vector2d(-2d, -20d).plus(center), Math.toRadians(185d)), Math.toRadians(0d)).build();
//
//		execute(drivetrain, new DrivetrainI5.Follow(wobbleDrop2));
//		execute(grabber, new WobbleGrabberI5.Grab(false));
//
//		wait(0.3f);
//
//		execute(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.HIGH));
//		execute(grabber, new WobbleGrabberI5.Grab(true));
//
//		buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.FOLD));
//
//		return wobbleDrop2.end();
//	}

	protected Pose2d powerShots(Pose2d start)
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		Launcher launcher = opMode.getBehavior(Launcher.class);

		SampleMecanumDrive drive = drivetrain.getDrive();

		execute(launcher, new Launcher.Prime(true));

		Trajectory goLaunch1 = drive.trajectoryBuilder(start)
				.lineToConstantHeading(new Vector2d(0d, 22d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(goLaunch1));
		execute(launcher, new Launcher.Lift(0));

		wait(0.5f);

		execute(launcher, new Launcher.Lift(1));

		wait(0.75f);

		execute(launcher, new Launcher.Lift(0));

		wait(1f);

		Trajectory goLaunch2 = drive.trajectoryBuilder(goLaunch1.end())
				.strafeTo(new Vector2d(0d, 15d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(goLaunch2));
		execute(launcher, new Launcher.Lift(1));

		wait(0.75f);

		execute(launcher, new Launcher.Lift(0));

		wait(1f);

		Trajectory goLaunch3 = drive.trajectoryBuilder(goLaunch2.end())
				.strafeTo(new Vector2d(0d, 9d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(goLaunch3));
		execute(launcher, new Launcher.Lift(1));

		wait(0.75f);

		execute(launcher, new Launcher.Prime(false));
		execute(launcher, new Launcher.Lift(-1));

		return goLaunch3.end();
	}

	protected Pose2d dropFirst(Pose2d start, Vector2d center)
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		Launcher launcher = opMode.getBehavior(Launcher.class);
		WobbleGrabberI5 grabber = opMode.getBehavior(WobbleGrabberI5.class);
		WobbleDraggerI5 dragger = opMode.getBehavior(WobbleDraggerI5.class);

		SampleMecanumDrive drive = drivetrain.getDrive();

		execute(launcher, new Launcher.Lift(-1));

		Trajectory wobbleDrop = drive.trajectoryBuilder(start).forward(48f)
				.splineTo(new Vector2d(-3d, -10d).plus(center), Math.toRadians(0d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(wobbleDrop));

		wait(0.1f);

		execute(dragger, new WobbleDraggerI5.Drag(false));
		execute(grabber, new WobbleGrabberI5.Grab(false));
		execute(launcher, new Launcher.Prime(Launcher.HIGH_POWER, true));

		wait(0.5f);

		Trajectory strafeRight = drive.trajectoryBuilder(wobbleDrop.end())
				.strafeRight(12d).build();

		execute(drivetrain, new DrivetrainI5.Follow(strafeRight));

		Trajectory launchHigh = drive.trajectoryBuilder(strafeRight.end())
				.lineToLinearHeading(LAUNCH_POINT).build();

		buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.FOLD));
		buffer(drivetrain, new DrivetrainI5.Follow(launchHigh));

		execute();
		launchRings();

		return launchHigh.end();
	}

	protected void launchRings()
	{
		Launcher launcher = opMode.getBehavior(Launcher.class);

		execute(launcher, new Launcher.Lift(0));
		wait(0.25f);

		for (int i = 0; i < 3; i++)
		{
			execute(launcher, new Launcher.Lift(1));
			wait(0.65f);

			if (i < 2)
			{
				execute(launcher, new Launcher.Lift(0));
				wait(0.5f);
			}
		}

		execute(launcher, new Launcher.Lift(-1));
		execute(launcher, new Launcher.Prime(false));
	}

	protected Pose2d intakeRings(Pose2d start)
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		Intake intake = opMode.getBehavior(Intake.class);
		SampleMecanumDrive drive = drivetrain.getDrive();

		execute(intake, new Intake.Run(1f));

		wait(0.3f);

		Trajectory intakeRings = drive.trajectoryBuilder(start)
				.lineToLinearHeading(new Pose2d(-4d, 36d, Math.toRadians(180d))).build();

		Trajectory driveForward = drive.trajectoryBuilder(intakeRings.end())
				.forward(40d).build();

		execute(drivetrain, new DrivetrainI5.Follow(intakeRings));
		execute(drivetrain, new DrivetrainI5.Follow(driveForward));

		execute(intake, new Intake.Run(0f));

		return driveForward.end();
	}

	protected Pose2d dropSecond(Pose2d start, Vector2d center, Vector2d grabPoint)
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		WobbleGrabberI5 grabber = opMode.getBehavior(WobbleGrabberI5.class);
		WobbleDraggerI5 dragger = opMode.getBehavior(WobbleDraggerI5.class);

		SampleMecanumDrive drive = drivetrain.getDrive();

		Trajectory wobbleAlign = drive.trajectoryBuilder(start)
				.splineToLinearHeading(new Pose2d(-24d, 36d, Math.toRadians(-45d)), Math.toRadians(-90d)).build();

		buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.GRAB));
		buffer(drivetrain, new DrivetrainI5.Follow(wobbleAlign));

		execute();

		Trajectory wobblePickup = drive.trajectoryBuilder(wobbleAlign.end())
				.lineToConstantHeading(new Vector2d(-48d, 36d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(wobblePickup));
		wait(0.1f);

		execute(dragger, new WobbleDraggerI5.Drag(true));
		execute(grabber, new WobbleGrabberI5.Grab(true));

		wait(0.5f);

		Trajectory wobbleDrop = drive.trajectoryBuilder(wobblePickup.end())
				.splineToSplineHeading(new Pose2d(new Vector2d(-2d, -18d).plus(center), Math.toRadians(180d)), Math.toRadians(0d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(wobbleDrop));
		execute(grabber, new WobbleGrabberI5.Grab(false));

		wait(0.3f);

		execute(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.DROP));
		execute(grabber, new WobbleGrabberI5.Grab(true));

		buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.FOLD));

		return wobbleDrop.end();
	}
}
