package org.firstinspires.ftc.teamcode.Iteration5;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.firstinspires.ftc.teamcode.Behaviors.DrivetrainI5;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleDraggerI5;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabberI5;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;

import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.OpModeBase;

public abstract class CommonSequence extends JobSequence
{
	public CommonSequence(OpModeBase opMode)
	{
		super(opMode);
	}

	protected final static Pose2d LAUNCH_POINT = new Pose2d(0d, 38d, Math.toRadians(180d));

	protected Pose2d dropFirst(Pose2d start, Vector2d center)
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		Launcher launcher = opMode.getBehavior(Launcher.class);
		WobbleGrabberI5 grabber = opMode.getBehavior(WobbleGrabberI5.class);
		WobbleDraggerI5 dragger = opMode.getBehavior(WobbleDraggerI5.class);

		SampleMecanumDrive drive = drivetrain.getDrive();

		execute(launcher, new Launcher.Lift(-1));

		Trajectory wobbleDrop = drive.trajectoryBuilder(start).forward(40f)
				.splineTo(new Vector2d(-3d, -12d).plus(center), Math.toRadians(0d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(wobbleDrop));

		wait(0.5f);

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

		execute(launcher, new Launcher.Lift(1));
		wait(0.65f);
		wait(0.65f);
		wait(0.65f);

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
				.lineToLinearHeading(new Pose2d(0d, 35d, Math.toRadians(180d))).build();

//		TrajectoryVelocityConstraint velocityConstraint = new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(0.5d), new AngularVelocityConstraint(0.5d)));
//		TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(0.5d);
//
//		Trajectory driveForward = drive.trajectoryBuilder(intakeRings.end())
//				.forward(35d, velocityConstraint, accelerationConstraint).build();

		Trajectory driveForward = drive.trajectoryBuilder(intakeRings.end())
				.forward(35d).build();

		execute(drivetrain, new DrivetrainI5.Follow(intakeRings));
		execute(drivetrain, new DrivetrainI5.Follow(driveForward));

		return driveForward.end();
	}

	protected Pose2d dropSecond(Pose2d start, Vector2d center)
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		WobbleGrabberI5 grabber = opMode.getBehavior(WobbleGrabberI5.class);
		WobbleDraggerI5 dragger = opMode.getBehavior(WobbleDraggerI5.class);

		SampleMecanumDrive drive = drivetrain.getDrive();

		Trajectory wobbleAlign = drive.trajectoryBuilder(start).back(20f)
				.splineToSplineHeading(new Pose2d(-24d, 31.5d, Math.toRadians(-45d)), Math.toRadians(-90d)).build();

		buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.GRAB));
		buffer(drivetrain, new DrivetrainI5.Follow(wobbleAlign));

		execute();

		Trajectory wobblePickup = drive.trajectoryBuilder(wobbleAlign.end())
				.lineToConstantHeading(new Vector2d(-40d, 31.5d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(wobblePickup));
		wait(0.1f);

		execute(dragger, new WobbleDraggerI5.Drag(true));
		execute(grabber, new WobbleGrabberI5.Grab(true));

		wait(0.5f);

		Trajectory wobbleDrop = drive.trajectoryBuilder(wobblePickup.end())
				.splineToSplineHeading(new Pose2d(new Vector2d(-2d, -20d).plus(center), Math.toRadians(180d)), Math.toRadians(0d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(wobbleDrop));
		execute(grabber, new WobbleGrabberI5.Grab(false));

		wait(0.3f);

		execute(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.DROP));
		execute(grabber, new WobbleGrabberI5.Grab(true));

		buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.FOLD));

		return wobbleDrop.end();
	}

	protected void powerShots(Pose2d start)
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		Launcher launcher = opMode.getBehavior(Launcher.class);

		SampleMecanumDrive drive = drivetrain.getDrive();
		drive.setPoseEstimate(start);

		Trajectory aim1 = drive.trajectoryBuilder(start)
				.strafeRight(22.5d).build();

		Trajectory aim2 = drive.trajectoryBuilder(aim1.end())
				.strafeRight(7d).build();

		Trajectory aim3 = drive.trajectoryBuilder(aim2.end())
				.strafeRight(7d).build();

		execute(drivetrain, new DrivetrainI5.Follow(aim1));
		execute(launcher, new Launcher.Lift(0));

		wait(0.3f);

		execute(launcher, new Launcher.Lift(1));
		wait(0.65f);
		execute(launcher, new Launcher.Lift(0));

		execute(drivetrain, new DrivetrainI5.Follow(aim2));
		wait(0.2f);

		execute(launcher, new Launcher.Lift(1));
		wait(0.65f);
		execute(launcher, new Launcher.Lift(0));

		execute(drivetrain, new DrivetrainI5.Follow(aim3));
		wait(0.2f);

		execute(launcher, new Launcher.Lift(1));
		wait(0.65f);
		execute(launcher, new Launcher.Lift(-1));
	}
}
