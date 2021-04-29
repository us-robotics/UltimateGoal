package org.firstinspires.ftc.teamcode.Iteration5;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
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

import java.util.Arrays;

import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.OpModeBase;

public abstract class CommonSequence extends JobSequence
{
	public CommonSequence(OpModeBase opMode)
	{
		super(opMode);
	}

	protected final static Pose2d LAUNCH_POINT = new Pose2d(1d, 38d, Math.toRadians(180d));

	protected final static TrajectoryVelocityConstraint velocityConstraint = new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(14d), new AngularVelocityConstraint(1.4d)));
	protected final static TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(7d);

	protected Pose2d dropFirst(Pose2d start, Vector2d center)
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		Launcher launcher = opMode.getBehavior(Launcher.class);
		WobbleGrabberI5 grabber = opMode.getBehavior(WobbleGrabberI5.class);
		WobbleDraggerI5 dragger = opMode.getBehavior(WobbleDraggerI5.class);

		SampleMecanumDrive drive = drivetrain.getDrive();

		execute(launcher, new Launcher.Lift(-1));

		Trajectory wobbleDrop = drive.trajectoryBuilder(start).forward(40f)
				.splineTo(new Vector2d(-3d, -11d).plus(center), Math.toRadians(0d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(wobbleDrop));

		wait(0.2f);

		execute(dragger, new WobbleDraggerI5.Drag(false));
		execute(grabber, new WobbleGrabberI5.Grab(false));
		execute(launcher, new Launcher.Prime(Launcher.HIGH_POWER, true));

		wait(0.5f);

		Trajectory strafeRight = drive.trajectoryBuilder(wobbleDrop.end())
				.strafeRight(12d).build();

		execute(drivetrain, new DrivetrainI5.Follow(strafeRight));

		Trajectory launchHigh = drive.trajectoryBuilder(strafeRight.end())
				.lineToLinearHeading(LAUNCH_POINT, velocityConstraint, accelerationConstraint).build();

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
		wait(0.75f);
		wait(0.75f);
		wait(0.75f);

		execute(launcher, new Launcher.Lift(-1));
	}

	protected Pose2d dropSecond(Pose2d start, Vector2d center, Vector2d target)
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		WobbleGrabberI5 grabber = opMode.getBehavior(WobbleGrabberI5.class);
		WobbleDraggerI5 dragger = opMode.getBehavior(WobbleDraggerI5.class);
		Launcher launcher = opMode.getBehavior(Launcher.class);

		SampleMecanumDrive drive = drivetrain.getDrive();

		Trajectory wobbleAlign = drive.trajectoryBuilder(start)
				.splineToLinearHeading(new Pose2d(-36d, target.getY(), Math.toRadians(-45d)), Math.toRadians(-90d), velocityConstraint, accelerationConstraint).build();

		buffer(grabber, new WobbleGrabberI5.Move(WobbleGrabberI5.Mode.GRAB));
		buffer(drivetrain, new DrivetrainI5.Follow(wobbleAlign));

		execute();

		Trajectory wobblePickup = drive.trajectoryBuilder(wobbleAlign.end())
				.lineToConstantHeading(target).build();

		execute(drivetrain, new DrivetrainI5.Follow(wobblePickup));
		execute(launcher, new Launcher.Prime(false));

		wait(0.5f);

		execute(dragger, new WobbleDraggerI5.Drag(true));
		execute(grabber, new WobbleGrabberI5.Grab(true));

		wait(0.5f);

		Trajectory wobbleDrop = drive.trajectoryBuilder(wobblePickup.end())
				.splineToSplineHeading(new Pose2d(new Vector2d(-6d, -20d).plus(center), Math.toRadians(180d)), Math.toRadians(0d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(wobbleDrop));

		wait(0.3f);

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
