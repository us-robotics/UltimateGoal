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


		Trajectory goLaunchOne = drive.trajectoryBuilder(wobbleDropSecond.end())
				.splineTo(new Vector2d(2d, 28d), Math.toRadians(180d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(goLaunchOne));

		Trajectory goLaunchTwo = drive.trajectoryBuilder(goLaunchOne.end())
				.strafeTo(new Vector2d(2d, 20.5d)).build();

		wait(0.3f);
		execute(drivetrain, new DrivetrainI5.Follow(goLaunchTwo));

		Trajectory goLaunchThree = drive.trajectoryBuilder(goLaunchTwo.end())
				.strafeTo(new Vector2d(2d, 13d)).build();

		wait(0.3f);
		execute(drivetrain, new DrivetrainI5.Follow(goLaunchThree));

		Trajectory goPark = drive.trajectoryBuilder(goLaunchThree.end(), true)
				.splineTo(new Vector2d(12d, 32d), Math.toRadians(150d)).build();

		wait(0.3f);
		execute(drivetrain, new DrivetrainI5.Follow(goPark));
	}
}
