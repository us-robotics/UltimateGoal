package org.firstinspires.ftc.teamcode.Iteration5;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.blocks.ftcrobotcontroller.util.CurrentGame;

import org.firstinspires.ftc.teamcode.Behaviors.DrivetrainI5;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleDraggerI5;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabberI5;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.OpModeBase;

public class SequenceA extends CommonSequence
{
	public SequenceA(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	protected void queueJobs()
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		SampleMecanumDrive drive = drivetrain.getDrive();

		Pose2d position = new Pose2d(-63d, 53d, 0d);
		Vector2d center = new Vector2d(12d, 60d);

		drive.setPoseEstimate(position);

		position = dropFirst(position, center);
		position = dropSecond(position, center);

		Trajectory park = drive.trajectoryBuilder(position, true)
				.splineTo(new Vector2d(12d, 32d), Math.toRadians(150d)).build();

		execute(drivetrain, new DrivetrainI5.Follow(park));
	}
}
