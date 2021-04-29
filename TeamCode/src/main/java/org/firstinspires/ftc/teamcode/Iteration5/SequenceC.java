package org.firstinspires.ftc.teamcode.Iteration5;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Behaviors.DrivetrainI5;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import FTCEngine.Core.OpModeBase;

public class SequenceC extends CommonSequence
{
	public SequenceC(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	protected void queueJobs()
	{
		DrivetrainI5 drivetrain = opMode.getBehavior(DrivetrainI5.class);
		SampleMecanumDrive drive = drivetrain.getDrive();

		Pose2d position = new Pose2d(-63d, 53d, 0d);
		Vector2d center = new Vector2d(60d, 60d);

		drive.setPoseEstimate(position);

		position = dropFirst(position, center);
		position = dropSecond(position, center, new Vector2d(-41d, 34d));

		Trajectory park = drive.trajectoryBuilder(position)
				.splineToSplineHeading(new Pose2d(12d, 36d, Math.toRadians(180d)), Math.toRadians(180f)).build();

		execute(drivetrain, new DrivetrainI5.Follow(park));
	}
}
