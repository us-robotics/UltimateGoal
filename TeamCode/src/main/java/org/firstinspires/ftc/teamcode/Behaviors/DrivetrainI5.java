package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.OpModeBase;

public class DrivetrainI5 extends AutoBehavior<Drivetrain.Job>
{
	public DrivetrainI5(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);
	}

	SampleMecanumDrive drive;

	@Override
	public void update()
	{
		super.update();
	}

	@Override
	protected void updateJob()
	{

	}
}
