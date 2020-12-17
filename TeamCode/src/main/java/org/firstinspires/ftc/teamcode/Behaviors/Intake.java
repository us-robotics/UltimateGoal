package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;

public class Intake extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public Intake(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		intake = hardwareMap.dcMotor.get("intake");
		intake.setPower(0d);
	}

	private DcMotor intake;

	@Override
	public void update()
	{
		super.update();

		if (opMode.getIsAuto()) return;
		Input input = opMode.getHelper(Input.class);

		intake.setPower(input.getTrigger(Input.Source.CONTROLLER_2, Input.Button.RIGHT_TRIGGER));
	}
}
