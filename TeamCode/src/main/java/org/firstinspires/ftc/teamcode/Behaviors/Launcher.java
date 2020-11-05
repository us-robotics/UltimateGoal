package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;

public class Launcher extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public Launcher(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);
		launcher = hardwareMap.dcMotor.get("launcher");
		launcher.setPower(0d);
	}

	private DcMotor launcher;

	@Override
	public void update()
	{
		super.update();

		Input input = opMode.getHelper(Input.class);
		boolean pressed = input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.A);

		launcher.setPower(pressed ? 1d : 0d);
	}
}
