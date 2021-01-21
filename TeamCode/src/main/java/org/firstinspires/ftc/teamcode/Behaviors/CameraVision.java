package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.openftc.easyopencv.*;

import FTCEngine.Core.*;

public class CameraVision extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public CameraVision(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		WebcamName webcamName = hardwareMap.get(WebcamName.class, "frontCamera");

		camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{
				pipeline = new CameraPipeline();

				camera.startStreaming(320, 240);
				camera.setPipeline(pipeline);
			}
		});
	}

	private OpenCvCamera camera;
	private CameraPipeline pipeline;

	@Override
	public void update()
	{
		super.update();

		Telemetry telemetry = opMode.getHelper(Telemetry.class);

		if (pipeline == null) telemetry.addData("No Pipeline", 0);
		else
		{
			Scalar mean = pipeline.getMean();

			if (mean == null) telemetry.addData("No Mean", 1);
			else telemetry.addData("Mean", mean);
		}
	}

	private static class CameraPipeline extends OpenCvPipeline
	{
		public CameraPipeline() {}

		private Scalar mean;

		@Override
		public Mat processFrame(Mat input)
		{
			mean = Core.mean(input);
			return input;
		}

		public Scalar getMean()
		{
			return mean;
		}
	}
}
