package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;
        import org.openftc.easyopencv.OpenCvInternalCamera;
        import org.firstinspires.ftc.teamcode.EOCV.aprilTag;

@TeleOp(name="AprilTag")
public class AprilTagOpmode extends OpMode
{

    public static aprilTag aprilTag;

    OpenCvCamera camera = null;
    int TagPosition = 0;



    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        aprilTag = new aprilTag(telemetry);




        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //camera
        camera.setPipeline(aprilTag);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);

            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });





    }
    public void init_loop(){

        TagPosition = aprilTag.detectionPosition;
        telemetry.addData("TagPosition", TagPosition);

    }
    public void start(){

    }

    public void loop(){
    }
}