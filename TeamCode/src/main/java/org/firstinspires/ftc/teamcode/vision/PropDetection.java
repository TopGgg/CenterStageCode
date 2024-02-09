package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipeline.PropDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class PropDetection {
    OpenCvCamera camera;
    PropDetectionPipeline propDetectionPipeline;
    int camW = 1280;
    int camH = 720;

    int zone = 1;

    public PropDetection(HardwareMap hardwareMap){
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        propDetectionPipeline = new PropDetectionPipeline();

        camera.setPipeline(propDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public void setAlliance(String alliance){
        propDetectionPipeline.setAlliancePipe(alliance);
    }

    public int elementDetection(Telemetry telemetry) {
        zone = propDetectionPipeline.get_element_zone();
        telemetry.addData("Distance1", propDetectionPipeline.distance1);
        telemetry.addData("Distance2", propDetectionPipeline.distance2);
        telemetry.addData("Element Zone", zone);
        return zone;
    }

    public void toggleAverageZone(){
        propDetectionPipeline.toggleAverageZonePipe();
    }

    public double getMaxDistance(){
        return propDetectionPipeline.getMaxDistance();
    }
}

