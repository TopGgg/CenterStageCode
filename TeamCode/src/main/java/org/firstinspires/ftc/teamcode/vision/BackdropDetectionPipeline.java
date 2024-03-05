package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoPlay.Board;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public
class BackdropDetectionPipeline extends OpenCvPipeline
{
    public static double[] rowXMultiplier = new double[6];
    public static double[] rowYMultiplier = new double[6];
    public static double[] RadiusMultiplier = new double[] {1,0.97,0.97,0.92,0,0};
    public static double Radius = 0;
    public static double X = 0;
    public static double Y = 0;
    public static Point[][] margin = new Point[7][6];
    public static int xA = 0;
    public static int yA = 0;
    public static double xCorrection = 0;
    public static double yCorrection = 0;
    static {
        margin[0][0] = new Point(0,0.25);
        margin[1][0] = new Point(-0.25,0.25);
        margin[2][0] = new Point(-0.5,0.25);
        margin[3][0] = new Point(-0.8,0.25);
        margin[4][0] = new Point(-1.1,0.25);
        margin[5][0] = new Point(-1.4,0.25);

        margin[0][1] = new Point(0,0.3);
        margin[1][1] = new Point(-0.2,0.3);
        margin[2][1] = new Point(-0.45,0.3);
    }
    private long nativeApriltagPtr;
    private Mat grey = new Mat();

    public static Scalar yellowRef = new Scalar(202,179,110);
    public static Scalar greenRef = new Scalar(107,157,113);
    public static Scalar purpleRef = new Scalar(148,150,195);
    public static Scalar whiteRef = new Scalar(210,210,210);
    public static Scalar blankRef = new Scalar(55,58,66);

    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;
    public static int smallRectSize = 5;

    Scalar blue = new Scalar(7,197,235,255);
    Scalar red = new Scalar(255,0,0,255);
    Scalar green = new Scalar(0,255,0,255);
    Scalar white = new Scalar(255,255,255,255);

    double fx;
    double fy;
    double cx;
    double cy;

    public static double ratioSides = 6;
    public static double ratioUp = 9;
    public static double distRatio = 3.1252933262936105;
    public static double ratioToFirstX = -3.7;
    public static double ratioToFirstY = -1.7;
    public static double sizeRatio = 0.7;
    public static double[] xMultiplier = new double[] {1,0.9,-0.4,0.5,-0.65,0.2,-1.2,-0.2};
    public static double[] yMultiplier = new double[]{1,0.2,0.4,0.8,1,1.4,2.1,2.9};
    public static double xMultiplierd = 0;
    public static double yMultiplierd = 0;
    public static int index = 0;

    public static double[] spacing = new double[] {18,15,10,7,2,1,1.2,0.2};
    public static double spacingd = 0;
    public static int index2 = 0;


    // UNITS ARE METERS
    double tagsize;
    double tagsizeX;
    double tagsizeY;
    Point center = new Point(1920/2,1080/2);

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();
    private Telemetry telemetry;
    private Board.Backdrop board;
    public static double RMultiplier;
    public static double realRadius;
    public static double dDead;
    public static double hCam;
    public static double CF = 12.7;
    public static double distConst;

    public static boolean active = true;

    public BackdropDetectionPipeline(double tagsize, double fx, double fy, double cx, double cy, Telemetry telemetry, Board.Backdrop board)
    {

        this.tagsize = tagsize;
        this.tagsizeX = tagsize;
        this.tagsizeY = tagsize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.telemetry = telemetry;
        this.board = board;

        constructMatrix();

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public void finalize()
    {
        // Might be null if createApriltagDetector() threw an exception
        if(nativeApriltagPtr != 0)
        {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
        else
        {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

    @Override
    public Mat processFrame(Mat input)
    {
        if(active) {
            margin[xA][yA] = new Point(xCorrection, yCorrection);
            // Convert to greyscale
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

            synchronized (decimationSync) {
                if (needToSetDecimation) {
                    AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                    needToSetDecimation = false;
                }
            }

            // Run AprilTag
            detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

            synchronized (detectionsUpdateSync) {
                detectionsUpdate = detections;
            }

            // For fun, use OpenCV to draw 6DOF markers on the image.
            double length = 0;
            double n = 0;
            Point center2 = null;
            Point center1 = null;
            double distFromCenter = 0;
            for (AprilTagDetection detection : detections) {
//            if(detection.id == 2){
//                center = detection.center;
//            }else


                Pose pose = aprilTagPoseToOpenCvPose(detection.pose);
                //Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
                drawAxisMarker(input, tagsizeY / 2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
                length += draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
                n++;

                double l = length / n;
                distFromCenter = detection.pose.x;

                if (detection.id == 2) {
                    center = detection.center;
                    center2 = detection.center;
                } else if (detection.id == 1) {
                    center1 = detection.center;
                    center = new Point(detection.center.x + l * distRatio, detection.center.y);
                } else if (detection.id == 3) {
                    center = new Point(detection.center.x - l * distRatio, detection.center.y);
                }
            }
            length = Math.floor(length / n);
            length = length % 4 == 0 ? length : length + 2;
            if (center2 != null && center1 != null) {
                distRatio = (center2.x - center1.x) / length;
            }
//        double centerToEdge = ratioSides *length;
//        Rect rectCrop = new Rect(0, 0, (int) (center.x-centerToEdge), 1080);
//        Rect rectCrop2 = new Rect(new Point( (int) (center.x+centerToEdge), 0),new Point(1920, 1080));
//        Rect rectCrop3 = new Rect(new Point(0, (int) (center.y-length)),new Point(1920, 1080));
//        Rect rectCrop4 = new Rect(new Point(0, 0),new Point(1920, length*ratioUp));
//
//        Imgproc.rectangle(input, rectCrop, new Scalar(255,0,0),-1);
//        Imgproc.rectangle(input, rectCrop2, new Scalar(255,0,0),-1);
//        Imgproc.rectangle(input, rectCrop3, new Scalar(255,0,0),-1);
//        Imgproc.rectangle(input, rectCrop4, new Scalar(255,0,0),-1);
            Imgproc.circle(input, center, 30, new Scalar(0, 255, 0), -1);
            int R = (int) (sizeRatio * length);
//        RadiusMultiplier[index] = Radius;
//        rowXMultiplier[index] = X;
//        rowYMultiplier[index] = Y;


            for (int x = 0; x < 7; x++) {
                for (int y = 0; y < 4; y++) {
                    if (y % 2 == 0 && x == 6) continue;

//                if(index == y){
//                    xMultiplier[y] = xMultiplierd;
//                    yMultiplier[y] = yMultiplierd;
//                }
//                if(index2 == y){
//                    spacing[y] = spacingd;
//                }
                    double newR = R * RadiusMultiplier[y];
                    double halfWidthHex = Math.sqrt(3) / 2 * newR;
                    double xPoint = center.x - 6 * halfWidthHex + 2 * newR * x - (y % 2 == 0 ? 0 : halfWidthHex) - rowXMultiplier[y] * length + distFromCenter * distConst;
                    double yPoint = center.y + length * ratioToFirstY - 1.5 * y * newR - rowYMultiplier[y] * length;
//                    if(margin[x][y] != null){
//                        xPoint += margin[x][y].x*length;
//                        yPoint += margin[x][y].y*length;
//                    }
                    Scalar color = getAverageCircleColor((int) xPoint, (int) yPoint, R, input);
//                    Scalar color = blankRef;
                    float[] hsv = new float[3];
                    Color.RGBToHSV((int) color.val[0], (int) color.val[1], (int) color.val[2], hsv);
                    if (y == 0) {
                        if (x == 2) {
//                            telemetry.addData("0Color", "[" + Math.floor(hsv[0]) + "," + Math.floor(hsv[1]) + "," + Math.floor(hsv[2]) + "]");
                        } else if (x == 3) {
//                            telemetry.addData("1Color", "[" + Math.floor(hsv[0]) + "," + Math.floor(hsv[1]) + "," + Math.floor(hsv[2]) + "]");
                        }
                    }
                    Scalar realColor = blankRef;
                    if (!(hsv[1] < 0.2) || !(hsv[2] < 0.65)) {
                        double yellowDistance = color_distance(color, yellowRef);
                        double greenDistance = color_distance(color, greenRef);
                        double purpleDistance = color_distance(color, purpleRef);
                        double whiteDistance = color_distance(color, whiteRef);
                        double[] distances = new double[]{yellowDistance, greenDistance, purpleDistance
                                , whiteDistance};
                        double min = Double.MAX_VALUE;
                        for (double dis : distances) {
                            if (dis < min) {
                                min = dis;
                            }
                        }
                        if (min == yellowDistance) {
                            board.findPixel(x, y).type = Board.PIXEL_TYPE.YELLOW;
                            realColor = yellowRef;
                        } else if (min == greenDistance) {
                            board.findPixel(x, y).type = Board.PIXEL_TYPE.GREEN;
                            realColor = greenRef;
                        } else if (min == purpleDistance) {
                            board.findPixel(x, y).type = Board.PIXEL_TYPE.PURPLE;
                            realColor = purpleRef;
                        } else if (min == whiteDistance) {
                            board.findPixel(x, y).type = Board.PIXEL_TYPE.WHITE;
                            realColor = whiteRef;
                        }
                    } else {
                        board.findPixel(x, y).type = Board.PIXEL_TYPE.NONE;
                    }

                    Imgproc.circle(input,
                            new Point(xPoint, yPoint),
                            (int) newR, realColor, -1);
                    Imgproc.circle(input,
                            new Point(xPoint, yPoint),
                            (int) newR, new Scalar(0, 0, 0), 10);

//                    if(x == 0 && y == 0){
//                        telemetry.addData("R", color.val[0]);
//                        telemetry.addData("G", color.val[1]);
//                        telemetry.addData("B", color.val[2]);
//                        telemetry.update();
//                    }
                }
            }
//        Imgproc.circle(input, new Point(center.x +length*ratioToFirstX, center.y +length*ratioToFirstY), R, new Scalar(0,0,255), 5);
//        Imgproc.circle(input, new Point(center.x +length*ratioToFirstX+2*R, center.y +length*ratioToFirstY), R, new Scalar(0,0,255), 5);

//        return new Mat(new Mat(input,rectCrop), rectCrop2);
//        Imgproc.circle(input, center, 300, new Scalar(255,0,0), 10);
//        Scalar color = getAverageCircleColor((int) center.x, (int) cente
//        r.y,300, input);
        }


//        Board.Action[] actions = board.getBestActions();
//        telemetry.addData("score", board.score());
//        telemetry.addData("Action 1", actions[0].toString());
//        telemetry.addData("Action 2", actions[1].toString());
//        telemetry.addData("Action 3", actions[2].toString());
//        telemetry.addData("Radius", Arrays.toString(RadiusMultiplier));
//        telemetry.addData("X", Arrays.toString(rowXMultiplier));
//        telemetry.addData("Radius", Arrays.toString(rowYMultiplier));
//        telemetry.update();
        return input;
    }

    public double color_distance(Scalar color1, Scalar color2){
        double r1 = color1.val[0];
        double g1 = color1.val[1];
        double b1 = color1.val[2];

        double r2 = color2.val[0];
        double g2 = color2.val[1];
        double b2 = color2.val[2];

        return Math.sqrt(Math.pow((r1 - r2), 2) + Math.pow((g1 - g2), 2) + Math.pow((b1 - b2), 2));
    }

    public void setDecimation(float decimation)
    {
        synchronized (decimationSync)
        {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate()
    {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf the RGB buffer on which to draw the marker
     * @param length the length of each of the marker 'poles'
     * @param rvec the rotation vector of the detection
     * @param tvec the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(length,0,0),
                new Point3(0,length,0),
                new Point3(0,0,-length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }

    double draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2,-tagHeight/2,-length),
                new Point3(-tagWidth/2,-tagHeight/2,-length));

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Pillars
        for(int i = 0; i < 4; i++)
        {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines

        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
        return projectedPoints[5].x-projectedPoints[4].x;
    }

    Pose aprilTagPoseToOpenCvPose(AprilTagPose aprilTagPose)
    {
        Pose pose = new Pose();
        pose.tvec.put(0,0, aprilTagPose.x);
        pose.tvec.put(1,0, aprilTagPose.y);
        pose.tvec.put(2,0, aprilTagPose.z);

        Mat R = new Mat(3, 3, CvType.CV_32F);

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R.put(i,j, aprilTagPose.R.get(i,j));
            }
        }

        Calib3d.Rodrigues(R, pose.rvec);
        return pose;
    }

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX the original width of the tag
     * @param tagsizeY the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
    {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */

    public Scalar getAverageCircleColor(int x, int y, int R, Mat mat){
        try {
            double RAvg = 0;
            double GAvg = 0;
            double BAvg = 0;
            int n = 0;
            for (int i = -R; i < R; i += smallRectSize) {
                for (int j = -R; j < R; j += smallRectSize) {
                    int xRect = x + i;
                    int yRect = y + j;
                    if (Math.pow((xRect + (double) smallRectSize / 2) - x, 2) + Math.pow((yRect + (double) smallRectSize / 2) - y, 2) < Math.pow(R, 2)) {
                        Rect rect = new Rect(xRect, yRect, smallRectSize, smallRectSize);
                        Mat submat = mat.submat(rect);
                        Scalar avgColor = Core.mean(submat);
                        RAvg += avgColor.val[0];
                        GAvg += avgColor.val[1];
                        BAvg += avgColor.val[2];
                        n++;
//                    Imgproc.rectangle(mat, rect, new Scalar(255, 255,0), 1);
//                    Imgproc.circle(mat,new Point((xRect+ (double) smallRectSize /2),(yRect+ (double) smallRectSize /2)),5, new Scalar(0,0,255), -1);
                    }
                }
            }
            RAvg = RAvg / n;
            GAvg = GAvg / n;
            BAvg = BAvg / n;
            return new Scalar(RAvg, GAvg, BAvg);
        }catch (Exception exception){
            return blankRef;
        }
    }

    public double getYCorrection(double d, int yHeight){
        double dRow = 1.5*yHeight*realRadius + dDead;
        double dFromRow = Math.sin(Math.toRadians(30))*dRow;
        double hFromGround = Math.cos(Math.toRadians(30))*dRow;
        double delta = hFromGround - hCam - Math.sqrt(3)*realRadius;
        double DELTA = hFromGround - hCam;
        double AC = Math.sqrt(Math.pow(dFromRow,2) + Math.pow(delta,2));
        double BC = Math.sqrt(Math.pow(DELTA, 2) + Math.pow(dFromRow,2));
        double AB = Math.sqrt(3)*realRadius;
        double theta = Math.acos(-(Math.pow(AB,2)-Math.pow(BC,2)-Math.pow(AC,2))/(2*AC*AB));
        return (CF*theta)/2;
    }

    class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat(3, 1, CvType.CV_32F);
            tvec = new Mat(3, 1, CvType.CV_32F);
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }
}