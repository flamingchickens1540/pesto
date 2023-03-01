package org.team1540.robot2023.vision;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.vision.Limelight;


public class Vision extends CommandBase {
    private Limelight limelight = new Limelight("");
    private VisionThread visionThread;
    private final Object imgLock = new Object();

    DoublePublisher xPub;
    DoublePublisher yPub;
    DoubleSubscriber ySub;
    double prev;

    //NetworkTable table = NetworkTableInstance.getDefault().getTable("GRIP/mycontoursReport");
    //private double centerX = 0.0
    //DoubleArraySubscriber centerX = table.getDoubleArrayTopic("area").subscribe(new double[] {});
    public Vision(Limelight limelight) {
        this.limelight = limelight;
        addRequirements(limelight);
//        NetworkTableInstance inst = NetworkTableInstance.getDefault();
//
//        // Get the table within that instance that contains the data. There can
//        // be as many tables as you like and exist to make it easier to organize
//        // your data. In this case, it's a table called datatable.
//        NetworkTable table = inst.getTable("datatable");
//        ySub = table.getDoubleTopic("Y").subscribe(0.0);
//
//        // Start publishing topics within that table that correspond to the X and Y values
//        // for some operation in your program.
//        // The topic names are actually "/datatable/x" and "/datatable/y".
//        xPub = table.getDoubleTopic("/datatable/x").publish();
//        yPub = table.getDoubleTopic("/datatable/y").publish();
    }

    // may not be necessary for robot programs if this class lives for
    // the length of the program
//    public void close() {
//        ySub.close();
//    }
    @Override
    public void execute() {
        limelight.getDistanceFromLimelightToGoalInches();
//        limelight.getTa();
//        System.out.println("target area = " + limelight.getTa());
//        double y = limelight.getTy();
        double x = limelight.getTx();
        System.out.println("horizontal offset = " + x);
//        SmartDashboard.putNumber("LimelightX", tx);
//        SmartDashboard.putNumber("LimelightY", y);
//        // get() can be used with simple change detection to the previous value
//        double value = ySub.get();
//        if (value != prev) {
//            prev = value;  // save previous value
//            System.out.println("X changed value: " + value);
//        }
//
//        // readQueueValues() provides all value changes since the last call;
//        // this way it's not possible to miss a change by polling too slowly
//        for (double iterVal : ySub.readQueueValues()) {
//            System.out.println("X changed value: " + iterVal);
//        }
//        limelight.isTargetAligned();

        //\SmartDashboard.putNumber("LimelightArea", area);
       /* for (double area : limelight.getAreas()) {
            System.out.print(area + "area hehe");
        }

        //System.out.println("center x = " + centerX);
        //limelight.limelightTable.getEntry("")
        //visionThread = new VisionThread(limelight, new GripPipelineFinal(), gripPipeline -> {
            /*if (!gripPipelineFinal.filterContoursOutput().isEmpty()) {
                gripPipelineFinal.filterContoursOutput().get(0);
                /*Rect r = Imgproc.boundingRect(gripPipelineFinal.filterContoursOutput().get(0));
                synchronized (imgLock) {
                    centerX = r.x + (r.width / 2);
                }

                System.out.println("filterContoursOutput" + gripPipelineFinal.filterContoursOutput().get(0));
            } else {
                //System.out.println("is empty");
            }
             */
        //});
       // visionThread.start();

    }

    /*public void output(){
        visionThread = new VisionThread(limelight, new GripPipelineFinal(), pipeline -> {
            if (!pipeline.filterContoursOutput().isEmpty()) {
                Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                synchronized (imgLock) {
                    centerX = r.x + (r.width / 2);
                }
            }
        });
        visionThread.start();
    }

    */
}


//filterContoursOutput