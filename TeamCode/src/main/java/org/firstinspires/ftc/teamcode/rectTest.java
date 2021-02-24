package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;
import org.opencv.core.Rect;


public class rectTest {

    final double frameWidth = 320;
    final double frameHeight = 240;

    final double leftMargin = 120;         // Left margin to be cropped off processedImageCr
    final double righMargin = 120;         // Rign margin to be cropped off
    final double topMargin = 50;           // Top margin to be cropped off
    final double botMargin = 140;          // Bottom margin to be cropped off


    private Point upperLeft = new Point(leftMargin, topMargin);
    private Point lowerRight = new Point(frameWidth - righMargin, frameHeight - botMargin);

    private Rect centerScanRect = new Rect(upperLeft, lowerRight);

    private double zoneWidth = centerScanRect.width;
    private double zoneHeight = centerScanRect.height;


}
