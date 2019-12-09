package org.firstinspires.ftc.teamcode.sensors;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

import java.io.File;
import java.io.FileOutputStream;

public class SkystoneDetector {

    private VuforiaLocalizer vuforia;

    private final int RED_THRESHOLD = 100, GREEN_THRESHOLD = 100, BLACK_PIXEL_THRESHOLD = 20;

    public SkystoneDetector(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;
    }

    /**
     * All possible positions for the Skystone, either Far from bridge, Near bridge, in the Center
     * or the Robot doesn't know
     */
    public enum SkystonePosition {
        FAR, CENTER, NEAR, UNCERTAIN
    }

    /**
     * Starting positions for the robot and the crop values at that position
     */
    public enum RobotPosition {

        BLUE_POSITION1(658, 1070, 269, 305);


        public final int leftX, rightX, topY, bottomY;
        RobotPosition(int leftX, int rightX, int topY, int bottomY)
        {
            this.leftX = leftX;
            this.rightX = rightX;
            this.topY = topY;
            this.bottomY = bottomY;
        }
    }

    /**
     * Returns the position of the Skystone on the field
     * @param saveBitmaps - true if user wants the images taken by the phone to be saved
     * @param red - true if on the red side
     * @param position - the position of the robot on the field
     * @return the position of the Skystone on the field
     */
    public SkystonePosition getSkystonePosition(boolean saveBitmaps, boolean red, RobotPosition position)
    {
        RobotMap.telemetry.addLine("Running Scan");
        Bitmap bitmap = getBitmap(saveBitmaps, red, position);
        if( bitmap == null)
            return SkystonePosition.UNCERTAIN;

        FtcDashboard.getInstance().sendImage(bitmap);
        int stoneOneBlacks = 0, stoneTwoBlacks = 0;
        for(int i = 0; i < bitmap.getWidth(); i ++)
        {
            int pixel = bitmap.getPixel(i,  bitmap.getHeight()/2);
            RobotMap.telemetry.addLine("Pixel " + i  + ": R - " + Color.red(pixel) + " G - " + Color.green(pixel) );
            boolean isBlack = Color.red(pixel) < RED_THRESHOLD && Color.green(pixel) < GREEN_THRESHOLD;
            if(isBlack)
            {
                if(i <  bitmap.getWidth()/2)
                    stoneOneBlacks ++;
                else
                    stoneTwoBlacks ++;
            }

        }

        RobotMap.telemetry.addData("Stone One Blacks: ", stoneOneBlacks);
        RobotMap.telemetry.addData("Stone Two Blacks: ", stoneTwoBlacks);

        if(stoneOneBlacks > BLACK_PIXEL_THRESHOLD) {
            if (red)
                return SkystonePosition.CENTER;
            else
                return SkystonePosition.NEAR;
        }
        if(stoneTwoBlacks > BLACK_PIXEL_THRESHOLD) {
            if(red)
                return SkystonePosition.NEAR;
            else
                return SkystonePosition.CENTER;
        }
        else
            return SkystonePosition.FAR;

    }

    /**
     * Returns a cropped bitmap image of the field
     * @param saveBitmaps - true if user wants the images taken by the phone to be saved
     * @param red - true if on the red side
     * @param position - the position of the robot on the field
     * @return a cropped bitmap image of the field
     */
    private Bitmap getBitmap(boolean saveBitmaps, boolean red, RobotPosition position) {

        Image rgbImage = getImage();
        if(rgbImage == null)
            return null;

        // copy the bitmap from the Vuforia frame
        Bitmap bitmap = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
        bitmap.copyPixelsFromBuffer(rgbImage.getPixels());

        String bitmapName = "BitmapBLUE.png";
        String croppedBitmapName = "BitmapCroppedBLUE.png";

        if (red) {
            bitmapName = "BitmapRED.png";
            croppedBitmapName = "BitmapCroppedRED.png";
        }

        //Save bitmap to file
        if (saveBitmaps)
            saveBitmap(bitmap, bitmapName);

        //Cropped Bitmap to show only stones
        bitmap = Bitmap.createBitmap(bitmap, position.leftX, position.topY,
                position.rightX-position.leftX, position.bottomY-position.topY);

        // Save cropped bitmap to file
        if (saveBitmaps)
            saveBitmap(bitmap, croppedBitmapName);

        return Bitmap.createScaledBitmap(bitmap, 110, 20, true); //Compress bitmap to reduce scan time


    }

    /**
     * Returns the current Image on the vuforia stream
     * @return the current Image on the vuforia stream
     */
    private Image getImage()
    {
        Image image = null;
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        int originalCapacity = vuforia.getFrameQueueCapacity();
        vuforia.setFrameQueueCapacity(1);
        try {
            VuforiaLocalizer.CloseableFrame closeableFrame = this.vuforia.getFrameQueue().take();
            for (int i = 0; i < closeableFrame.getNumImages(); i++) {
                if (closeableFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    image = closeableFrame.getImage(i);
                    if (image != null)
                        break;
                }
            }
            closeableFrame.close();
        } catch (InterruptedException exc) {

        }
        vuforia.setFrameQueueCapacity(originalCapacity);
        return image;
    }

    /**
     * Saves bitmap to a file as a .png
     * @param bitmap - the bitmap to save as a .png
     * @param fileName - the file to save to
     */
    private void saveBitmap(Bitmap bitmap, String fileName)
    {
        try {
            FileOutputStream out = new FileOutputStream(new File(Robot.filePath, fileName));
            bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.flush();
            out.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
