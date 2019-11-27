package org.firstinspires.ftc.teamcode.sensors;

import android.graphics.Bitmap;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.Robot;

import java.io.File;
import java.io.FileOutputStream;

class SkystoneDetector {

    private VuforiaLocalizer vuforia;

    public SkystoneDetector(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;
    }

    public void test()
    {
        getBitmap(true, true);
    }

    private Bitmap getBitmap(boolean saveBitmaps, boolean red) {

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

//        //Cropped Bitmap to show only stones
//        bitmap = Bitmap.createBitmap(bitmap, position.leftX, position.topY,
//                position.rightX-position.leftX, position.bottomY-position.topY);
//
//        // Save cropped bitmap to file
//        if (saveBitmaps)
//            saveBitmap(bitmap, croppedBitmapName);

        return Bitmap.createScaledBitmap(bitmap, 110, 20, true); //Compress bitmap to reduce scan time


    }

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
