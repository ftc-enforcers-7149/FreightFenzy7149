package org.firstinspires.ftc.teamcode.Mattu.recordnreplay;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.security.Key;
import java.util.ArrayList;

/**
 * This class is used to save and load data for our Record 'N Replay Autonomous
 */
public class Path {

    /**
     * Serializes a path of KeyPoints to a file
     * @param path The array of key points to serialize
     * @param fileName The full location and name for where to save the file
     */
    public static void savePath(ArrayList<KeyPoint> path, String fileName) {
        FileOutputStream fos;
        ObjectOutputStream out;

        try {
            fos = new FileOutputStream(fileName); //Opens the file
            out = new ObjectOutputStream(fos); //Used to serialize data into the file

            //Serialize the entire ArrayList as a whole
            out.writeObject(path);

            //Close everything to clean it up
            out.close();
            fos.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Deserializes a path of KeyPoints
     * @param fileName The full location and name for where the path is saved
     * @return
     */
    public static ArrayList<KeyPoint> loadPath(String fileName) {
        FileInputStream fis;
        ObjectInputStream in;
        ArrayList path =  new ArrayList<>();

        try {
            fis = new FileInputStream(fileName); //Opens the file
            in = new ObjectInputStream(fis); //Used for deserialization

            //Deserializes all contents of the file into an array
            //WILL cause problems if the serialized version of KeyPoint doesn't match the most
            //current version
            path = (ArrayList) in.readObject();

            //Close everything to clean it up
            in.close();
            fis.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

        return path;
    }

    /**
     * Saves the literal string code into a text file for readability
     * @param pathCode An array of lines of code (in order)
     * @param fileName The full location and name of the text file to save everything in
     */
    public static void savePathCode(ArrayList<String> pathCode, String fileName) {
        try {
            FileWriter writer = new FileWriter(fileName); //Opens a writable file

            //Saves each line (or lines) of code in the array, followed by a new line
            //to separate it
            for (String code : pathCode) {
                writer.write(code + "\n\n");
            }

            //Close everything to clean it up
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
