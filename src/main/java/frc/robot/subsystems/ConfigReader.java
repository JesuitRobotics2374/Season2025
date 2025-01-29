package frc.robot.subsystems;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.ParseException;
import java.util.*;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class ConfigReader {

    File file;

    public ConfigReader(File file){
        this.file = file;
    }


    public Object convertString(String value){
        try{
            return Integer.parseInt(value);
        }catch(NumberFormatException e){

        }

        try{
            return Double.parseDouble(value);
        }catch(NumberFormatException e){

        }

        try{
            return Boolean.parseBoolean(value);
        }catch(NumberFormatException e){

        }


        return value;

    }
}
