package frc.robot.utils;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N4;

public class FMapConstant {

    public static String jsonobjConst = "{\"fiducials\":[{\"family\":\"apriltag3_36h11_classic\",\"id\":1,\"size\":165.1,\"transform\":[-0.5877852522924729,-0.8090169943749473,0,7.923198000000001,0.8090169943749473,-0.5877852522924729,0,-3.3706799999999997,0,0,1,1.4859,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":2,\"size\":165.1,\"transform\":[-0.5877852522924734,0.8090169943749473,0,7.923198000000001,-0.8090169943749473,-0.5877852522924734,0,3.3704799999999997,0,0,1,1.4859,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":3,\"size\":165.1,\"transform\":[-2.220446049250313e-16,1,0,2.786809999999999,-1,-2.220446049250313e-16,0,4.02961,0,0,1,1.30175,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":4,\"size\":165.1,\"transform\":[0.8660254037844387,0,0.49999999999999994,0.5020799999999994,0,1,0,2.111656,-0.49999999999999994,0,0.8660254037844387,1.8679160000000001,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":5,\"size\":165.1,\"transform\":[0.8660254037844387,0,0.49999999999999994,0.5020799999999994,0,1,0,-2.1110939999999996,-0.49999999999999994,0,0.8660254037844387,1.8679160000000001,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":6,\"size\":165.1,\"transform\":[0.5000000000000001,0.8660254037844386,0,4.700446000000001,-0.8660254037844386,0.5000000000000001,0,-0.7196820000000002,0,0,1,0.308102,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":7,\"size\":165.1,\"transform\":[1,0,0,5.116498,0,1,0,-0.00009999999999976694,0,0,1,0.308102,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":8,\"size\":165.1,\"transform\":[0.5000000000000001,-0.8660254037844386,0,4.700446000000001,0.8660254037844386,0.5000000000000001,0,0.7194820000000002,0,0,1,0.308102,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":9,\"size\":165.1,\"transform\":[-0.4999999999999998,-0.8660254037844388,0,3.869358,0.8660254037844388,-0.4999999999999998,0,0.7194820000000002,0,0,1,0.308102,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":10,\"size\":165.1,\"transform\":[-1,-1.2246467991473532e-16,0,3.4533059999999995,1.2246467991473532e-16,-1,0,-0.00009999999999976694,0,0,1,0.308102,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":11,\"size\":165.1,\"transform\":[-0.5000000000000002,0.8660254037844384,0,3.869358,-0.8660254037844384,-0.5000000000000002,0,-0.7196820000000002,0,0,1,0.308102,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":12,\"size\":165.1,\"transform\":[0.5877852522924731,-0.8090169943749473,0,-7.922845999999999,0.8090169943749473,0.5877852522924731,0,-3.3706799999999997,0,0,1,1.4859,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":13,\"size\":165.1,\"transform\":[0.587785252292473,0.8090169943749475,0,-7.922845999999999,-0.8090169943749475,0.587785252292473,0,3.3704799999999997,0,0,1,1.4859,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":14,\"size\":165.1,\"transform\":[-0.8660254037844388,-1.2246467991473532e-16,-0.49999999999999994,-0.501728,1.0605752387249069e-16,-1,6.123233995736766e-17,2.111656,-0.49999999999999994,0,0.8660254037844387,1.8679160000000001,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":15,\"size\":165.1,\"transform\":[-0.8660254037844388,-1.2246467991473532e-16,-0.49999999999999994,-0.501728,1.0605752387249069e-16,-1,6.123233995736766e-17,-2.1110939999999996,-0.49999999999999994,0,0.8660254037844387,1.8679160000000001,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":16,\"size\":165.1,\"transform\":[-2.220446049250313e-16,-1.0000000000000002,0,-2.7864579999999997,1.0000000000000002,-2.220446049250313e-16,0,-4.0298099999999994,0,0,1,1.30175,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":17,\"size\":165.1,\"transform\":[-0.5000000000000002,0.8660254037844384,0,-4.700094,-0.8660254037844384,-0.5000000000000002,0,-0.7196820000000002,0,0,1,0.308102,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":18,\"size\":165.1,\"transform\":[-1,-1.2246467991473532e-16,0,-5.116399999999999,1.2246467991473532e-16,-1,0,-0.00009999999999976694,0,0,1,0.308102,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":19,\"size\":165.1,\"transform\":[-0.4999999999999998,-0.8660254037844388,0,-4.700094,0.8660254037844388,-0.4999999999999998,0,0.7194820000000002,0,0,1,0.308102,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":20,\"size\":165.1,\"transform\":[0.5000000000000001,-0.8660254037844386,0,-3.8692599999999997,0.8660254037844386,0.5000000000000001,0,0.7194820000000002,0,0,1,0.308102,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":21,\"size\":165.1,\"transform\":[1,0,0,-3.452953999999999,0,1,0,-0.00009999999999976694,0,0,1,0.308102,0,0,0,1],\"unique\":true},{\"family\":\"apriltag3_36h11_classic\",\"id\":22,\"size\":165.1,\"transform\":[0.5000000000000001,0.8660254037844386,0,-3.8692599999999997,-0.8660254037844386,0.5000000000000001,0,-0.7196820000000002,0,0,1,0.308102,0,0,0,1],\"unique\":true}],\"type\":\"frc\"}";

    public static Pose3d getFMapPosition(int tagId) {
        JSONParser parser = new JSONParser();
        try {
            JSONObject top = (JSONObject) parser.parse(jsonobjConst);
            JSONArray fiducials = (JSONArray) top.get("fiducials");
            for (Object item : fiducials) {
                JSONObject tag = (JSONObject) item;
                if (((Long) tag.get("id")).intValue() == tagId) {
                    JSONArray transformArr = (JSONArray) tag.get("transform");
                    double[] matrixData = new double[transformArr.size()];
                    for (int i = 0; i < transformArr.size(); i++) {
                        matrixData[i] = ((Number) transformArr.get(i)).doubleValue();
                    }
                    Matrix<N4, N4> transform = new Matrix<>(N4.instance, N4.instance);
                    for (int j = 0; j < 4; j++) {
                        for (int k = 0; k < 4; k++) {
                            transform.set(j, k, matrixData[j * 4 + k]);
                        }
                    }
                    return new Pose3d(transform);
                }
            }
        } catch (ParseException e) {
            e.printStackTrace();
        }
        return null;
    }

}
