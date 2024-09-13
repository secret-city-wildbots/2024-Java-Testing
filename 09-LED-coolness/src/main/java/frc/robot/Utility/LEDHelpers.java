package frc.robot.Utility;

public class LEDHelpers {
    public static String hsvToRgb(float hue, float saturation, float value) {

        int h = (int)(hue * 6);
        float f = hue * 6 - h;
        float p = value * (1 - saturation);
        float q = value * (1 - f * saturation);
        float t = value * (1 - (1 - f) * saturation);
    
        switch (h) {
          case 0: return rgbToString(value, t, p);
          case 1: return rgbToString(q, value, p);
          case 2: return rgbToString(p, value, t);
          case 3: return rgbToString(p, q, value);
          case 4: return rgbToString(t, p, value);
          case 5: return rgbToString(value, p, q);
          default: throw new RuntimeException("Something went wrong when converting from HSV to RGB. Input was " + hue + ", " + saturation + ", " + value);
        }
    }
    public static String rgbToString(float r, float g, float b) {
        String rs = Integer.toHexString((int)(r * 256));
        String gs = Integer.toHexString((int)(g * 256));
        String bs = Integer.toHexString((int)(b * 256));
        return rs + gs + bs;
    }
    public static String rgbtogrb(String rgb) {
        return rgb.substring(2, 3) + rgb.substring(0, 1) + rgb.substring(4, 5);
    }
}
