import processing.serial.*;

Serial myPort;  // Create object from Serial class
String val;      // Data received from the serial port

int[][] colors = {{255, 255, 255}, {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {128, 128, 255}, {0, 128, 30}, {35, 45, 120}};

public class Point{
  public float x;
  public float y;
  Point(){};
};

Point[] map;
//float w = tan(0.29) * 202.0;
//float h = tan(0.2) * 202.0;
//float d;

void setup()
{
  size(1024, 512);
  //w = int(map(w, 0, 244, 0, width));
  //h = int(map(h, 0, 244, 0, width));
  //d = (w * w + h * h);
  // I know that the first port in the serial list on my mac
  // is always my  FTDI adaptor, so I open Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.
  print(Serial.list());
  String portName = Serial.list()[4];
  myPort = new Serial(this, portName, 115200);
  map = new Point[8];
  for (int i=0; i < map.length; i++){
    map[i] = new Point();
  }
  map[0].x = 63.5;
  map[0].y = 61;
  map[1].x = 82.6;
  map[1].y = 71.1;
  map[2].x = 121.9;
  map[2].y = 61;
  map[3].x = 121.9;
  map[3].y = 31.8;
  map[4].x = 111.8;
  map[4].y = 100.3;
  map[5].x = 141;
  map[5].y = 71.1;
  map[6].x = 161.3;
  map[6].y = 50.8;  
  map[7].x = 190.5;
  map[7].y = 71.1;  

  for (Point p : map){
    p.x = int(map(p.x, 0, 244, 0, width));
    p.y = int(map(p.y, 0, 122, 0, height));  
  }
}

void draw()
{
  int index = 0;
  int radius = 10;
  for (Point p : map){
    fill(255, 255, 255);
    ellipse(p.x, p.y, 10, 10);
  }
  if ( myPort.available() > 0) {  // If data is available,
    background(30);
    val = myPort.readStringUntil('\n');
    //println(val);
    if (val != null){
      String[] strings = split(val, ',');
      if (strings != null){
        float[] inputs = float(strings);
        for (int i=0; i<inputs.length-4; i+=5){
          index = int(inputs[i]);
          if (inputs[i+1] > 0 && inputs[i+2] > 0){
            int px_input_x = int(map(inputs[i+1], 0, 244, 0, width));
            int px_input_y = int(map(inputs[i+2], 0, 122, 0, height));
            if (index > 50){  
              
              //noFill();
              //stroke(0, 0, 255);
              //ellipse(inputs[i+1], inputs[i+2], sqrt(d), sqrt(d));
              fill(0, 0, 255);
              stroke(0, 0, 0);
              radius = 10;
              print("X: ");
              print(inputs[i+1]);
              print(", Y: ");
              println(inputs[i+2]);
            } else if (index > 0){
              fill(255, 0, 0);
              radius = 3 + int(inputs[i+4] * 5);
            } else {
              noFill();
              stroke(0, 255, 0);
              //ellipse(inputs[i+1], inputs[i+2], sqrt(d), sqrt(d));
              stroke(0, 0, 0);
              fill(0,255,0);
              radius = 25;
            }
            ellipse(px_input_x, px_input_y, radius, radius);
            //float x1 = inputs[i+1] - radius/2 * cos(inputs[i+3]);
            //float x2 = inputs[i+1] + radius/2 * cos(inputs[i+3]);
            //float y1 = inputs[i+2] - radius/2 * sin(inputs[i+3]);
            //float y2 = inputs[i+2] + radius/2 * sin(inputs[i+3]);
            //line(x1, y1, x2, y2);
          }
        }
      }
    }
  }
}
