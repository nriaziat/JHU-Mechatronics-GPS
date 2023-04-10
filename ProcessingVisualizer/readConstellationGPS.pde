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
float w = tan(0.29) * 244.0;
float h = tan(0.2) * 244.0;
float d;

void setup()
{
  size(1024, 512);
  w = int(map(w, 0, 244, 0, width));
  h = int(map(h, 0, 244, 0, width));
  d = (w * w + h * h);
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
  //map[0].x = 10;
  //map[0].y = 10;
  //map[1].x = 10;
  //map[1].y = 70;
  //map[2].x = 50;
  //map[2].y = 50;
  //map[3].x = 100;
  //map[3].y = 100;
  //map[4].x = 120;
  //map[4].y = 10;
  //map[5].x = 150;
  //map[5].y = 90;
  //map[6].x = 200;
  //map[6].y = 30;
  //map[7].x = 30;
  //map[7].y = 30;
  //map[8].x = 45;
  //map[8].y = 110;
  //map[9].x = 190;
  //map[9].y = 50;
  //map[10].x = 175;
  //map[10].y = 10;
  //map[11].x = 20;
  //map[11].y = 80;
  //map[12].x = 160;
  //map[12].y = 40;
  //map[13].x = 120;
  //map[13].y = 60;
  map[0].x = 10;
  map[0].y = 90;
  map[1].x = 40;
  map[1].y = 90;
  map[2].x = 70;
  map[2].y = 30;
  map[3].x = 100;
  map[3].y = 30;
  map[4].x = 160;
  map[4].y = 60;
  map[5].x = 160;
  map[5].y = 30;
  map[6].x = 220;
  map[6].y = 90;  
  map[7].x = 220;
  map[7].y = 60;

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
            inputs[i+1] = int(map(inputs[i+1], 0, 244, 0, width));
            inputs[i+2] = int(map(inputs[i+2], 0, 122, 0, height));
            if (index > 50){              
              noFill();
              stroke(0, 0, 255);
              ellipse(inputs[i+1], inputs[i+2], sqrt(d), sqrt(d));
              fill(0, 0, 255);
              stroke(0, 0, 0);
              radius = 10;
            } else if (index > 0){
              fill(255, 0, 0);
              radius = 3 + int(inputs[i+4] * 5);
            } else {
              noFill();
              stroke(0, 255, 0);
              ellipse(inputs[i+1], inputs[i+2], sqrt(d), sqrt(d));
              stroke(0, 0, 0);
              fill(0,255,0);
              radius = 25;
            }
            ellipse(inputs[i+1], inputs[i+2], radius, radius);
            float x1 = inputs[i+1] - radius/2 * cos(inputs[i+3]);
            float x2 = inputs[i+1] + radius/2 * cos(inputs[i+3]);
            float y1 = inputs[i+2] - radius/2 * sin(inputs[i+3]);
            float y2 = inputs[i+2] + radius/2 * sin(inputs[i+3]);
            line(x1, y1, x2, y2);
          }
        }
      }
    }
  }
}
