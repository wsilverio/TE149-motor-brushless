import controlP5.*;

ControlP5 cp5;
FloatList values;

int xmin, xmax;
int ymin, ymax;
int posx;

int N, b;
float alpha;
float toff, t;

void setup() {
  size(900, 500);
  rectMode(CORNERS);

  xmin = 0;
  xmax = int(3*width/4);
  ymin = -1;
  ymax = height;

  posx = xmin;
  toff = 0f;

  values = new FloatList();
  for (int i = posx; i < xmax; i++) {
    values.append(0);
  }

  int cp5width = width-xmax;
  cp5 = new ControlP5(this);

  cp5.addSlider("b")
    .setPosition(xmax+5, height-40)
    .setSize(cp5width-15-5, 10)
    .setRange(0, 500)
    .setNumberOfTickMarks(50)
    .setValue(10);

  cp5.addSlider("t")
    .setPosition(xmax+5, height-60)
    .setSize(cp5width-15-5, 10)
    .setRange(0.0, 2.5)
    .setValue(0.007);

  cp5.addSlider("N")
    .setPosition(xmax+5, height-100)
    .setSize(cp5width-15-5, 10)
    .setRange(5, 50)
    .setNumberOfTickMarks(10)
    .setSliderMode(Slider.FLEXIBLE)
    .setValue(20);
}

boolean resetColors = true;
float input = 0, lastInput;

void draw() {
  if (resetColors) {
    fill(50);
    rect(xmax, -1, width, height);  
    fill(240);
    rect(xmin-1, ymin-1, xmax, ymax);
    resetColors = false;
  }

  toff += t;
  lastInput = input;
  input = noise(toff);
  input = (ymax-ymin)/2 + map(input, 0, 1, -50, 50);

  if (posx>0) {
    alpha = float(N)/(N+b);
    values.set(posx, input*alpha+values.get(posx-1)*(1f-alpha));

    stroke(28);
    line(posx-1, lastInput, posx, input);
    stroke(#FF0022);
    line(posx-1, values.get(posx-1), posx, values.get(posx));
  }

  println(t);

  posx++;
  if (posx > xmax) {
    values.set(0, input*alpha+values.get(posx-1)*(1f-alpha));
    posx = xmin;
    resetColors = true;
  }
}