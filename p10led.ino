#define PIN_OE    2  // white
#define PIN_A     3  // red
#define PIN_B     4  // orange
#define PIN_CLK   5  // yellow
#define PIN_SCLK  6  // green
#define PIN_R     7  // blue
//      PIN_GND         black
//#define PIN_AUDIO A0

#define LED      13
//#define SHADES_OF_GRAY

#define SNAKE_HISTORY 384


#define PANELS       11
#define WIDTH        (PANEL_WIDTH*PANELS)
#define HEIGHT       (PANEL_HEIGHT)

#define ZOOM      2

// DO NOT CHANGE THESE
#define ROWS          4
#define PANEL_WIDTH  32
#define PANEL_HEIGHT 16
#define PIXELS_PER_PANEL_PER_ROW      (PANEL_WIDTH*PANEL_HEIGHT/ROWS)
#define BYTES_PER_PANEL_PER_ROW       (PIXELS_PER_PANEL_PER_ROW/8)
#define BYTES_PER_LINE                (WIDTH/8)
#define SCREENBUF                     (BYTES_PER_LINE*HEIGHT)
#define VWIDTH    (WIDTH/ZOOM)
#define VHEIGHT   (HEIGHT/ZOOM)

#define M(x) (1<<((x)&7))

word incomingAudio, previncomingAudio;

#include <TimerOne.h>
#include <avr/pgmspace.h>
#include "text.h"


__attribute__((always_inline))
void enable(bool oe) {
  asm volatile
  (
    "cbi  %[port], %[pinoe]"          "\n\t"
    "cpse %[oe], 0"                   "\n\t"
    "sbi  %[port], %[pinoe]"          "\n\t"
  ::
    [oe] "r" (oe),
    [port] "I" ( _SFR_IO_ADDR(PORTD) ),
    [pinoe] "I" ( PIN_OE )
  );
  //digitalWrite(PIN_OE, oe);
}

#define DELAY //"rjmp .+0\n\t"


__attribute__((always_inline))
void set_row(byte row) {
  asm volatile
  (
    "cbi  %[port], %[a]"             "\n\t"
    "sbrc %[row], 0"                   "\n\t"
    "sbi  %[port], %[a]"             "\n\t"
    "cbi  %[port], %[b]"             "\n\t"
    "sbrc %[row], 1"                   "\n\t"
    "sbi  %[port], %[b]"             "\n\t"
    "cbi  %[port], %[sclk]"           "\n\t"
    DELAY
    "sbi  %[port], %[sclk]"           "\n\t"
  ::
    [row] "r" (row),
    [port] "I" ( _SFR_IO_ADDR(PORTD) ),
    [sclk] "I" ( PIN_SCLK ),
    [a] "I" ( PIN_A ),
    [b] "I" ( PIN_B )
  );
  //digitalWrite(PIN_A, row & 1);
  //digitalWrite(PIN_B, row & 2);
  //digitalWrite(PIN_SCLK, LOW);
  //delayMicroseconds(2);
  //digitalWrite(PIN_SCLK, HIGH);
}

__attribute__((noinline))
void pixels(byte p) {
  asm volatile
  (
    "cbi  %[port], %[r]"             "\n\t"
    "sbrs %[p], 0"                   "\n\t"
    "sbi  %[port], %[r]"             "\n\t"
    "cbi  %[port], %[clk]"           "\n\t"
    "sbi  %[port], %[clk]"           "\n\t"
    DELAY

    "cbi  %[port], %[r]"             "\n\t"
    "sbrs %[p], 1"                   "\n\t"
    "sbi  %[port], %[r]"             "\n\t"
    "cbi  %[port], %[clk]"           "\n\t"
    "sbi  %[port], %[clk]"           "\n\t"
    DELAY

    "cbi  %[port], %[r]"             "\n\t"
    "sbrs %[p], 2"                   "\n\t"
    "sbi  %[port], %[r]"             "\n\t"
    "cbi  %[port], %[clk]"           "\n\t"
    "sbi  %[port], %[clk]"           "\n\t"
    DELAY

    "cbi  %[port], %[r]"             "\n\t"
    "sbrs %[p], 3"                   "\n\t"
    "sbi  %[port], %[r]"             "\n\t"
    "cbi  %[port], %[clk]"           "\n\t"
    "sbi  %[port], %[clk]"           "\n\t"
    DELAY

    "cbi  %[port], %[r]"             "\n\t"
    "sbrs %[p], 4"                   "\n\t"
    "sbi  %[port], %[r]"             "\n\t"
    "cbi  %[port], %[clk]"           "\n\t"
    "sbi  %[port], %[clk]"           "\n\t"
    DELAY

    "cbi  %[port], %[r]"             "\n\t"
    "sbrs %[p], 5"                   "\n\t"
    "sbi  %[port], %[r]"             "\n\t"
    "cbi  %[port], %[clk]"           "\n\t"
    "sbi  %[port], %[clk]"           "\n\t"
    DELAY

    "cbi  %[port], %[r]"             "\n\t"
    "sbrs %[p], 6"                   "\n\t"
    "sbi  %[port], %[r]"             "\n\t"
    "cbi  %[port], %[clk]"           "\n\t"
    "sbi  %[port], %[clk]"           "\n\t"
    DELAY

    "cbi  %[port], %[r]"             "\n\t"
    "sbrs %[p], 7"                   "\n\t"
    "sbi  %[port], %[r]"             "\n\t"
    "cbi  %[port], %[clk]"           "\n\t"
    "sbi  %[port], %[clk]"           "\n\t"
    DELAY
  ::
    [p] "r" (p),
    [port] "I" ( _SFR_IO_ADDR(PORTD) ),
    [clk] "I" ( PIN_CLK ),
    [r] "I" ( PIN_R )
  ); 
  //digitalWrite(PIN_R, p);
  //digitalWrite(PIN_CLK, LOW);
  //digitalWrite(PIN_CLK, HIGH);
}

__attribute__((always_inline))
void pixel(bool p) {
  asm volatile
  (
    "cbi  %[port], %[r]"             "\n\t"
    "cpse %[p], 0"                   "\n\t"
    "sbi  %[port], %[r]"             "\n\t"
    "cbi  %[port], %[clk]"           "\n\t"
    //"nop"                            "\n\t"
    "sbi  %[port], %[clk]"           "\n\t"
  ::
    [p] "r" (p),
    [port] "I" ( _SFR_IO_ADDR(PORTD) ),
    [clk] "I" ( PIN_CLK ),
    [r] "I" ( PIN_R )
  ); 
  //digitalWrite(PIN_R, p);
  //digitalWrite(PIN_CLK, LOW);
  //digitalWrite(PIN_CLK, HIGH);
}

byte screenbuf[SCREENBUF] = {};      // LSB
#if SHADES_OF_GRAY
byte screenbuf1[SCREENBUF] = {};     // MSB
#endif

__attribute__((always_inline))
word getrow(word x, byte y) {
  byte panel = x / PANEL_WIDTH;
  byte xx = (x>>1)&0xC;
  byte row = y&3;
  byte yy = 3 - (y>>2);
  return BYTES_PER_PANEL_PER_ROW * (PANELS * row + panel) + yy + xx;
}


__attribute__((always_inline))
void _setpixel(word x, byte y) {
  word w = getrow(x, y);
  screenbuf[w] |= M(x);
#if SHADES_OF_GRAY
  screenbuf1[w] |= M(x);
#endif
}

__attribute__((always_inline))
void setpixel(word x, byte y) {
  _setpixel(x*ZOOM  , y*ZOOM);
#if ZOOM == 2
  _setpixel(x*ZOOM+1, y*ZOOM);
  _setpixel(x*ZOOM  , y*ZOOM+1);
  _setpixel(x*ZOOM+1, y*ZOOM+1);
#endif
}

#if SHADES_OF_GRAY
__attribute__((always_inline))
void setpixel1(word x, byte y) {
  word w = getrow(x, y);
  screenbuf1[w] |= M(x);
}
#endif

__attribute__((always_inline))
void _clearpixel(word x, byte y) {
  word w = getrow(x, y);
  screenbuf[w] &= ~M(x);
#if SHADES_OF_GRAY
  screenbuf1[w] &= ~M(x);
#endif
}

__attribute__((always_inline))
void clearpixel(word x, byte y) {
  _clearpixel(x*ZOOM  , y*ZOOM);
#if ZOOM == 2
  _clearpixel(x*ZOOM+1, y*ZOOM);
  _clearpixel(x*ZOOM  , y*ZOOM+1);
  _clearpixel(x*ZOOM+1, y*ZOOM+1);
#endif
}

void fire();

void setup() {
  Serial.begin(115200);
  
#if PIN_AUDIO
  cli();
  ADCSRA = 0;
  ADCSRB = 0;
  ADMUX |= (1<<REFS0);
  ADMUX |= (1<<ADLAR);
  
  ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);  // 128 div = ~125kHz
  ADCSRA |= (1<<ADATE);
  ADCSRA |= (1<<ADIE);
  ADCSRA |= (1<<ADEN);
  ADCSRA |= (1<<ADSC);
  sei();
#endif
  
  pinMode(PIN_OE, OUTPUT);
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_R, OUTPUT);

  pinMode(LED, OUTPUT);

  digitalWrite(PIN_SCLK, HIGH);
  digitalWrite(PIN_CLK, HIGH);
  digitalWrite(PIN_OE, HIGH);
 
  Timer1.initialize(5000);
  Timer1.attachInterrupt(&show0);
  //timer.start();

  //for (int t=0;t<SCREENBUF;++t)
  //  screenbuf[t] = 0xff;
/*  
  word p=100;
  for(char y=HEIGHT-1;y>=0;--y) {
    for (int x=WIDTH-1;x>=0;--x) {
      if (pgm_read_byte(japanese_raw + p))
        setpixel(x,y);
      p+=2;
    }
  }*/
}

/*
void set_pixel(word x, word y) {
  byte panel = x / PIXELS_PER_ROW;
  byte row = y&3;
  byte yy = y>>2;
  screenbuf[panel>>3] |= 1<<(panel&7);
}
*/

#define PONG_PADLEN 5

void pong_drawPad(word x, byte y, bool set) {
  char f = y - PONG_PADLEN/2;
  if (f<0) f=0;
  else if (f > HEIGHT-PONG_PADLEN) f = HEIGHT-PONG_PADLEN;
  
  byte e = f + PONG_PADLEN;
  if (set) {
    byte m = M(x);
    while (f<e)
      screenbuf[getrow(x, f++)] |= m;  
  }
  else {
    byte m = ~M(x);
    while (f<e)
      screenbuf[getrow(x, f++)] &= m;  
  }  
}

// returns values from 1 to 255 inclusive, period is 255
static uint8_t xorshift8() {
    static uint8_t y8 = 1;
    y8 ^= (y8 << 2);
    y8 ^= (y8 >> 1);
    return y8 ^= (y8 << 1);
}

static uint16_t xorshift16() {
    static uint16_t y16 = 1;
    y16 ^= (y16 << 2);
    y16 ^= (y16 >> 5);
    return y16 ^= (y16 << 1);
}

// returns -1, 0 or 1
static char rand1() {
  byte b = xorshift8();
  return (b&1) - ((b>>1)&1);
}


void show(byte *sb, word d) {
  word z=0;
  for (byte row = 0; row < ROWS; ++row) {
    for (byte y = 0; y < PANELS*PANEL_HEIGHT/ROWS; ++y) {
        // Write PANEL_WIDTH pixels:
        pixels(sb[z++]);
        pixels(sb[z++]);
        pixels(sb[z++]);
        pixels(sb[z++]);
    }

    set_row(row);    
    
    enable(true);
    delayMicroseconds(d);
    enable(false);
  }//row  
}

/*
  00  0
  10  1
  01  2
  11  3
*/

void pong() {
  #define SUB 2
  
  static int yy = 10;
  static int xx = 10;
  static char dx = SUB;
  static char dy = 1;
  
  static int p1y=8;
  static int p2y=8;
  
  pong_drawPad(0, p1y, false);  
  pong_drawPad(WIDTH-1, p2y, false);  
  p1y = yy/SUB;
  p2y = yy/SUB;
  pong_drawPad(0, p1y, true);  
  pong_drawPad(WIDTH-1, p2y, true); 

  clearpixel(xx/SUB, yy/SUB);
  xx += dx;
  yy += dy;
  if (xx < 0) { xx = -xx; dx = -dx; dy += rand1(); }
  else if (xx >= WIDTH*SUB) { xx = (WIDTH*SUB-1)*2 - xx; dx = -dx; dy += rand1(); }
  if (yy < 0) { yy = -yy; dy = -dy; }
  else if (yy >= HEIGHT*SUB) { yy = (HEIGHT*SUB-1)*2 - yy; dy = -dy; }
  setpixel(xx/SUB, yy/SUB);
}


__attribute__((always_inline))
byte getpixel(word x, byte y) {
  word w = getrow(x*ZOOM,y*ZOOM);
  byte m = M(x*ZOOM);
  return
#if SHADES_OF_GRAY
   ((!!(screenbuf1[w]&m))<<1) +
#endif
    !!(screenbuf[w]&m);
}

__attribute__((always_inline))
void _setpixel(word x, byte y, byte b) {
  word w = getrow(x, y);
  byte m = M(x);
  if (b&1)
    screenbuf[w] |= m;
  else
    screenbuf[w] &= ~m;
#if SHADES_OF_GRAY
  if (b&2)
    screenbuf1[w] |= m;
  else
    screenbuf1[w] &= ~m;
#endif
}


__attribute__((always_inline))
void setpixel(word x, byte y, byte b) {
  _setpixel(x*ZOOM  , y*ZOOM,   b);
#if ZOOM == 2
  _setpixel(x*ZOOM+1, y*ZOOM,   b);
  _setpixel(x*ZOOM  , y*ZOOM+1, b);
  _setpixel(x*ZOOM+1, y*ZOOM+1, b);
#endif
}

void fire() {
  static bool l;
  digitalWrite(LED, l = !l);

  for(char y=0; y<VHEIGHT; ++y) {
    for(word x=0; x<VWIDTH; ++x) {
      setpixel(x, y, y>>2); 
    }
  }
}

void show0() {
//  pong();
}

void snake() {
  static int px, py;
  static bool hascookie = false;
  if (!hascookie) {
    // no cookie; make one at a random position
    px = xorshift8() % VWIDTH;
    py = xorshift8() % VHEIGHT;
 #if VWIDTH > 256
    // center the cookie in the middle
    px += (VWIDTH - 256)/2;
 #endif
    setpixel(px, py);
    hascookie = true;
  }
  
  static int x, y;
  static char dx=1, dy;

  static int first, last;
  static struct { word x:12, y:4; } history[SNAKE_HISTORY];
  
  // random command for demo mode 
  char cmd = xorshift16();
 
  if (incomingAudio > 100)
    incomingAudio = ',';
   
  if (Serial.available())
  {
    cmd = Serial.read();
  }
  
  // with a little help from my friends
  if ((x == px || y == py) && !(x+dx==px || y+dy==py)) cmd = '.';
  
  char o;
  switch (cmd)
  {
  //case '<':
  case ',':
    if (x-dy<0 || x-dy>=VWIDTH || y+dx<0 || y+dx>=VHEIGHT) break;
   // right: 0,1 => -1,0; 0,-1 => 1,0; 1,0 => 0,1; -1,0 => 0,-1
    o=dx; dx=-dy; dy=o;
    break;
  //case '>':
  case '.': 
    // left: 0,1 => 1,0; 0,-1 => -1,0; 1,0 => 0,-1; -1,0 => 0,1
    if (x+dy<0 || x+dy>=VWIDTH || y-dx<0 || y-dx>=VHEIGHT) break;
    o=dx; dx=dy; dy=-o;
    break;
/*  
  case 'w': dx=0; dy=1; break;
  case 's': dx=0; dy=-1; break;
  case 'a': dx=1; dy=0; break;
  case 'd': dx=-1; dy=0; break;
  case 'q': if (--first<0) first=H-1; break;
*/
  }
    
  if (first != last)
  {
    // clear the old pixel
    clearpixel(history[first].x, history[first].y);
    if (++first == SNAKE_HISTORY)
      first = 0;
    // keep the old pixel if the snake is too short
    if (first == 1 && last < 15)
      first = 0;
  }

  x += dx;
  y += dy;  
  
  // force a left or right turn when running into the edge
  if (x<0) { x=0; dx=0; if (y<VHEIGHT/2) dy=1; else dy=-1; y+=dy; }
  else if (x>=VWIDTH) { x = VWIDTH-1; dx=0; if (y<VHEIGHT/2) dy=1; else dy=-1; y+=dy; }
  
  if (y<0) { y=0; dy=0; if (x<VWIDTH/2) dx=1; else dx=-1; x+=dx; }
  else if (y>=VHEIGHT) {y = VHEIGHT-1; dy=0; if (x<VWIDTH/2) dx=1; else dx=-1; x+=dx; }

  // is the pixel already set?  
  if (getpixel(x,y)) {
    // did we eat the cookie?
    if (x == px && y == py) {
      // keep one more pixel (the last one) in the history
      if (--first<0) first=SNAKE_HISTORY-1;
      // create a new cookie
      hascookie = false;
    }
    else {
      // delete the whole snake
      while (first != last) {
        // clear the old pixel
        clearpixel(history[first].x, history[first].y);
        if (++first == SNAKE_HISTORY)
          first = 0;
        // update the screen (animate the deletion)
        for (int t=0; t<10; ++t)
          show(screenbuf, 1000);
      }
    }
  }
  
  // Set the new pixel
  setpixel(x, y);
  
  // track history for the snake points
  history[last].x = x;
  history[last].y = y;
  if (++last == SNAKE_HISTORY) last=0;
}


void loop() {
/*  
  #define SUB 64
  static int mx;
  static int my;
  
  static char dx=100;
  static char dy=50;
  static const char adx=0;
  static const char ady=-1;
  static bool side;
  
  dx += adx;
  dy += ady;
  
  if (my < SUB*HEIGHT) {
    clearpixel(mx/SUB, my/SUB);
  }
  mx += dx;
  my += dy;
  if (my <= 0 || mx > WIDTH*SUB || mx < 0) {
    if (my <= 0) setpixel(mx/SUB, 0);
    side = !side;
    mx = 0;
    dx = xorshift8()>>1;
    if (side) {
      dx = -dx;
      mx = WIDTH*SUB-1;
    }
    my = 0;
    dy = xorshift8()>>1;
  }
  else if (my < SUB*HEIGHT) { 
    if (side)
      setpixel(mx/SUB, my/SUB);
    else
      setpixel1(mx/SUB, my/SUB);
  }*/
  
  //memset(screenbuf, 0, SCREENBUF);
  //for (int s=0; s<SCREENBUF; ++s)
  //    screenbuf[s] = xorshift8();  
  //for (word x=0; x<WIDTH; ++x)
    //setpixel(x, xorshift8()&15);

  //pong();
  snake();

  // update the screenbuffer
  show(screenbuf, 1000);
#if SHADES_OF_GRAY
  show(screenbuf1, 20);
#endif

  //Serial.println(incomingAudio);
}


ISR(ADC_vect) {
  previncomingAudio = incomingAudio;
  incomingAudio = ADCH + previncomingAudio;    // pmf
  //incomingAudio += previncomingAudio;
}

