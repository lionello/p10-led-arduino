#define PIN_OE    2
#define PIN_A     3
#define PIN_B     4
#define PIN_CLK   5
#define PIN_SCLK  6
#define PIN_R     7
#define LED      13

#include <avr/pgmspace.h>
#include "text.h"

#define PANELS       11
#define WIDTH        (PANEL_WIDTH*PANELS)
#define HEIGHT       (PANEL_HEIGHT)

// DO NOT CHANGE THESE
#define ROWS          4
#define PANEL_WIDTH  32
#define PANEL_HEIGHT 16
#define PIXELS_PER_PANEL_PER_ROW      (PANEL_WIDTH*PANEL_HEIGHT/ROWS)
#define BYTES_PER_PANEL_PER_ROW       (PIXELS_PER_PANEL_PER_ROW/8)
#define BYTES_PER_LINE                (WIDTH/8)
#define SCREENBUF                     (BYTES_PER_LINE*HEIGHT)

#define M(x) (1<<((x)&7))

#include <TimerOne.h>

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
byte screenbuf1[SCREENBUF] = {};     // MSB

__attribute__((always_inline))
word getrow(word x, byte y) {
  byte panel = x / PANEL_WIDTH;
  byte xx = (x>>1)&0xC;
  byte row = y&3;
  byte yy = 3 - (y>>2);
  return BYTES_PER_PANEL_PER_ROW * (PANELS * row + panel) + yy + xx;
}


__attribute__((always_inline))
void setpixel(word x, byte y) {
  word w = getrow(x, y);
  screenbuf[w] |= M(x);
  screenbuf1[w] |= M(x);
}

__attribute__((always_inline))
void setpixel1(word x, byte y) {
  word w = getrow(x, y);
  screenbuf1[w] |= M(x);
}

__attribute__((always_inline))
void clearpixel(word x, byte y) {
  word w = getrow(x, y);
  screenbuf[w] &= ~M(x);
  screenbuf1[w] &= ~M(x);
}

void fire();

void setup() {
  Serial.begin(115200);
  
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
  word w = getrow(x,y);
  byte m = M(x);
  return !!(screenbuf[w]&m) + ((!!(screenbuf1[w]&m))<<1);
}
__attribute__((always_inline))
void setpixel(word x, byte y, byte b) {
  word w = getrow(x, y);
  byte m = M(x);
  if (b&1)
    screenbuf[w] |= m;
  else
    screenbuf[w] &= ~m;
  if (b&2)
    screenbuf1[w] |= m;
  else
    screenbuf1[w] &= ~m;
}

void fire() {
  static bool l;
  digitalWrite(LED, l = !l);

  for(char y=0; y<HEIGHT; ++y) {
    for(word x=0; x<WIDTH; ++x) {
      setpixel(x, y, y>>2); 
    }
  }
}

void show0() {
//  pong();
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

  static int px, py;
  static bool hascookie = false;
  if (!hascookie) {
    px = xorshift8()+50;
    py = xorshift8()&15;
    setpixel(px, py);
    hascookie = true;
  }
  
  static int x, y;
  static char dx, dy;

  static int first, last;
  struct P { word x,y;};
  const int H=100;
  static P history[H];
  
  if (first!=last)
  {
    clearpixel(history[first].x,history[first].y);
    if (++first == H)
      first = 0;
  }

  x += dx;
  y += dy;  
  
  if (x<0) { x=0; dx=0; if (y<HEIGHT/2) dy=1; else dy=-1; y+=dy; }
  else if (x>=WIDTH) { x = WIDTH-1; dx=0; if (y<HEIGHT/2) dy=1; else dy=-1; y+=dy; }
  
  if (y<0) { y=0; dy=0; if (x<WIDTH/2) dx=1; else dx=-1; x+=dx; }
  else if (y>=HEIGHT) {y = HEIGHT-1; dy=0; if (x<WIDTH/2) dx=1; else dx=-1; x+=dx; }

  history[last].x = x;
  history[last].y = y;
  if (++last == H) last=0;
  setpixel(x, y);
  
  if (x == px && y == py) {
    if (--first<0) first=H-1;
    hascookie = false;
  }
  
  if (Serial.available())
  {
    switch (Serial.read())
    {
    case 'w': dx=0; dy=1; break;
    case 's': dx=0; dy=-1; break;
    case 'a': dx=1; dy=0; break;
    case 'd': dx=-1; dy=0; break;
    case 'q': if (--first<0) first=H-1; break;
    }
  }
  
  show(screenbuf, 20);
  show(screenbuf1, 1000);
}

