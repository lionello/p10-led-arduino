# p10-led-arduino

Arduino project that shows how to drive a P10 LED screen in different ways. The code shows several techniques:

* Bit-banging the P10 shift registers using inline assembly
* Using Timer1 to drive the display versus "loop()"
* Different mini game implementations: Snake, Pong
* 64-byte sine/cosine table

This particular code was made to run on an Arduino UNO, attached to a 11 panel P10 LED banner, resulting in a 352x16 LED screen.

Some of the copyrighted assets were removed from the repo, but you can use the content from `text.h` as a template.

