#if 0

/*********************************************************************Code*******
 *   $Id:: ssptest.c 6098 2011-01-08 02:26:20Z nxp12832                     $
 *   Project: NXP LPC17xx SSP example
 *
 *   Description:
 *     This file contains SSP test modules, main entry, to 2-D implement vector graphics.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#include <cr_section_macros.h>
#include <NXP/crp.h>


// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

#include "LPC17xx.h"                        /* LPC17xx definitions */
#include "ssp.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "font.h"
#include "SJSU.h"

//Colours
#define GREEN 0x006400
#define BLACK 0x000000
#define RED 0xFF0000
#define BLUE 0x000080
#define WHITE 0xFFFFFF
#define PINK 0xFFC0CB
#define PURPLE 0x800080
#define YELLOW 0xFFFF00
#define LIME 0x00FF00
#define MAGENTA 0xFF00FF
#define CYAN 0x00FFFF
#define SILVER 0xC0C0C0
#define GREY 0x808080
#define ORANGE 0xFFA500
#define BROWN 0xA52A2A
#define MAROON 0x800000
//Axes



/* Be careful with the port number and location number, because
some of the location may not exist in that port. */
#define PORT_NUM            1
#define LOCATION_NUM        0

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

uint8_t src_addr[SSP_BUFSIZE];
uint8_t dest_addr[SSP_BUFSIZE];
int colstart = 0;
int rowstart = 0;

/*****************************************************************************
** Function name:       LCD_TEST
**
** Descriptions:        2-d vector graphics- line segment, screen saver, forest
**
** parameters:            None
** Returned value:        None
**
*****************************************************************************/

//LCD
#define ST7735_TFTWIDTH  127 //LCD width
#define ST7735_TFTHEIGHT 159 //LCD height
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define swap(x, y) { x = x + y; y = x - y; x = x - y; }

//color code
#define GREEN 0x00FF00
#define BLACK  0x000000
#define RED  0xFF0000
#define BLUE  0x0000FF
#define WHITE  0xFFFFFF
#define CYAN    0x00FFFF
#define PURPLE 0x8E388E
#define YELLOW  0xFFD700
#define ORANGE 0xFF8000

int _height = ST7735_TFTHEIGHT;
int _width = ST7735_TFTWIDTH;
int cursor_x = 0, cursor_y = 0;
//uint16_t textcolor = RED, textbgcolor= GREEN;
float textsize = 2;
int wrap = 1;

//for writing data into the SPI
void spiwrite(uint8_t c)
{
    int portnum = 0;
    src_addr[0] = c;
    SSP_SSELToggle( portnum, 0 );
    SSPSend( portnum, (uint8_t *)src_addr, 1 );
    SSP_SSELToggle( portnum, 1 );
}
//writing commands into SPI
void writecommand(uint8_t c) {
    LPC_GPIO0->FIOCLR |= (0x1<<21);
    spiwrite(c);
}
//making LCD ready to write data
void writedata(uint8_t c) {

    LPC_GPIO0->FIOSET |= (0x1<<21);
    spiwrite(c);
}
//writing data to the LCD
void writeword(uint16_t c) {

    uint8_t d;
    d = c >> 8;
    writedata(d);
    d = c & 0xFF;
    writedata(d);
}
//write colour
void write888(uint32_t color, uint32_t repeat) {
    uint8_t red, green, blue;
    int i;
    red = (color >> 16);
    green = (color >> 8) & 0xFF;
    blue = color & 0xFF;
    for (i = 0; i< repeat; i++) {
        writedata(red);
        writedata(green);
        writedata(blue);
    }
}

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
                    uint16_t y1) {

      writecommand(ST7735_CASET);
      writeword(x0);
      writeword(x1);
      writecommand(ST7735_RASET);
      writeword(y0);
      writeword(y1);

}
void drawPixel(int16_t x, int16_t y, uint32_t color) {
    if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

    setAddrWindow(x,y,x+1,y+1);
    writecommand(ST7735_RAMWR);
    write888(color, 1);
}

//drawLetters()

void HLine(int16_t x0,int16_t x1,int16_t y,uint16_t color){
    width = x1-x0+1;
    setAddrWindow(x0,y,x1,y);
    writecommand(ST7735_RAMWR);
    write888(color,width);
}
void VLine(int16_t x,int16_t y0,int16_t y1,uint16_t color){
    width = y1-y0+1;
    setAddrWindow(x,y0,x,y1);
    writecommand(ST7735_RAMWR);
    write888(color,width);
}

void drawCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color)
{
      int16_t f = 1 - r;
      int16_t ddF_x = 1;
      int16_t ddF_y = -2 * r;
      int16_t x = 0;
      int16_t y = r;
      drawPixel(x0 , y0+r, color);
      drawPixel(x0 , y0-r, color);
      drawPixel(x0+r, y0 , color);
      drawPixel(x0-r, y0 , color);
      while (x<y)
      {
      if (f >= 0)
      {
      y--;
      ddF_y += 2;
      f += ddF_y;
      }
      x++;
      ddF_x += 2;
      f += ddF_x;
      drawPixel(x0 + x, y0 + y, color);
      drawPixel(x0 - x, y0 + y, color);
      drawPixel(x0 + x, y0 - y, color);
      drawPixel(x0 - x, y0 - y, color);
      drawPixel(x0 + y, y0 + x, color);
      drawPixel(x0 - y, y0 + x, color);
      drawPixel(x0 + y, y0 - x, color);
      drawPixel(x0 - y, y0 - x, color);
      }
}

//Initialize LCD
void lcd_init()
{
/*
 * portnum     = 0 ;
 * cs         = p0.16 / p0.6 ?
 * rs        = p0.21
 * rst        = p0.22
 */
    uint32_t portnum = 0;
    int i;
    printf("LCD initialized\n");
    /* Notice the hack, for portnum 0 p0.16 is used */
    if ( portnum == 0 )
      {
        LPC_GPIO0->FIODIR |= (0x1<<16);        /* SSP1, P0.16 defined as Outputs */
      }
      else
      {
        LPC_GPIO0->FIODIR |= (0x1<<6);        /* SSP0 P0.6 defined as Outputs */
      }
    /* Set rs(dc) and rst as outputs */
    LPC_GPIO0->FIODIR |= (0x1<<21);        /* rs/dc P0.21 defined as Outputs */
    LPC_GPIO0->FIODIR |= (0x1<<22);        /* rst P0.22 defined as Outputs */


    /* Reset sequence */
    LPC_GPIO0->FIOSET |= (0x1<<22);

    for(i = 0; i < 10000000; i++);//lcddelay(500);                        /*delay 500 ms */
    LPC_GPIO0->FIOCLR |= (0x1<<22);
    for(i = 0; i < 10000000; i++);//lcddelay(500);                        /*delay 500 ms */
    LPC_GPIO0->FIOSET |= (0x1<<22);
    for(i = 0; i < 10000000; i++);//lcddelay(500);                        /*delay 500 ms */

    //ask?
     for ( i = 0; i < SSP_BUFSIZE; i++ )    /* Init RD and WR buffer */
        {
            src_addr[i] = 0;
            dest_addr[i] = 0;
        }

     /* do we need Sw reset (cmd 0x01) ? */

     /* Sleep out */
     SSP_SSELToggle( portnum, 0 );
     src_addr[0] = 0x11;    /* Sleep out */
     SSPSend( portnum, (uint8_t *)src_addr, 1 );
     SSP_SSELToggle( portnum, 1 );

     for(i = 0; i < 10000000; i++);//lcddelay(500);                        /*delay 500 ms */

    /* delay 200 ms */
    /* Disp on */
     SSP_SSELToggle( portnum, 0 );
     src_addr[0] = 0x29;    /* Disp On */
     SSPSend( portnum, (uint8_t *)src_addr, 1 );
     SSP_SSELToggle( portnum, 1 );
    /* delay 200 ms */
     for(i = 0; i < 1000000; i++);//lcddelay(500);                        /*delay 500 ms */
}

//Draw line function
void drawline(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint32_t color) {
      int16_t slope = abs(y1 - y0) > abs(x1 - x0);
      if (slope) {
      swap(x0, y0);
      swap(x1, y1);
      }

      if (x0 > x1) {
      swap(x0, x1);
      swap(y0, y1);
      }

      int16_t dx, dy;
      dx = x1 - x0;
      dy = abs(y1 - y0);

      int16_t err = dx / 2;
      int16_t ystep;

      if (y0 < y1) {
      ystep = 1;
      } else {
      ystep = -1;
      }

      for (; x0<=x1; x0++) {
      if (slope) {
      drawPixel(y0, x0, color);
      } else {
      drawPixel(x0, y0, color);
      }
      err -= dy;
      if (err < 0) {
      y0 += ystep;
      err += dx;
      }
      }
}

void makeBackGroundBlack()
{
      int i, j;
      for(i = 0; i < ST7735_TFTWIDTH; i++)
      for(j = 0; j < ST7735_TFTHEIGHT; j++)
      drawPixel(i, j, 0x00);
}

void drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color)
{
      int16_t i, j, byteWidth = (w + 7) / 8;
      for(j=0; j<h; j++)
      {
      for(i=0; i<w; i++ )
      {
      if(pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7)))
      {
      drawPixel(x+i, y+j, color);
      }
      }
      }
}

void write(uint8_t c, uint8_t textSize, uint16_t textbgcolor, uint16_t textcolor)
{
      uint8_t wrap = 1;
      if (c == '\n')
      {
      cursor_y += textSize*8;
      cursor_x = 0;
      }
      else if (c == '\r')
      {
      // skip em
      }
      else
      {
      drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textSize);
      cursor_x += textSize*6;
      if (wrap && (cursor_x > (100 - textSize*6)))
      {
      cursor_y += textSize*8;
      cursor_x = 0;
      }
      }

}

// Draw a character
void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size)
{
      int8_t i, j;
      if((x >= _width) || // Clip right
      (y >= _height) || // Clip bottom
      ((x + 6 * size - 1) < 0) || // Clip left
      ((y + 8 * size - 1) < 0)) // Clip top
      return;
      for (i=0; i<6; i++ ) {
      uint8_t line;
      if (i == 5)
      line = 0x0;
      else
      line = pgm_read_byte(font+(c*5)+i);
      for (j = 0; j<8; j++) {
      if (line & 0x1) {
      if (size == 1) // default size
      drawPixel(x+i, y+j, color);
      else { // big size
      fillRect(x+(i*size), y+(j*size), size, size, color);
      }
      } else if (bg != color) {
      if (size == 1) // default size
      drawPixel(x+i, y+j, bg);
      else { // big size
      fillRect(x+i*size, y+j*size, size, size, bg);
      }
      }
      line >>= 1;
      }
      }
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint32_t color)
{
      int16_t i;
      // Update in subclasses if desired!
      for (i=x; i<x+w; i++) {
      drawFastVLine(i, y, h, color);
      }
}



void drawFastVLine(int16_t x, int16_t y, int16_t h, uint32_t color)
{
      // Update in subclasses if desired!
      drawline(x, y, x, y+h-1, color);
}


struct Vector2i {
    int x, y;
};

struct Vector3i {
    int x, y, z;
};

struct Vector2i Point(struct Vector3i P ){
      int viewPoint[]={90,90,90};
      int pita= acosf(viewPoint[0]/(sqrt(pow(viewPoint[0],2)+pow(viewPoint[1],2))));
      int phi=acosf(viewPoint[2]/(sqrt(pow(viewPoint[0],2)+pow(viewPoint[1],2)+pow(viewPoint[2],2))));
      struct Vector3i CameraPoint;
      CameraPoint.x=-sin(pita)*P.x+ cos(pita)*P.y;
      CameraPoint.y=-cos(pita)*cos(phi)*P.x- sin(pita)*cos(phi)*P.y+sin(phi)*P.z;
      CameraPoint.z=-cos(pita)*sin(phi)*P.x- cos(pita)*sin(phi)*P.y-cos(phi)*P.z+(sqrt(pow(viewPoint[0],2)+pow(viewPoint[1],2)+pow(viewPoint[2],2)));
      //sqrt(pow(P.x,2)+pow(P.y,2)+pow(P.z,2));
      struct Vector2i derivedPoint;
      derivedPoint.x=((CameraPoint.x*120 ) / CameraPoint.z);
      derivedPoint.y=((CameraPoint.y *120 )/ CameraPoint.z)+140;
       return derivedPoint;
}


/*float Shade(struct Vector3i P ){
      int viewPoint[]={90,60,10};
      float lamda=-P.z/viewPoint[2]-P.z;
      return lamda;
}*/



void DrawScreenSaver(){
      int baseCoOrdinates[4][3]={{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
      int newCoOrdinates[4][3]={{0,0},{0,0},{0,0},{0,0}};
      uint32_t *color[9] ={0xFF0000,0x003399,0x00CC00,0x9933FF,0x00FFFF,0xFF6600,0xFF3399,0x006666,0x80000};
      int numSavers=0;
      while(numSavers<10){
      int intSquareSize=rand()%(20)+10;
      int x=rand()%(20)+50;
      int y=50;
      int z=rand()%(25)+0;
      baseCoOrdinates[0][0]=x;
      baseCoOrdinates[0][1]=y;
      baseCoOrdinates[0][2]=z;

      baseCoOrdinates[1][0]=x;
      baseCoOrdinates[1][1]=y;
      baseCoOrdinates[1][2]=z+intSquareSize;

      baseCoOrdinates[2][0]=x+intSquareSize;
      baseCoOrdinates[2][1]=y;
      baseCoOrdinates[2][2]=z+intSquareSize;

      baseCoOrdinates[3][0]=x+intSquareSize;
      baseCoOrdinates[3][1]=y;
      baseCoOrdinates[3][2]=z;

      int intColor=rand()%(8)+0;
      int saverItrations=0;
      while(saverItrations<10){
      int i=0;

      while(i<4){

      if(i==3){

      struct Vector3i P;
      P.x=baseCoOrdinates[i][0];
      P.y=baseCoOrdinates[i][1];
      P.z=baseCoOrdinates[i][2];
      struct Vector3i P1;
      P1.x=baseCoOrdinates[0][0];
      P1.y=baseCoOrdinates[0][1];
      P1.z=baseCoOrdinates[0][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,color[intColor]);
      newCoOrdinates[i][0]=baseCoOrdinates[i][0]+0.2*(baseCoOrdinates[0][0]-baseCoOrdinates[i][0]);
      newCoOrdinates[i][1]=baseCoOrdinates[i][1]+0.2*(baseCoOrdinates[0][1]-baseCoOrdinates[i][1]);
      newCoOrdinates[i][2]=baseCoOrdinates[i][2]+0.2*(baseCoOrdinates[0][2]-baseCoOrdinates[i][2]);
      }
      else{
      struct Vector3i P;
      P.x=baseCoOrdinates[i][0];
      P.y=baseCoOrdinates[i][1];
      P.z=baseCoOrdinates[i][2];
      struct Vector3i P1;
      P1.x=baseCoOrdinates[i+1][0];
      P1.y=baseCoOrdinates[i+1][1];
      P1.z=baseCoOrdinates[i+1][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,color[intColor]);
      newCoOrdinates[i][0]=baseCoOrdinates[i][0]+0.2*(baseCoOrdinates[i+1][0]-baseCoOrdinates[i][0]);
      newCoOrdinates[i][1]=baseCoOrdinates[i][1]+0.2*(baseCoOrdinates[i+1][1]-baseCoOrdinates[i][1]);
      newCoOrdinates[i][2]=baseCoOrdinates[i][2]+0.2*(baseCoOrdinates[i+1][2]-baseCoOrdinates[i][2]);
      }
      i++;
      }

      memcpy(baseCoOrdinates, newCoOrdinates, sizeof(baseCoOrdinates));
      saverItrations++;

      }
      numSavers++;
      }

}

void DrawScreenSaverup(){
      int baseCoOrdinates[4][3]={{0,0,6},{0,0,0},{0,0,0},{0,0,0}};
      int newCoOrdinates[4][3]={{0,0},{0,0},{0,0},{0,0}};
      uint32_t *color[9] ={0xFF0000,0x003399,0x00CC00,0x9933FF,0x00FFFF,0xFF6600,0xFF3399,0x006666,0x80000};
      int numSavers=0;
      while(numSavers<10){
      int intSquareSize=rand()%(20)+10;
      int x=rand()%(40)+65;
      int y=rand()%(30)+80;
      int z=50;
      baseCoOrdinates[0][0]=x;
      baseCoOrdinates[0][1]=y;
      baseCoOrdinates[0][2]=z;

      baseCoOrdinates[1][0]=x;
      baseCoOrdinates[1][1]=y;
      baseCoOrdinates[1][2]=z+intSquareSize;

      baseCoOrdinates[2][0]=x+intSquareSize;
      baseCoOrdinates[2][1]=y;
      baseCoOrdinates[2][2]=z+intSquareSize;

      baseCoOrdinates[3][0]=x+intSquareSize;
      baseCoOrdinates[3][1]=y;
      baseCoOrdinates[3][2]=z;

      int intColor=rand()%(8)+0;
      int saverItrations=0;
      while(saverItrations<10){
      int i=0;

      while(i<4){

      if(i==3){

      struct Vector3i P;
      P.x=baseCoOrdinates[i][0];
      P.y=baseCoOrdinates[i][1];
      P.z=baseCoOrdinates[i][2];
      struct Vector3i P1;
      P1.x=baseCoOrdinates[0][0];
      P1.y=baseCoOrdinates[0][1];
      P1.z=baseCoOrdinates[0][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,color[intColor]);
      newCoOrdinates[i][0]=baseCoOrdinates[i][0]+0.2*(baseCoOrdinates[0][0]-baseCoOrdinates[i][0]);
      newCoOrdinates[i][1]=baseCoOrdinates[i][1]+0.2*(baseCoOrdinates[0][1]-baseCoOrdinates[i][1]);
      newCoOrdinates[i][2]=baseCoOrdinates[i][2]+0.2*(baseCoOrdinates[0][2]-baseCoOrdinates[i][2]);
      }
      else{
      struct Vector3i P;
      P.x=baseCoOrdinates[i][0];
      P.y=baseCoOrdinates[i][1];
      P.z=baseCoOrdinates[i][2];
      struct Vector3i P1;
      P1.x=baseCoOrdinates[i+1][0];
      P1.y=baseCoOrdinates[i+1][1];
      P1.z=baseCoOrdinates[i+1][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,color[intColor]);
      newCoOrdinates[i][0]=baseCoOrdinates[i][0]+0.2*(baseCoOrdinates[i+1][0]-baseCoOrdinates[i][0]);
      newCoOrdinates[i][1]=baseCoOrdinates[i][1]+0.2*(baseCoOrdinates[i+1][1]-baseCoOrdinates[i][1]);
      newCoOrdinates[i][2]=baseCoOrdinates[i][2]+0.2*(baseCoOrdinates[i+1][2]-baseCoOrdinates[i][2]);
      }
      i++;
      }

      memcpy(baseCoOrdinates, newCoOrdinates, sizeof(baseCoOrdinates));
      saverItrations++;

      }
      numSavers++;
      }

}

//drawing background for forest
void DrawBackgroundorForest(){
//sec2
      int itr=0;
      for(itr=0;itr<61;itr++){
      struct Vector3i P;
      P.x=50;
      P.y=50;
      P.z=itr;
      struct Vector3i P1;
      P1.x=50;
      P1.y=110;
      P1.z=itr;
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,0x003d99);
      }


//sec1
      itr=0;
      for(itr=0;itr<61;itr++){
      struct Vector3i P;
      P.x=50;
      P.y=50;
      P.z=itr;
      struct Vector3i P1;
      P1.x=110;
      P1.y=50;
      P1.z=itr;
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,0xff6600);
      }

// secup

      itr=50;
      for(itr=50;itr<110;itr++){
      struct Vector3i P;
      P.x=itr;
      P.y=110;
      P.z=60;
      struct Vector3i P1;
      P1.x=itr;
      P1.y=50;
      P1.z=60;
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,0xcc3300);
      }

}

void grow_mytree(int x0, int y0, float angle, int length, int level, int color){
//    printf("inside mytree\n");

      int x1, y1, length1;
      float angle1;

      if(level>0){
      //x-y coordinates of branch
      x1 = x0+length*cos(angle);
//    printf("%f\n",x1);
          y1 = y0+length*sin(angle);
//        printf("%f\n",y1);

          drawline(x0,y0,x1,y1,color); //tree branch

          angle1 = angle + 0.52; //deviate right->0.52 rad/30 deg
          length1 = 0.8 * length; //reduction of length by 20% of previous length
          grow_mytree(x1,y1,angle1,length1,level-1,color);

          angle1 = angle - 0.52; //deviate left->0.52 rad/30 deg
          length1 = 0.8 * length;
          grow_mytree(x1,y1,angle1,length1,level-1,color);

          angle1 = angle; //center->0 deg
          length1 = 0.8 * length;
          grow_mytree(x1,y1,angle1,length1,level-1,color);
      }
      //    printf("exiting mytree\n");
}

void DrawForest(){
       {
      int baseX=50;
      // int baseX=rand()%(10)+10;
      int baseY=rand()%(30)+70;
      int baseZ=rand()%(20)+0;
      struct Vector3i P;
      P.x=baseX;
      P.y=baseY;
      P.z=baseZ;
      struct Vector2i p2d=Point(P);

       grow_mytree(p2d.x,p2d.y,5.23,5,7,GREEN); //right branch (angle = 5.23 rad/300 deg)
      //  grow_mytree(32,150,5.23,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.18,5,7,GREEN); //left branch (angle = 4.18 rad/240 deg)
      //  grow_mytree(32,150,4.18,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.71,5,7,GREEN); //center branch (angle = 4.71 rad/0 deg)
      //  grow_mytree(32,150,4.71,30,4,GREEN);
       }

       {
      int baseX=50;
       //int baseX=rand()%(10)+10;
       int baseY=rand()%(30)+70;
      int baseZ=rand()%(20)+0;
      struct Vector3i P;
      P.x=baseX;
      P.y=baseY;
      P.z=baseZ;
      struct Vector2i p2d=Point(P);

       grow_mytree(p2d.x,p2d.y,5.23,5,7,GREEN); //right branch (angle = 5.23 rad/300 deg)
      //  grow_mytree(32,150,5.23,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.18,5,7,GREEN); //left branch (angle = 4.18 rad/240 deg)
      //  grow_mytree(32,150,4.18,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.71,5,7,GREEN); //center branch (angle = 4.71 rad/0 deg)
      //  grow_mytree(32,150,4.71,30,4,GREEN);
       }

       {
      int baseX=50;
      // int baseX=rand()%(10)+10;
            int baseY=rand()%(30)+70;
      int baseZ=rand()%(20)+0;
      struct Vector3i P;
      P.x=baseX;
      P.y=baseY;
      P.z=baseZ;
      struct Vector2i p2d=Point(P);

       grow_mytree(p2d.x,p2d.y,5.23,5,7,GREEN); //right branch (angle = 5.23 rad/300 deg)
      //  grow_mytree(32,150,5.23,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.18,5,7,GREEN); //left branch (angle = 4.18 rad/240 deg)
      //  grow_mytree(32,150,4.18,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.71,5,7,GREEN); //center branch (angle = 4.71 rad/0 deg)
      //  grow_mytree(32,150,4.71,30,4,GREEN);
       }

       {
      int baseX=50;
      // int baseX=rand()%(10)+10;
            int baseY=rand()%(30)+70;
      int baseZ=rand()%(20)+0;
      struct Vector3i P;
      P.x=baseX;
      P.y=baseY;
      P.z=baseZ;
      struct Vector2i p2d=Point(P);

       grow_mytree(p2d.x,p2d.y,5.23,5,7,GREEN); //right branch (angle = 5.23 rad/300 deg)
      //  grow_mytree(32,150,5.23,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.18,5,7,GREEN); //left branch (angle = 4.18 rad/240 deg)
      //  grow_mytree(32,150,4.18,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.71,5,7,GREEN); //center branch (angle = 4.71 rad/0 deg)
      //  grow_mytree(32,150,4.71,30,4,GREEN);
       }

}

void DrawMap(){
       int texas[][3]={{0,0,0},{80,65,60},{80,55,60},{75,60,60},{70,55,60},{75,65,60},{60,80,60},{60,95,60},{85,95,60},{85,75,60},{95,75,60}};
       int TexItr=0;
       for(TexItr=0;TexItr<11;TexItr++){
       if(TexItr<10){
      struct Vector3i P;
      P.x=texas[TexItr][0];
      P.y=texas[TexItr][1];
      P.z=texas[TexItr][2];
      struct Vector3i P1;
      P1.x=texas[TexItr+1][0];
      P1.y=texas[TexItr+1][1];
      P1.z=texas[TexItr+1][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,BLACK);
      drawPixel(p2d.x,p2d.y,BLACK);
      drawPixel(p12d.x,p12d.y,BLACK);


       }
       else{

      struct Vector3i P;
      P.x=texas[TexItr][0];
      P.y=texas[TexItr][1];
      P.z=texas[TexItr][2];
      struct Vector3i P1;
      P1.x=texas[0][0];
      P1.y=texas[0][1];
      P1.z=texas[0][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,BLACK);
       }
       }
}







/******************************************************************************
**   Main Function  main()
******************************************************************************/
int main (void)
{
       //EINTInit();
 uint32_t i, portnum = PORT_NUM;
 portnum = 0 ; /* For LCD use 1 */
 if ( portnum == 0 )
        SSP0Init();            /* initialize SSP port */
 else if ( portnum == 1 )
        SSP1Init();
 for ( i = 0; i < SSP_BUFSIZE; i++ )
 {
        src_addr[i] = (uint8_t)i;
        dest_addr[i] = 0;
 }

int x;
printf("Enter 0 if you want see only forest without cube else enter 1");
scanf("%d",&x);
while(1)
{
 if(x==0)
 {
lcd_init();
 fillRect(0,0,ST7735_TFTWIDTH,ST7735_TFTHEIGHT,WHITE);
 while(1)
 {
       {
       int baseX=rand()%(10)+10;
      int baseY=rand()%(30)+70;
      int baseZ=rand()%(20)+0;
      struct Vector3i P;
      P.x=baseX;
      P.y=baseY;
      P.z=baseZ;
      struct Vector2i p2d=Point(P);

       grow_mytree(p2d.x,p2d.y,5.23,20,7,GREEN); //right branch (angle = 5.23 rad/300 deg)
      //  grow_mytree(32,150,5.23,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.18,20,7,GREEN); //left branch (angle = 4.18 rad/240 deg)
      //  grow_mytree(32,150,4.18,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.71,20,7,GREEN); //center branch (angle = 4.71 rad/0 deg)
      //  grow_mytree(32,150,4.71,30,4,GREEN);
       }

       {
      //int baseX=50;
       int baseX=rand()%(10)+10;
       int baseY=rand()%(30)+70;
      int baseZ=rand()%(20)+0;
      struct Vector3i P;
      P.x=baseX;
      P.y=baseY;
      P.z=baseZ;
      struct Vector2i p2d=Point(P);

       grow_mytree(p2d.x,p2d.y,5.23,20,7,GREEN); //right branch (angle = 5.23 rad/300 deg)
      //  grow_mytree(32,150,5.23,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.18,20,7,GREEN); //left branch (angle = 4.18 rad/240 deg)
      //  grow_mytree(32,150,4.18,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.71,20,7,GREEN); //center branch (angle = 4.71 rad/0 deg)
      //  grow_mytree(32,150,4.71,30,4,GREEN);
       }

       {
      //int baseX=50;
       int baseX=rand()%(10)+10;
            int baseY=rand()%(30)+70;
      int baseZ=rand()%(20)+0;
      struct Vector3i P;
      P.x=baseX;
      P.y=baseY;
      P.z=baseZ;
      struct Vector2i p2d=Point(P);

       grow_mytree(p2d.x,p2d.y,5.23,20,7,GREEN); //right branch (angle = 5.23 rad/300 deg)
      //  grow_mytree(32,150,5.23,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.18,20,7,GREEN); //left branch (angle = 4.18 rad/240 deg)
      //  grow_mytree(32,150,4.18,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.71,20,7,GREEN); //center branch (angle = 4.71 rad/0 deg)
      //  grow_mytree(32,150,4.71,30,4,GREEN);
       }

       {
      //int baseX=50;
       int baseX=rand()%(10)+10;
            int baseY=rand()%(30)+70;
      int baseZ=rand()%(20)+0;
      struct Vector3i P;
      P.x=baseX;
      P.y=baseY;
      P.z=baseZ;
      struct Vector2i p2d=Point(P);

       grow_mytree(p2d.x,p2d.y,5.23,20,7,GREEN); //right branch (angle = 5.23 rad/300 deg)
      //  grow_mytree(32,150,5.23,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.18,20,7,GREEN); //left branch (angle = 4.18 rad/240 deg)
      //  grow_mytree(32,150,4.18,30,4,GREEN);
       grow_mytree(p2d.x,p2d.y,4.71,20,7,GREEN); //center branch (angle = 4.71 rad/0 deg)
      //  grow_mytree(32,150,4.71,30,4,GREEN);
       }
       int b;
        printf("Enter 0 if you want to continue else enter 1");
        scanf("%d",&b);
        if(b==1)
        {
        x=x+1;
        break;
        }
        else
        printf("still printing trees");
 }

 //fillRect(0,0,ST7735_TFTWIDTH,ST7735_TFTHEIGHT,WHITE);
 }
 else if(x==1)
 //else
 {
        //initialize LCD

 //lcd_init();
 fillRect(0,0,ST7735_TFTWIDTH,ST7735_TFTHEIGHT,WHITE);
 //cube
//int cube[][3]={{65,80,0},{115,80,0},{115,130,0},{65,130,0}};
 //int cubebase[][3]={{65,80,50},{115,80,50},{115,130,50},{65,130,50}};
 //int cube[][3]={{54,62,0},{64,52.9,0},{64,62,0},{64,65.9,0}};
 //int cubebase[][3]={{54,61,14},{64,61.1,14},{54,51,14},{64,40.9,14}};


int cubebase[][3]={{50,50,0},{110,50,0},{110,110,0},{50,110,0}};
 int cube[][3]={{50,50,60},{110,50,60},{110,110,60},{50,110,60}};
 int cubeitr=0;
 for(cubeitr=0;cubeitr<4;cubeitr++){
       if(cubeitr<3){
      struct Vector3i P;
      P.x=cube[cubeitr][0];
      P.y=cube[cubeitr][1];
      P.z=cube[cubeitr][2];
      struct Vector3i P1;
      P1.x=cube[cubeitr+1][0];
      P1.y=cube[cubeitr+1][1];
      P1.z=cube[cubeitr+1][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,BLACK);

       }
       else{

      struct Vector3i P;
      P.x=cube[cubeitr][0];
      P.y=cube[cubeitr][1];
      P.z=cube[cubeitr][2];
      struct Vector3i P1;
      P1.x=cube[0][0];
      P1.y=cube[0][1];
      P1.z=cube[0][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,BLACK);
       }
 }
 {
      struct Vector3i P;
      P.x=cubebase[0][0];
      P.y=cubebase[0][1];
      P.z=cubebase[0][2];
      struct Vector3i P1;
      P1.x=cubebase[1][0];
      P1.y=cubebase[1][1];
      P1.z=cubebase[1][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,BLACK);
 }

 {
      struct Vector3i P;
      P.x=cubebase[0][0];
      P.y=cubebase[0][1];
      P.z=cubebase[0][2];
      struct Vector3i P1;
      P1.x=cubebase[3][0];
      P1.y=cubebase[3][1];
      P1.z=cubebase[3][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,BLACK);
 }

 cubeitr=0;
  for(cubeitr=0;cubeitr<4;cubeitr++){
        if(cubeitr!=2){
        struct Vector3i P;
      P.x=cube[cubeitr][0];
      P.y=cube[cubeitr][1];
      P.z=cube[cubeitr][2];
      struct Vector3i P1;
      P1.x=cubebase[cubeitr][0];
      P1.y=cubebase[cubeitr][1];
      P1.z=cubebase[cubeitr][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,BLACK);
        }
  }



DrawMap();
//DrawBackgroundorForest();
//DrawScreenSaver();
//DrawForest();
 //}
// draw shadow
/*double lamda[4];
 int shadeitr=0;
 for(shadeitr=0;shadeitr<4;shadeitr++){
       struct Vector3i p;
       p.x=cube[shadeitr][0];
       p.y=cube[shadeitr][1];
       p.z=cube[shadeitr][2];
      // lamda[shadeitr]=Shade(p);
 }*/

 //fillRect(10,110,30,30,GREY);

 //HLine(20,130,3,BLACK);
 //VLine(61,3,160,BLACK);




/*float intersectionPoint[4][3]={};
shadeitr=0;
for(shadeitr=0;shadeitr<4;shadeitr++){
       struct Vector3i p;
       p.x=cube[shadeitr][0];
       p.y=cube[shadeitr][1];
       p.z=cube[shadeitr][2];
       float lam=lamda[shadeitr];
       intersectionPoint[shadeitr][0]=p.x-lam*(90-p.x);
       intersectionPoint[shadeitr][1]=p.y-lam*(60-p.y);
       intersectionPoint[shadeitr][2]=p.z-lam*(10-p.z);
}
shadeitr=0;
for(shadeitr=0;shadeitr<4;shadeitr++){
       if(shadeitr<3){
      struct Vector3i P;
      P.x=intersectionPoint[shadeitr][0];
      P.y=intersectionPoint[shadeitr][1];
      P.z=intersectionPoint[shadeitr][2];
      struct Vector3i P1;
      P1.x=intersectionPoint[shadeitr+1][0];
      P1.y=intersectionPoint[shadeitr+1][1];
      P1.z=intersectionPoint[shadeitr+1][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,BLACK);

       }
       else{

      struct Vector3i P;
      P.x=intersectionPoint[shadeitr][0];
      P.y=intersectionPoint[shadeitr][1];
      P.z=intersectionPoint[shadeitr][2];
      struct Vector3i P1;
      P1.x=intersectionPoint[0][0];
      P1.y=intersectionPoint[0][1];
      P1.z=intersectionPoint[0][2];
      struct Vector2i p2d=Point(P);
      struct Vector2i p12d=Point(P1);
      drawline(p2d.x,p2d.y,p12d.x,p12d.y,BLACK);
       }
}
*/

  return 0;
}
}
}

/******************************************************************************
**                            End Of File
******************************************************************************/

#endif
