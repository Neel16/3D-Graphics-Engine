#if 0

/*
===============================================================================
 Name        : DrawLine.c
 Author      : Himanshu Gunjal(Adding on Prof. Harry Hua Li's code template)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/


#include <cr_section_macros.h>
#include <NXP/crp.h>
//#include "longhorn_sunset.h"

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
//__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

#include "LPC17xx.h"       /* LPC17xx definitions */
#include "ssp.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

/* Be careful with the port number and location number, because
some of the location may not exist in that port. */
#define PORT_NUM            0
#define LOCATION_NUM        0
#define TREE_NUM            2


uint8_t src_addr[SSP_BUFSIZE];
uint8_t dest_addr[SSP_BUFSIZE];
int colstart = 0;
int rowstart = 0;

//LCD
#define ST7735_TFTWIDTH  128
#define ST7735_TFTHEIGHT 160
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define swap(x, y) { x = x + y; y = x - y; x = x - y; }
#define PI 3.14

//Colors
#define BLACK   0x000000
#define WHITE   0xFFFFFF
#define RED     0xFF0000
#define GREEN   0x00FF00
#define BLUE    0x0000FF
#define MAGENTA 0xFF00FF
#define YELLOW  0xFFFF00
#define CYAN    0x00FFFF
#define PURPLE  0x8000FF
#define ORANGE  0xFF8000
#define LTBLUE  0x9090FF
#define BROWN   0x330000
#define OLIVE   0x808000
#define LIGHTBLUE 0x00FFE0
#define GREEN 0x00FF00
#define DARKBLUE 0x000033
#define GREENYELLOW 0xADFF2F
#define CYANBLUE 0xE0FFFF
#define DARKGREEN 0x006400
#define AZURE 0xF0FFFF

//Axes
int _height = ST7735_TFTHEIGHT;
int _width = ST7735_TFTWIDTH;
int cursor_x = 0, cursor_y = 0;

//Reduction Factor Lambda
float lambda = 0.5;
float lambda1 = 0.2;

//For Tilted Cube
double x1prime,y1prime,z1prime,x2prime,y2prime,z2prime, x3prime,y3prime,z3prime, x4prime,y4prime,z4prime,
x5prime,y5prime,z5prime, x6prime,y6prime,z6prime, x7prime,y7prime,z7prime,x8prime,y8prime,z8prime = 0;

uint32_t returnColour(int option) {

	uint32_t colour;
	switch(option) {
	case 0:
	  colour = BLACK;
	  break;
	case 1:
	  colour = WHITE;
	  break;
	case 2:
	  colour = RED;
	  break;
	case 3:
	  colour = GREEN;
	  break;
	case 4:
	  colour = BLUE;
	  break;
	case 5:
	  colour = YELLOW;
	  break;
	case 6:
	  colour = CYAN;
	  break;
	case 7:
	  colour = MAGENTA;
	  break;
	case 8:
	  colour = PURPLE;
	  break;
	case 9:
	  colour = ORANGE;
	  break;
	case 10:
	  colour = OLIVE;
	  break;
	case 11:
	  colour = LTBLUE;
	  break;
	case 12:
	  colour = BROWN;
	  break;
	default:
	  colour = BLACK;
	  break;
	}
	return colour;
}

struct world{
	int x;
	int y;
}sW;

struct camera{
	int xe;
	int ye;
	int ze;
}sCam;

struct Location{
   int x;
   int y;
}loc;

struct normal{
	float x;
	float y;
	float z;
};



//Function to write data into SPI
void spiwrite(uint8_t c) {
    int portnum = 0;
    src_addr[0] = c;
    SSP_SSELToggle( portnum, 0);
    SSPSend( portnum, (uint8_t *)src_addr, 1);
    SSP_SSELToggle( portnum, 1);
}

//Function to write command into SPI
void writecommand(uint8_t c) {
    LPC_GPIO0->FIOCLR |= (0x1<<21);
    spiwrite(c);
}

//Function to write data
void writedata(uint8_t c) {

    LPC_GPIO0->FIOSET |= (0x1<<21);
    spiwrite(c);
}

//Function to write word
void writeword(uint16_t c) {

    uint8_t d;
    d = c >> 8;
    writedata(d);
    d = c & 0xFF;
    writedata(d);
}

// Get the color and repeat the parameter to display
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

//Function to set address window between the given coordinates
void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {

      writecommand(ST7735_CASET);
      writeword(x0);
      writeword(x1);
      writecommand(ST7735_RASET);
      writeword(y0);
      writeword(y1);

}

//Function to draw pixel on the screen
void drawPixel(int16_t x, int16_t y, uint32_t color) {

	if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

    setAddrWindow(x,y,x+1,y+1);
    writecommand(ST7735_RAMWR);
    write888(color, 1);
}

//Function to perform LCD delay in milliseconds
void lcddelay(int ms){
	int count = 24000;
	int i;
	for ( i = count*ms; i--; i > 0);
}

//Initialize LCD
void lcd_init(){
/*
 * portnum    = 0 ;
 * cs         = p0.16 / p0.6 ?
 * rs         = p0.21
 * rst        = p0.22
*/
   uint32_t portnum = 0;
   int i;
   printf("LCD initialized\n");

   /* Notice the hack, for portnum 0 p0.16 is used */
   if ( portnum == 0 ){
        LPC_GPIO0->FIODIR |= (0x1<<16);   /*SSP1, P0.16 defined as Outputs*/
      }
   else
      {
        LPC_GPIO0->FIODIR |= (0x1<<6);    /*SSP0 P0.6 defined as Outputs*/
      }

   /* Set rs(dc) and rst as outputs */
    LPC_GPIO0->FIODIR |= (0x1<<21);       /*rs/dc P0.21 defined as Outputs*/
    LPC_GPIO0->FIODIR |= (0x1<<22);       /*rst P0.22 defined as Outputs*/

   /* Reset sequence */
    LPC_GPIO0->FIOSET |= (0x1<<22);
    lcddelay(500);                        /* delay 500 ms */
    LPC_GPIO0->FIOCLR |= (0x1<<22);
    lcddelay(500);                        /* delay 500 ms */
    LPC_GPIO0->FIOSET |= (0x1<<22);
    lcddelay(500);                        /* delay 500 ms */

    for ( i = 0; i < SSP_BUFSIZE; i++ )   /* Init RD and WR buffer */
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

     lcddelay(200);
     /* delay 200 ms */
     /* Disp on */
     SSP_SSELToggle( portnum, 0 );
     src_addr[0] = 0x29;    /* Disp On */
     SSPSend( portnum, (uint8_t *)src_addr, 1 );
     SSP_SSELToggle( portnum, 1 );
     /* delay 200 ms */
     lcddelay(200);
}

//Function to fill rectangle
void fillrect(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)
{
	int16_t i;
	int16_t width, height;

	width = x1-x0+1;
	height = y1-y0+1;
	setAddrWindow(x0,y0,x1,y1);
	writecommand(ST7735_RAMWR);
	write888(color,width*height);
}

//Function to fill screen
void fillScreen(uint16_t color) {
  fillRect(0, 0, _width, _height, color);
}

//Function to Draw a line function
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint32_t color){

	int16_t slope = abs(y1 - y0) > abs(x1 - x0);
	//printf("%d",slope);
	if (slope)
	{
	swap(x0, y0);
	swap(x1, y1);
	}

	if (x0 > x1)
	{
	swap(x0, x1);
	swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
    //printf("%d",err);
	int16_t ystep;

	if (y0 < y1)
	{
	ystep = 1;
	} else
	{
	ystep = -1;
	}

	for (; x0<=x1; x0++)
	{
	if (slope)
	{
	drawPixel(y0, x0, color);
	} else
	{
	drawPixel(x0, y0, color);
	}
	err -= dy;
	if (err < 0)
	{
	y0 += ystep;
	err += dx;
	}
	}
}

void VLine(int16_t x, int16_t y, int16_t h, uint16_t color){

	drawLine(x, y, x, y+h-1, color);
}

void HLine(int16_t x, int16_t y,  int16_t w, uint16_t color){

	 drawLine(x, y, x+w-1, y, color);
}

struct world worldCordinateSystem(int x, int y, int z)
{
	int xe=sCam.xe, ye=sCam.ye, ze=sCam.ze;
	int d=100;
	int x0=64,y0=80;
	struct world s;

	double theta,phi,rho;
	double xprime,yprime,zprime;

	rho = sqrt(pow(xe,2)+pow(ye,2)+pow(ze,2));
	theta = acos(xe/(sqrt(pow(xe,2)+pow(ye,2))));
	phi = acos(ze/(sqrt(pow(xe,2)+pow(ye,2)+pow(ze,2))));

	double matrix3d[4][4]={
			{-sin(theta),				cos(theta),					0,						0	},
			{-cos(phi)*cos(theta),		-cos(phi)*sin(theta),		sin(phi),				0	},
			{-sin(phi)*cos(theta),		-sin(phi)*cos(theta),		-cos(phi),				rho	},
			{0,							0,							0,						1	}
			};

	xprime	=	((x*matrix3d[0][0])	+	(y*matrix3d[0][1])	+	(z*matrix3d[0][2])	+	(1*matrix3d[0][3]));
	yprime	=	((x*matrix3d[1][0])	+	(y*matrix3d[1][1])	+	(z*matrix3d[1][2])	+	(1*matrix3d[1][3]));
	zprime	=	((x*matrix3d[2][0])	+	(y*matrix3d[2][1])	+	(z*matrix3d[2][2])	+	(1*matrix3d[2][3]));

	s.x=x0+(10+(xprime/zprime)*d);
	s.y=y0-(30+(yprime/zprime)*d);

	return s;
}

void drawWorldCordinates()
{
	struct world s1,s2,s3;

	s1=worldCordinateSystem(160,0,0);
		drawLine(sW.x,sW.y,s1.x,s1.y,0xFF0000);
	s1=worldCordinateSystem(160,1,0);
		drawLine(sW.x,sW.y,s1.x,s1.y,0xFF0000);
	s1=worldCordinateSystem(160,0,1);
		drawLine(sW.x,sW.y,s1.x,s1.y,0xFF0000);


	s2=worldCordinateSystem(0,160,0);
		drawLine(sW.x,sW.y,s2.x,s2.y,0x00FF00);
	s2=worldCordinateSystem(1,160,0);
		drawLine(sW.x,sW.y,s2.x,s2.y,0x00FF00);
	s2=worldCordinateSystem(0,160,1);
		drawLine(sW.x,sW.y,s2.x,s2.y,0x00FF00);


	s3=worldCordinateSystem(0,0,160);
		drawLine(sW.x,sW.y,s3.x,s3.y,0x0000FF);
	s3=worldCordinateSystem(1,0,160);
		drawLine(sW.x,sW.y,s3.x,s3.y,0x0000FF);
	s3=worldCordinateSystem(0,1,160);
		drawLine(sW.x,sW.y,s3.x,s3.y,0x0000FF);
}

// Implement Tree Pattern
void Endpoint_rotation(float *x2, float *y2, float x1, float y1, int angle) {
 	float xt, yt, xr, yr;
 	float c, s, ra;
 	ra = angle * (3.14159265359 / 180);
 	xt = *x2 - x1;
 	yt = *y2 - y1;
 	c = cos(ra);
 	s = sin(ra);
 	xr = xt * c - yt * s;
 	yr = xt * s + yt * c;
 	*x2 = xr + x1;
 	*y2 = yr + y1;
 	return;
 }

void tree(float x1, float y1, float x2, float y2, uint16_t color, int num) {
 	int x3, y3;
 	if (num <= 0)
 		return;
 	drawLine(x1, y1, x2, y2, BROWN);
 	x3 = x1;
 	y3 = y1;
 	x1 = x2;
 	y1 = y2;
 	y2 = y2 + (y1 - y3) * lambda;
 	x2 = x2 + (x1 - x3) * lambda;

 	Endpoint_rotation(&x2, &y2, x1, y1, 30);  //Rotate by 30 and find the new location
 	drawLine(x1, y1, x2, y2, color);
 	tree(x1, y1, x2, y2,GREEN, num - 1);

 	Endpoint_rotation(&x2, &y2, x1, y1, 330);
 	drawLine(x1, y1, x2, y2, color);
 	tree(x1, y1, x2, y2, GREEN, num - 1);

 	Endpoint_rotation(&x2, &y2, x1, y1, 330);
 	drawLine(x1, y1, x2, y2, color);
 	tree(x1, y1, x2, y2, GREEN, num - 1);
}

// Implement Square Pattern
bool exitPatternGeneration(struct Location P0, struct Location P1, struct Location P2, struct Location P3) {

	return ((((P0.x == P1.x) && (P0.y = P1.y) && (P1.x == P2.x) && (P1.y == P2.y) &&
			(P2.x == P3.x) && (P2.y == P3.y) && (P3.x == P0.x) && (P3.y == P0.y)))
			? true : false);
}

void calculateNewLocations(struct Location a, struct Location b, struct Location *result) {

   result->x = a.x + (lambda1 * (b.x - a.x));
   result->y = a.y + (lambda1 * (b.y - a.y));
}

void Square_pattern_cube(struct Location start, int width, uint32_t colour,int x_dist,int y_dist,int z_dist, int size) {

   struct Location P0, P1, P2, P3;
   struct Location P0_buffer, P1_buffer, P2_buffer, P3_buffer;
   struct world pt0, pt1, pt2, pt3;

   P0 = start;
   P1.x = start.x + width;
   P1.y = start.y;
   P2.x = start.x + width;
   P2.y = start.y + width;
   P3.x = start.x;
   P3.y = start.y + width;

   bool status = false;
   int j = 0;
   while (1) {

   //if P0 = P1 = P2 = P3 then exit
   status =exitPatternGeneration(P0, P1, P2, P3);
   if (status == true) {
	 return;
	}

   //Transformation to the respective Locations on the World co-ordinate system
   pt0 = worldCordinateSystem(size+x_dist, P0.y+y_dist, P0.x+z_dist);
   pt1 = worldCordinateSystem(size+x_dist, P1.y+y_dist, P1.x+z_dist);
   pt2 = worldCordinateSystem(size+x_dist, P2.y+y_dist, P2.x+z_dist);
   pt3 = worldCordinateSystem(size+x_dist, P3.y+y_dist, P3.x+z_dist);


   drawLine(pt0.x, pt0.y, pt1.x, pt1.y, colour); // Draw line from p0 to p1
   drawLine(pt1.x, pt1.y, pt2.x, pt2.y, colour);
   drawLine(pt2.x, pt2.y, pt3.x, pt3.y, colour);
   drawLine(pt3.x, pt3.y, pt0.x, pt0.y, colour);

   // Store the old vertices for reference
   P0_buffer = P0;
   P1_buffer = P1;
   P2_buffer = P2;
   P3_buffer = P3;

   // Calculate new vertices for the square
   calculateNewLocations(P0_buffer, P1_buffer, &P0);
   calculateNewLocations(P1_buffer, P2_buffer, &P1);
   calculateNewLocations(P2_buffer, P3_buffer, &P2);
   calculateNewLocations(P3_buffer, P0_buffer, &P3);
    j++;
   }
}

void Square_pattern_Tiltedcube(struct Location start, int width, uint32_t colour, int x_dist, int y_dist, int z_dist,int size) {


   struct Location P0, P1, P2, P3;
   struct Location P0_buffer, P1_buffer, P2_buffer, P3_buffer;
   struct world pt0, pt1, pt2, pt3;
   double gamma = PI/9;

   P0 = start;
   P1.x = start.x + width;
   P1.y = start.y;
   P2.x = start.x + width;
   P2.y = start.y + width;
   P3.x = start.x;
   P3.y = start.y + width;

   bool status = false;
   int j = 0;
   while (1) {

   //if P0 = P1 = P2 = P3 then exit
   status =exitPatternGeneration(P0, P1, P2, P3);
   if (status == true) {
	return;
	}

   //Transformation to the respective Locations on the World co-ordinate system
   pt0 = worldCordinateSystem(size+x_dist,(((P0.y+y_dist)*cos(gamma))+((P0.x+z_dist)*sin(gamma))),(((P0.y+y_dist)*(-sin(gamma)))+((P0.x+z_dist)*cos(gamma))));
   pt1 = worldCordinateSystem(size+x_dist,(((P1.y+y_dist)*cos(gamma))+((P1.x+z_dist)*sin(gamma))),(((P1.y+y_dist)*(-sin(gamma)))+((P1.x+z_dist)*cos(gamma))));
   pt2 = worldCordinateSystem(size+x_dist,(((P2.y+y_dist)*cos(gamma))+((P2.x+z_dist)*sin(gamma))),(((P2.y+y_dist)*(-sin(gamma)))+((P2.x+z_dist)*cos(gamma))));
   pt3 = worldCordinateSystem(size+x_dist,(((P3.y+y_dist)*cos(gamma))+((P3.x+z_dist)*sin(gamma))),(((P3.y+y_dist)*(-sin(gamma)))+((P3.x+z_dist)*cos(gamma))));

   drawLine(pt0.x, pt0.y, pt1.x, pt1.y, colour); // Draw line from p0 to p1
   drawLine(pt1.x, pt1.y, pt2.x, pt2.y, colour);
   drawLine(pt2.x, pt2.y, pt3.x, pt3.y, colour);
   drawLine(pt3.x, pt3.y, pt0.x, pt0.y, colour);

   // Store the old vertices for reference
   P0_buffer = P0;
   P1_buffer = P1;
   P2_buffer = P2;
   P3_buffer = P3;

   // Calculate new vertices for the square
   calculateNewLocations(P0_buffer, P1_buffer, &P0);
   calculateNewLocations(P1_buffer, P2_buffer, &P1);
   calculateNewLocations(P2_buffer, P3_buffer, &P2);
   calculateNewLocations(P3_buffer, P0_buffer, &P3);
    j++;
   }
}

// Implement Initials (alphabet)
void alphabet(int x_dist, int y_dist,int z_dist,int size)
{
	struct world s1;

	int i,j;
	int map[size][size];

	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
		{
			if(i>=5 && j>=5 && j<=35 && i<=10)
				map[i][j]=1;
			else if(i>=10 && j>=5 && j<=10 && i<=35)
				map[i][j]=1;
			else if(i>=10 && j>=20 && j<=25 && i<=25)
				map[i][j]=1;
			else
				map[i][j]=0;
		}
	}

for(i=0;i<size;i++)
{
  for(j=0;j<size;j++)
  {
	if(map[i][j]==1){
		s1=worldCordinateSystem(j+x_dist,i+y_dist,size+z_dist);
		drawPixel(s1.x,s1.y,0x000000);
   }
  }
 }
}

// Implement Initials (alphabet)
void alphabet_tilted(int x_dist, int y_dist,int z_dist,int size)
{
	struct world s1;

	int i,j;
	int map[size][size];
	double gamma = PI/9;

	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
		{
			if(i>=5 && j>=5 && j<=35 && i<=10)
				map[i][j]=1;
			else if(i>=10 && j>=5 && j<=10 && i<=35)
				map[i][j]=1;
			else if(i>=10 && j>=20 && j<=25 && i<=25)
				map[i][j]=1;
			else
				map[i][j]=0;
		}
	}

for(i=0;i<size;i++)
{
  for(j=0;j<size;j++)
  {
	if(map[i][j]==1){
		s1=worldCordinateSystem(j+x_dist,i*cos(gamma)+size*sin(gamma)+y_dist,i*(-sin(gamma))+size*cos(gamma)+z_dist);
		drawPixel(s1.x,s1.y,0x000000);
   }
  }
 }
}

//Function to draw Cube and its shadow using the Ray Equation
void drawCube(int x_dist, int y_dist,int z_dist,int size)
{
    struct world s1,s2,s3,s4;
	struct world s5,s6,s7,s8;
	struct world Ray1,Ray2,Ray3,Ray4, s_temp;
	struct normal norm_point1,norm_point2,norm_point3,norm_point4,norm_pointA,norm_pointB,normal_vector,point_source_3d,R0,R1,R2,R3,temp_R, temp_3D;
	int i,j;

	uint16_t outline_color = 0x000000;

	s1=worldCordinateSystem(0+x_dist,0+y_dist,size+z_dist);
	s2=worldCordinateSystem(size+x_dist,0+y_dist,size+z_dist);
	s3=worldCordinateSystem(size+x_dist,size+y_dist,size+z_dist);
	s4=worldCordinateSystem(0+x_dist,size+y_dist,size+z_dist);
	s5=worldCordinateSystem(0+x_dist,0+y_dist,0+z_dist);
	s6=worldCordinateSystem(size+x_dist,0+y_dist,0+z_dist);
	s7=worldCordinateSystem(size+x_dist,size+y_dist,0+z_dist);
	s8=worldCordinateSystem(0+x_dist,size+y_dist,0+z_dist);

	drawLine(s1.x,s1.y,s2.x,s2.y,outline_color);
	drawLine(s2.x,s2.y,s3.x,s3.y,outline_color);
	drawLine(s3.x,s3.y,s4.x,s4.y,outline_color);
	drawLine(s4.x,s4.y,s1.x,s1.y,outline_color);
	drawLine(s1.x,s1.y,s5.x,s5.y,outline_color);
	drawLine(s2.x,s2.y,s6.x,s6.y,outline_color);
	drawLine(s3.x,s3.y,s7.x,s7.y,outline_color);
	drawLine(s4.x,s4.y,s8.x,s8.y,outline_color);
	drawLine(s5.x,s5.y,s6.x,s6.y,outline_color);
	drawLine(s6.x,s6.y,s7.x,s7.y,outline_color);
	drawLine(s7.x,s7.y,s8.x,s8.y,outline_color);
	drawLine(s8.x,s8.y,s5.x,s5.y,outline_color);



	//drawLine(point_source.x,point_source.y,s1.x,s1.y,BLACK);
	//drawLine(point_source.x,point_source.y,s2.x,s2.y,BLACK);
	//drawLine(point_source.x,point_source.y,s3.x,s3.y,BLACK);
	//drawLine(point_source.x,point_source.y,s4.x,s4.y,BLACK);


	point_source_3d.x = 200;
	point_source_3d.y = 100;
	point_source_3d.z = 1500;

	norm_point1.x = 0+x_dist;
	norm_point1.y = size+y_dist;
	norm_point1.z = size+z_dist;

	norm_point2.x = size+x_dist;
	norm_point2.y = size+y_dist;
	norm_point2.z = size+z_dist;

	norm_point3.x = size+x_dist;
	norm_point3.y = 0+y_dist;
	norm_point3.z = size+z_dist;

	norm_point4.x = 0+x_dist;
	norm_point4.y = 0+y_dist;
	norm_point4.z = size+z_dist;

	norm_pointA.x = norm_point1.x- norm_point2.x;
	norm_pointA.y = norm_point1.y -norm_point2.y;
	norm_pointA.z = norm_point1.z -norm_point2.z;

	norm_pointB.x = norm_point3.x- norm_point2.x;
	norm_pointB.y = norm_point3.y- norm_point2.y;
	norm_pointB.z = norm_point3.z- norm_point2.z;

	normal_vector.x = (norm_pointA.y * norm_pointB.z) - (norm_pointA.z * norm_pointB.y);
	normal_vector.y = (norm_pointA.z * norm_pointB.x) - (norm_pointA.x * norm_pointB.z);
	normal_vector.z = (norm_pointA.x * norm_pointB.y) - (norm_pointA.y * norm_pointB.x);

	float lambda0, lambda1, lambda2, lambda3, temp_lambda;
	lambda0 = ((normal_vector.x * norm_point2.x) + (normal_vector.y * norm_point2.y) + (normal_vector.z * norm_point2.z))/((normal_vector.x * (norm_point2.x-point_source_3d.x)) + (normal_vector.y * (norm_point2.y-point_source_3d.y)) + (normal_vector.z * (norm_point2.z-point_source_3d.z)));
	lambda1 = ((normal_vector.x * norm_point1.x) + (normal_vector.y * norm_point1.y) + (normal_vector.z * norm_point1.z))/((normal_vector.x * (norm_point1.x-point_source_3d.x)) + (normal_vector.y * (norm_point1.y-point_source_3d.y)) + (normal_vector.z * (norm_point1.z-point_source_3d.z)));
	lambda2 = ((normal_vector.x * norm_point4.x) + (normal_vector.y * norm_point4.y) + (normal_vector.z * norm_point4.z))/((normal_vector.x * (norm_point4.x-point_source_3d.x)) + (normal_vector.y * (norm_point4.y-point_source_3d.y)) + (normal_vector.z * (norm_point4.z-point_source_3d.z)));
	lambda3 = ((normal_vector.x * norm_point3.x) + (normal_vector.y * norm_point3.y) + (normal_vector.z * norm_point3.z))/((normal_vector.x * (norm_point3.x-point_source_3d.x)) + (normal_vector.y * (norm_point3.y-point_source_3d.y)) + (normal_vector.z * (norm_point3.z-point_source_3d.z)));

	R0.x = norm_point2.x + lambda0 * (point_source_3d.x - norm_point2.x);
	R0.y = norm_point2.y + lambda0 * (point_source_3d.y - norm_point2.y);
	R0.z = norm_point2.z + lambda0 * (point_source_3d.z - norm_point2.z);
	R1.x = norm_point1.x + lambda1 * (point_source_3d.x - norm_point1.x);
	R1.y = norm_point1.y + lambda1 * (point_source_3d.y - norm_point1.y);
	R1.z = norm_point1.z + lambda1 * (point_source_3d.z - norm_point1.z);
	R2.x = norm_point4.x + lambda2 * (point_source_3d.x - norm_point4.x);
	R2.y = norm_point4.y + lambda2 * (point_source_3d.y - norm_point4.y);
	R2.z = norm_point4.z + lambda2 * (point_source_3d.z - norm_point4.z);
	R3.x = norm_point3.x + lambda3 * (point_source_3d.x - norm_point3.x);
	R3.y = norm_point3.y + lambda3 * (point_source_3d.y - norm_point3.y);
	R3.z = norm_point3.z + lambda3 * (point_source_3d.z - norm_point3.z);

	Ray1=worldCordinateSystem(R0.x, R0.y, R0.z); // P0
	Ray2=worldCordinateSystem(R1.x, R1.y, R1.z); // P1
	Ray3=worldCordinateSystem(R2.x, R2.y, R2.z); // P2
	Ray4=worldCordinateSystem(R3.x, R3.y, R3.z); // P3

	drawLine(Ray1.x, Ray1.y, Ray2.x, Ray2.y, BLACK);
	drawLine(Ray2.x, Ray2.y, Ray3.x, Ray3.y, BLACK);
	drawLine(Ray3.x, Ray3.y, Ray4.x, Ray4.y, BLACK);
	drawLine(Ray4.x, Ray4.y, Ray1.x, Ray1.y, BLACK);


	for (i=0; i<size; i++) {

	    for (j=0; j<size; j++) {

	        temp_3D.x = j+x_dist;
	    	temp_3D.y = i+y_dist;
	    	temp_3D.z = size+z_dist;
	    	temp_lambda = ((normal_vector.x * temp_3D.x) + (normal_vector.y * temp_3D.y) + (normal_vector.z * temp_3D.z))/((normal_vector.x * (temp_3D.x-point_source_3d.x)) + (normal_vector.y * (temp_3D.y-point_source_3d.y)) + (normal_vector.z * (temp_3D.z-point_source_3d.z)));
	    	temp_R.x = temp_3D.x + temp_lambda * (point_source_3d.x - temp_3D.x);
	    	temp_R.y = temp_3D.y + temp_lambda * (point_source_3d.y - temp_3D.y);
	    	temp_R.z = temp_3D.z + temp_lambda * (point_source_3d.z - temp_3D.z);
	    	s_temp = worldCordinateSystem(temp_R.x, temp_R.y, temp_R.z);
	    	drawPixel(s_temp.x, s_temp.y, BLACK);
	    }
	 }
}

//Function to get the rotated Coordinates of the cube around X-axis
void RotationMatrix(double size)
{
	double x1=0,		y1=0,		z1=size;
	double x2=size,		y2=0,		z2=size;
  	double x3=size,		y3=size,	z3=size;
  	double x4=0,		y4=size,	z4=size;
  	double x5=0,		y5=0,		z5=0;
  	double x6=size,		y6=0,		z6=0;
  	double x7=size,		y7=size,	z7=0;
  	double x8=0,		y8=size,	z8=0;

	double alpha=PI/9;

	double RotationMatrix[4][4]={
				{1,					0,					0,						0	},
				{0,                 cos(alpha),		    -sin(alpha),		    0   },
				{0,					sin(alpha),		     cos(alpha),			0   },
				{0,					0,					0,						1	}
				};

	x1prime	=	((x1*RotationMatrix[0][0])	+(y1*RotationMatrix[1][0])	+	(z1*RotationMatrix[2][0])	+	(1*RotationMatrix[3][0]));
	y1prime	=	((x1*RotationMatrix[0][1])	+(y1*RotationMatrix[1][1])	+	(z1*RotationMatrix[2][1]) 	+	(1*RotationMatrix[3][1]));
	z1prime	=	((x1*RotationMatrix[0][2])	+(y1*RotationMatrix[1][2])	+	(z1*RotationMatrix[2][2])	+	(1*RotationMatrix[3][2]));
	x2prime	=	((x2*RotationMatrix[0][0])	+(y2*RotationMatrix[1][0])	+	(z2*RotationMatrix[2][0])	+	(1*RotationMatrix[3][0]));
	y2prime	=	((x2*RotationMatrix[0][1])	+(y2*RotationMatrix[1][1])	+	(z2*RotationMatrix[2][1])	+	(1*RotationMatrix[3][1]));
	z2prime	=	((x2*RotationMatrix[0][2])	+(y2*RotationMatrix[1][2])	+	(z2*RotationMatrix[2][2])	+	(1*RotationMatrix[3][2]));
    x3prime	=	((x3*RotationMatrix[0][0])	+(y3*RotationMatrix[1][0])	+	(z3*RotationMatrix[2][0])	+	(1*RotationMatrix[3][0]));
	y3prime	=	((x3*RotationMatrix[0][1])	+(y3*RotationMatrix[1][1])	+	(z3*RotationMatrix[2][1])	+	(1*RotationMatrix[3][1]));
	z3prime	=	((x3*RotationMatrix[0][2])	+(y3*RotationMatrix[1][2])	+	(z3*RotationMatrix[2][2])	+	(1*RotationMatrix[3][2]));
    x4prime	=	((x4*RotationMatrix[0][0])	+(y4*RotationMatrix[1][0])	+	(z4*RotationMatrix[2][0])	+	(1*RotationMatrix[3][0]));
	y4prime	=	((x4*RotationMatrix[0][1])	+(y4*RotationMatrix[1][1])	+	(z4*RotationMatrix[2][1])	+	(1*RotationMatrix[3][1]));
	z4prime	=	((x4*RotationMatrix[0][2])	+(y4*RotationMatrix[1][2])	+	(z4*RotationMatrix[2][2])	+	(1*RotationMatrix[3][2]));
    x5prime	=	((x5*RotationMatrix[0][0])	+(y5*RotationMatrix[1][0])	+	(z5*RotationMatrix[2][0])	+	(1*RotationMatrix[3][0]));
	y5prime	=	((x5*RotationMatrix[0][1])	+(y5*RotationMatrix[1][1])	+	(z5*RotationMatrix[2][1])	+	(1*RotationMatrix[3][1]));
	z5prime	=	((x5*RotationMatrix[0][2])	+(y5*RotationMatrix[1][2])	+	(z5*RotationMatrix[2][2])	+	(1*RotationMatrix[3][2]));
    x6prime	=	((x6*RotationMatrix[0][0])	+(y6*RotationMatrix[1][0])	+	(z6*RotationMatrix[2][0])	+	(1*RotationMatrix[3][0]));
	y6prime	=	((x6*RotationMatrix[0][1])	+(y6*RotationMatrix[1][1])	+	(z6*RotationMatrix[2][1])	+	(1*RotationMatrix[3][1]));
	z6prime	=	((x6*RotationMatrix[0][2])	+(y6*RotationMatrix[1][2])	+	(z6*RotationMatrix[2][2])	+	(1*RotationMatrix[3][2]));
    x7prime	=	((x7*RotationMatrix[0][0])	+(y7*RotationMatrix[1][0])	+	(z7*RotationMatrix[2][0])	+	(1*RotationMatrix[3][0]));
	y7prime	=	((x7*RotationMatrix[0][1])	+(y7*RotationMatrix[1][1])	+	(z7*RotationMatrix[2][1])	+	(1*RotationMatrix[3][1]));
	z7prime	=	((x7*RotationMatrix[0][2])	+(y7*RotationMatrix[1][2])	+	(z7*RotationMatrix[2][2])	+	(1*RotationMatrix[3][2]));
    x8prime	=	((x8*RotationMatrix[0][0])	+(y8*RotationMatrix[1][0])	+	(z8*RotationMatrix[2][0])	+	(1*RotationMatrix[3][0]));
	y8prime	=	((x8*RotationMatrix[0][1])	+(y8*RotationMatrix[1][1])	+	(z8*RotationMatrix[2][1])	+	(1*RotationMatrix[3][1]));
	z8prime	=	((x8*RotationMatrix[0][2])	+(y8*RotationMatrix[1][2])	+	(z8*RotationMatrix[2][2])	+	(1*RotationMatrix[3][2]));

}

//Function to draw tilted Cube and its Shadow using Ray Equation
void drawTiltedCube(int x_dist, int y_dist,int z_dist,int size)
{
	struct world s1,s2,s3,s4;
	struct world s5,s6,s7,s8;
	struct world point_source,Ray1,Ray2,Ray3,Ray4, s_temp;
	struct normal norm_point1,norm_point2,norm_point3,norm_point4,norm_pointA,norm_pointB,normal_vector,point_source_3d,R0,R1,R2,R3,temp_R,temp_3D;
	int i,j;

	uint16_t outline_color = 0x000000;

	s1=worldCordinateSystem(x1prime+x_dist,y1prime+y_dist,z1prime+z_dist);
	s2=worldCordinateSystem(x2prime+x_dist,y2prime+y_dist,z2prime+z_dist);
	s3=worldCordinateSystem(x3prime+x_dist,y3prime+y_dist,z3prime+z_dist);
	s4=worldCordinateSystem(x4prime+x_dist,y4prime+y_dist,z4prime+z_dist);
	s5=worldCordinateSystem(x5prime+x_dist,y5prime+y_dist,z5prime+z_dist);
	s6=worldCordinateSystem(x6prime+x_dist,y6prime+y_dist,z6prime+z_dist);
	s7=worldCordinateSystem(x7prime+x_dist,y7prime+y_dist,z7prime+z_dist);
	s8=worldCordinateSystem(x8prime+x_dist,y8prime+y_dist,z8prime+z_dist);

	drawLine(s1.x,s1.y,s2.x,s2.y,outline_color);
	drawLine(s2.x,s2.y,s3.x,s3.y,outline_color);
	drawLine(s3.x,s3.y,s4.x,s4.y,outline_color);
	drawLine(s4.x,s4.y,s1.x,s1.y,outline_color);
	drawLine(s1.x,s1.y,s5.x,s5.y,outline_color);
	drawLine(s2.x,s2.y,s6.x,s6.y,outline_color);
	drawLine(s3.x,s3.y,s7.x,s7.y,outline_color);
	drawLine(s4.x,s4.y,s8.x,s8.y,outline_color);
	drawLine(s5.x,s5.y,s6.x,s6.y,outline_color);
	drawLine(s6.x,s6.y,s7.x,s7.y,outline_color);
	drawLine(s7.x,s7.y,s8.x,s8.y,outline_color);
	drawLine(s8.x,s8.y,s5.x,s5.y,outline_color);

	//drawLine(point_source.x,point_source.y,s1.x,s1.y,BLACK);
	//drawLine(point_source.x,point_source.y,s2.x,s2.y,BLACK);
	//drawLine(point_source.x,point_source.y,s3.x,s3.y,BLACK);
	//drawLine(point_source.x,point_source.y,s4.x,s4.y,BLACK);


	point_source_3d.x = 200;
	point_source_3d.y = 100;
	point_source_3d.z = 1500;

	norm_point1.x = 0+x_dist;
	norm_point1.y = size+y_dist;
	norm_point1.z = size+z_dist;

	norm_point2.x = size+x_dist;
	norm_point2.y = size+y_dist;
	norm_point2.z = size+z_dist;

	norm_point3.x = size+x_dist;
	norm_point3.y = 0+y_dist;
	norm_point3.z = size+z_dist;

	norm_point4.x = 0+x_dist;
	norm_point4.y = 0+y_dist;
	norm_point4.z = size+z_dist;

	norm_pointA.x = norm_point1.x- norm_point2.x;
	norm_pointA.y = norm_point1.y -norm_point2.y;
	norm_pointA.z = norm_point1.z -norm_point2.z;

	norm_pointB.x = norm_point3.x- norm_point2.x;
	norm_pointB.y = norm_point3.y- norm_point2.y;
	norm_pointB.z = norm_point3.z- norm_point2.z;

	normal_vector.x = (norm_pointA.y * norm_pointB.z) - (norm_pointA.z * norm_pointB.y);
	normal_vector.y = (norm_pointA.z * norm_pointB.x) - (norm_pointA.x * norm_pointB.z);
	normal_vector.z = (norm_pointA.x * norm_pointB.y) - (norm_pointA.y * norm_pointB.x);

	float lambda0, lambda1, lambda2, lambda3, temp_lambda;
	lambda0 = ((normal_vector.x * norm_point2.x) + (normal_vector.y * norm_point2.y) + (normal_vector.z * norm_point2.z))/((normal_vector.x * (norm_point2.x-point_source_3d.x)) + (normal_vector.y * (norm_point2.y-point_source_3d.y)) + (normal_vector.z * (norm_point2.z-point_source_3d.z)));
	lambda1 = ((normal_vector.x * norm_point1.x) + (normal_vector.y * norm_point1.y) + (normal_vector.z * norm_point1.z))/((normal_vector.x * (norm_point1.x-point_source_3d.x)) + (normal_vector.y * (norm_point1.y-point_source_3d.y)) + (normal_vector.z * (norm_point1.z-point_source_3d.z)));
	lambda2 = ((normal_vector.x * norm_point4.x) + (normal_vector.y * norm_point4.y) + (normal_vector.z * norm_point4.z))/((normal_vector.x * (norm_point4.x-point_source_3d.x)) + (normal_vector.y * (norm_point4.y-point_source_3d.y)) + (normal_vector.z * (norm_point4.z-point_source_3d.z)));
	lambda3 = ((normal_vector.x * norm_point3.x) + (normal_vector.y * norm_point3.y) + (normal_vector.z * norm_point3.z))/((normal_vector.x * (norm_point3.x-point_source_3d.x)) + (normal_vector.y * (norm_point3.y-point_source_3d.y)) + (normal_vector.z * (norm_point3.z-point_source_3d.z)));

	R0.x = norm_point2.x + lambda0 * (point_source_3d.x - norm_point2.x);
	R0.y = norm_point2.y + lambda0 * (point_source_3d.y - norm_point2.y);
	R0.z = norm_point2.z + lambda0 * (point_source_3d.z - norm_point2.z);
	R1.x = norm_point1.x + lambda1 * (point_source_3d.x - norm_point1.x);
	R1.y = norm_point1.y + lambda1 * (point_source_3d.y - norm_point1.y);
	R1.z = norm_point1.z + lambda1 * (point_source_3d.z - norm_point1.z);
	R2.x = norm_point4.x + lambda2 * (point_source_3d.x - norm_point4.x);
	R2.y = norm_point4.y + lambda2 * (point_source_3d.y - norm_point4.y);
	R2.z = norm_point4.z + lambda2 * (point_source_3d.z - norm_point4.z);
	R3.x = norm_point3.x + lambda3 * (point_source_3d.x - norm_point3.x);
	R3.y = norm_point3.y + lambda3 * (point_source_3d.y - norm_point3.y);
	R3.z = norm_point3.z + lambda3 * (point_source_3d.z - norm_point3.z);

	Ray1=worldCordinateSystem(R0.x, R0.y, R0.z); // P0
	Ray2=worldCordinateSystem(R1.x, R1.y, R1.z); // P1
	Ray3=worldCordinateSystem(R2.x, R2.y, R2.z); // P2
	Ray4=worldCordinateSystem(R3.x, R3.y, R3.z); // P3

	drawLine(Ray1.x, Ray1.y, Ray2.x, Ray2.y, BLACK);
	drawLine(Ray2.x, Ray2.y, Ray3.x, Ray3.y, BLACK);
	drawLine(Ray3.x, Ray3.y, Ray4.x, Ray4.y, BLACK);
	drawLine(Ray4.x, Ray4.y, Ray1.x, Ray1.y, BLACK);


	for (i=0; i<size; i++) {

		   for (j=0; j<size; j++) {

			   temp_3D.x = j+x_dist;
		       temp_3D.y = i+y_dist;
		       temp_3D.z = size+z_dist;
		       temp_lambda = ((normal_vector.x * temp_3D.x) + (normal_vector.y * temp_3D.y) + (normal_vector.z * temp_3D.z))/((normal_vector.x * (temp_3D.x-point_source_3d.x)) + (normal_vector.y * (temp_3D.y-point_source_3d.y)) + (normal_vector.z * (temp_3D.z-point_source_3d.z)));
		       temp_R.x = temp_3D.x + temp_lambda * (point_source_3d.x - temp_3D.x);
		       temp_R.y = temp_3D.y + temp_lambda * (point_source_3d.y - temp_3D.y);
		       temp_R.z = temp_3D.z + temp_lambda * (point_source_3d.z - temp_3D.z);
		       s_temp = worldCordinateSystem(temp_R.x, temp_R.y, temp_R.z);
		       	   drawPixel(s_temp.x, s_temp.y, BLACK);
		    	}
		 }
}

//Function to fill Cube
void cubefill(int x_size,int y_size,int z_size,int size)
{
	struct world s1;

	int i,j;
	int a[size][size];

	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
		{
				s1=worldCordinateSystem(j+x_size,i+y_size,size+z_size);	//top fill
				drawPixel(s1.x,s1.y,BLUE);

				s1=worldCordinateSystem(i+x_size,size+y_size,j+z_size);	// right fill
				drawPixel(s1.x,s1.y,CYAN);

            	s1=worldCordinateSystem(size+x_size,j+y_size,i+z_size);	// left fill
				drawPixel(s1.x,s1.y,GREEN);
		}
	}
}

//Function to fill tilted Cube
void cubefill_tilted(int x_size,int y_size,int z_size,int size)
{
	struct world s1;

	    double gamma = PI/9;
		int i,j;
		int a[size][size];

	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
		{
					s1=worldCordinateSystem(j+x_size,i*cos(gamma)+size*sin(gamma)+y_size,i*(-sin(gamma))+size*cos(gamma)+z_size);	//top fill
					drawPixel(s1.x,s1.y,RED);

					s1=worldCordinateSystem(i+x_size,size*cos(gamma)+j*sin(gamma)+y_size,size*(-sin(gamma))+j*cos(gamma)+z_size);	// right fill
					drawPixel(s1.x,s1.y,PURPLE);

	            	s1=worldCordinateSystem(size+x_size,j*cos(gamma)+i*sin(gamma)+y_size,j*(-sin(gamma))+i*cos(gamma)+z_size);		// left fill
					drawPixel(s1.x,s1.y,BLUE);
		}
	}
}

//Random function to generate values between range of numbers
unsigned int random_generator(unsigned int min, unsigned int max){

         double scaled = (double)rand()/RAND_MAX;

         return (max - min +1)*scaled + min;
}

/******************************************************************************
 Main Function  main()
******************************************************************************/
int main (void)
{
  //EINTInit();
  uint32_t i, portnum = PORT_NUM;
  portnum = 0 ; /* For LCD use 1 */
  /* SystemClockUpdate() updates the SystemFrequency variable */
  //  SystemClockUpdate();
  if ( portnum == 0 )
  SSP0Init();               /* initialize SSP port */
  else if ( portnum == 1 )
  SSP1Init();
  for ( i = 0; i < SSP_BUFSIZE; i++ ){

    src_addr[i] = (uint8_t)i;
    dest_addr[i] = 0;
  }

//initialize LCD
  lcd_init();
  fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, WHITE);
  fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, LTBLUE);


  	  	sCam.xe=100;
	  	sCam.ye=100;
	  	sCam.ze=100;

	  	sW=worldCordinateSystem(0,0,0);
	  	drawWorldCordinates();

//For Cube1

	  	int x_displacement,y_displacement,z_displacement,cube_size;
	  	x_displacement = 60;
		y_displacement = 0;
		z_displacement = 40;
		cube_size = 25;

	  	//Drawing Cube1 and performing its linear decoration
		drawCube(x_displacement,y_displacement,z_displacement,cube_size);
	  	cubefill(x_displacement,y_displacement,z_displacement,cube_size);

	  	//Generating Trees on First Surface
	  	int loop;
	  	for (loop=0;loop<4;loop++){
	  		  	struct world t1,t2;
	  		  	int a = random_generator(0,3);
	  		  	t1=worldCordinateSystem(12+a+x_displacement,3*cube_size/4+5+y_displacement,-29+a+20+z_displacement);
	  		  	t2=worldCordinateSystem(12+a+x_displacement,3*cube_size/4+5+y_displacement,-19+a+20+z_displacement);
	  		  	tree(t1.x, t1.y, t2.x, t2.y, GREEN, 3);
	  	  }

	  	// Generating Square patterns on Second Surface
	  	 int col_option;
	  	 uint32_t colour;

	  	 col_option = rand()%13 + 1;
	  	 colour = returnColour(col_option);

	  	 int square_side = 3*cube_size/4;
	  	 struct Location start_vertex = {0,0};

	  	 Square_pattern_cube(start_vertex, square_side, YELLOW,x_displacement,y_displacement,z_displacement, cube_size);
	  	 start_vertex.x = 2;
	  	 start_vertex.y = 7;

	  	 Square_pattern_cube(start_vertex, square_side, GREEN,x_displacement,y_displacement,z_displacement, cube_size);
	  	 start_vertex.x = 5;
	  	 start_vertex.y = 10;
	  	 Square_pattern_cube(start_vertex, square_side, BLUE,x_displacement,y_displacement,z_displacement, cube_size);

	  	// Generating Alphabet on Top Surface
	  	alphabet(x_displacement,y_displacement,z_displacement,cube_size);

// For Cube2

	    x_displacement =10;
	  	y_displacement =65;
	  	z_displacement =30;
	  	cube_size =25;

	  	//Drawing Cube2 and performing its linear decoration
	  	drawCube(x_displacement,y_displacement,z_displacement,cube_size);
	  	cubefill(x_displacement,y_displacement,z_displacement,cube_size);

	  	//Generating Trees on First Surface
	  	for(loop=0;loop<4;loop++){
	  		  	struct world t1,t2;
	  		  	int a = random_generator(0,3);
	  		  	t1=worldCordinateSystem(12+a+x_displacement,3*cube_size/4+5+y_displacement,-29+a+20+z_displacement);
	  		    t2=worldCordinateSystem(12+a+x_displacement,3*cube_size/4+5+y_displacement,-19+a+20+z_displacement);
	  		  	tree(t1.x, t1.y, t2.x, t2.y, GREEN, 3);

	  	    }

	  	// Generating Square patterns on Second Surface
	  	col_option = rand()%13 + 1;
	  	colour = returnColour(col_option);

	  	square_side = 3*cube_size/4;

	  	Square_pattern_cube(start_vertex, square_side, BLACK,x_displacement,y_displacement,z_displacement,cube_size);
	  	start_vertex.x = 2;
	  	start_vertex.y = 7;
	  	Square_pattern_cube(start_vertex, square_side, ORANGE,x_displacement,y_displacement,z_displacement, cube_size);
	  	start_vertex.x = 5;
	  	start_vertex.y = 10;
	  	Square_pattern_cube(start_vertex, square_side, BLUE,x_displacement,y_displacement,z_displacement, cube_size);

	  	// Generating Alphabet on Top Surface
		alphabet(x_displacement,y_displacement,z_displacement,cube_size);

//For Tilted Cube

	  	x_displacement =0;
	    y_displacement =0;
	  	z_displacement =30;
	  	cube_size =30;

	  	//Drawing Tilted Cube and performing its linear decoration
	  	RotationMatrix(cube_size);
	  	drawTiltedCube(x_displacement,y_displacement,z_displacement,cube_size);
	  	cubefill_tilted(x_displacement,y_displacement,z_displacement,cube_size);

	  	//Generating Trees on First Surface
	  	for (loop=0;loop<4;loop++){
	  		  	struct world t1,t2;
	  		  	int a = random_generator(0,2);
	  		  	double gamma = PI/9;
	  		  	t1=worldCordinateSystem(6+a+x_displacement,((32*cos(gamma))+((-12+a)*sin(gamma)))+y_displacement,((32*(-sin(gamma)))+((-12+a)*cos(gamma)))+z_displacement);
	  			t2=worldCordinateSystem(6+a+x_displacement,((32*cos(gamma))+((-4+a)*sin(gamma)))+y_displacement,((32*(-sin(gamma)))+((-4+a)*cos(gamma)))+z_displacement);
	  		  	tree(t1.x, t1.y, t2.x, t2.y, GREEN, 2);
	  		}

	  	// Generating Square patterns on Second Surface
	  	col_option = rand()%13 + 1;
	  	colour = returnColour(col_option);

	  	square_side = 20;

	  	//Square_pattern_Tiltedcube(start_vertex, square_side, RED,x_displacement,y_displacement,z_displacement,cube_size);
	  	start_vertex.x = 0;
	  	start_vertex.y = 0;
	  	Square_pattern_Tiltedcube(start_vertex, square_side, PURPLE,x_displacement,y_displacement,z_displacement, cube_size);
	  	start_vertex.x = 2;
	  	start_vertex.y = 3;
	  	Square_pattern_Tiltedcube(start_vertex, square_side, ORANGE,x_displacement,y_displacement,z_displacement, cube_size);

	  	// Generating Alphabet on Top Surface
	  	alphabet_tilted(x_displacement,y_displacement,z_displacement,cube_size);


	  	return 0;

}

#endif
