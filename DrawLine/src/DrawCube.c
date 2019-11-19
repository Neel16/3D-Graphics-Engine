/*
===============================================================================
 Name        : DrawCube.c
 Author      : Himanshu Gunjal(Adding on Prof. Harry Hua Li's code template)
===============================================================================
*/
#include <cr_section_macros.h>
#include <NXP/crp.h>
#include "LPC17xx.h"                        /* LPC17xx definitions */
#include "ssp.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Be careful with the port number and location number, because

 some of the location may not exist in that port. */

#define PORT_NUM            0

uint8_t src_addr[SSP_BUFSIZE];
uint8_t dest_addr[SSP_BUFSIZE];

#define ST7735_TFTWIDTH 127
#define ST7735_TFTHEIGHT 159

#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_SLPOUT 0x11
#define ST7735_DISPON 0x29

//
#define swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }

struct coordinate_pnt {
	int x;
	int y;
};

struct coordinates{
int x;
int y;
};

int cam_x = 90;
int cam_y = 90;
int cam_z = 125;
int light_x = 50;
int light_y = 50;
int light_z = 60;

#define swap(x, y) {x = x + y; y = x - y; x = x - y ;}

// defining color values


#define B_BLACK 0x000000
#define LT_GRAY 0xF6F3F3
#define GREEN 0x00FF00
#define LIGHTGREEN 0x80FF00
#define DARKBLUE 0x000033
#define BLACK 0x000000
#define YELLOW 0x666600
#define BLUE 0x0007FF
#define RED 0xFF0000
#define LIGHTRED 0xFF3333
#define BROWN 0xA52A2A
#define LIGHTBLUE 0x0080FF
#define DARK_GREEN 0x006400
#define AZURE 0xF0FFFF

#define random(x) (rand()%x)

int _height = ST7735_TFTHEIGHT;
int _width = ST7735_TFTWIDTH;
double mySin(double a);
double myCos(double a);

void spiwrite(uint8_t c)
{
	int pnum = 0;
	src_addr[0] = c;
	SSP_SSELToggle(pnum, 0);
	SSPSend(pnum, (uint8_t *) src_addr, 1);
	SSP_SSELToggle(pnum, 1);
}

void writecommand(uint8_t c)
{
	LPC_GPIO0->FIOCLR |= (0x1 << 21);
	spiwrite(c);
}

void writedata(uint8_t c)
{
	LPC_GPIO0->FIOSET |= (0x1 << 21);
	spiwrite(c);
}

void writeword(uint16_t c)
{
	uint8_t d;
	d = c >> 8;
	writedata(d);
	d = c & 0xFF;
	writedata(d);
}

void write888(uint32_t color, uint32_t repeat)
{
	uint8_t red, green, blue;
	int i;
	red = (color >> 16);
	green = (color >> 8) & 0xFF;
	blue = color & 0xFF;
	for (i = 0; i < repeat; i++)
	{
		writedata(red);
		writedata(green);
		writedata(blue);
	}
}

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	writecommand(ST7735_CASET);
	writeword(x0);
	writeword(x1);
	writecommand(ST7735_RASET);
	writeword(y0);
	writeword(y1);
}

void fillrect(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)
{
	int16_t width, height;
	width = x1 - x0 + 1;
	height = y1 - y0 + 1;
	setAddrWindow(x0, y0, x1, y1);
	writecommand(ST7735_RAMWR);
	write888(color, width * height);
}

void lcddelay(int ms)
{
	int count = 24000;
	int i;
	for (i = count * ms; i--; i > 0);
}

void lcd_init()
{
	int i;
	printf("LCD Demo Begins!!!\n");
	// Set pins P0.16, P0.21, P0.22 as output
	LPC_GPIO0->FIODIR |= (0x1 << 16);
	LPC_GPIO0->FIODIR |= (0x1 << 21);
	LPC_GPIO0->FIODIR |= (0x1 << 22);

	// Hardware Reset Sequence
	LPC_GPIO0->FIOSET |= (0x1 << 22);
	lcddelay(500);

	LPC_GPIO0->FIOCLR |= (0x1 << 22);
	lcddelay(500);

	LPC_GPIO0->FIOSET |= (0x1 << 22);
	lcddelay(500);

	// initialize buffers
	for (i = 0; i < SSP_BUFSIZE; i++)
	{
		src_addr[i] = 0;
		dest_addr[i] = 0;
	}

	// Take LCD display out of sleep mode
	writecommand(ST7735_SLPOUT);
	lcddelay(200);

	// Turn LCD display on
	writecommand(ST7735_DISPON);
	lcddelay(200);
}

void draw_pixel(int16_t x, int16_t y, uint32_t color)
{
	if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
		return;

	setAddrWindow(x, y, x + 1, y + 1);
	writecommand(ST7735_RAMWR);
	write888(color, 1);
}

/*****************************************************************************


 ** Descriptions:        Draw line function

 **

 ** parameters:           Starting point (x0,y0), Ending point(x1,y1) and color

 ** Returned value:        None

 **

 *****************************************************************************/

void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)
{
	int16_t slope = abs(y1 - y0) > abs(x1 - x0);
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
	int16_t ystep;
	if (y0 < y1)
	{
		ystep = 1;
	}
	else
	{
		ystep = -1;
	}
	for (; x0 <= x1; x0++)
	{
		if (slope)
		{
			draw_pixel(y0, x0, color);
		}
		else
		{
			draw_pixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

struct coordinate_pnt project_coordinates (int x_w, int y_w, int z_w)
{
	int scrn_x, scrn_y, Dist=100, x_diff=ST7735_TFTWIDTH/2, y_diff=ST7735_TFTHEIGHT/2;
	double x_p, y_p, z_p, theta, phi, rho;
	struct coordinate_pnt screen;
	theta = acos(cam_x/sqrt(pow(cam_x,2)+pow(cam_y,2)));
	phi = acos(cam_z/sqrt(pow(cam_x,2)+pow(cam_y,2)+pow(cam_z,2)));

	rho= sqrt((pow(cam_x,2))+(pow(cam_y,2))+(pow(cam_z,2)));
	x_p = (y_w*cos(theta))-(x_w*sin(theta));
	y_p = (z_w*sin(phi))-(x_w*cos(theta)*cos(phi))-(y_w*cos(phi)*sin(theta));
	z_p = rho-(y_w*sin(phi)*cos(theta))-(x_w*sin(phi)*cos(theta))-(z_w*cos(phi));
    scrn_x = x_p*Dist/z_p;
	scrn_y = y_p*Dist/z_p;
	scrn_x = x_diff+scrn_x;
	scrn_y = y_diff-scrn_y;
	screen.x = scrn_x;
	screen.y = scrn_y;
	return screen;
}

void draw_coordinates ()
{
	struct coordinate_pnt lcd;
	int x1,y1,x2,y2, x3,y3,x4,y4;
	lcd = project_coordinates (0,0,0);
	x1=lcd.x;
	y1=lcd.y;
	lcd = project_coordinates (180,0,0);
	x2=lcd.x;
	y2=lcd.y;
	lcd = project_coordinates (0,180,0);
	x3=lcd.x;
	y3=lcd.y;
	lcd = project_coordinates (0,0,180);
	x4=lcd.x;
	y4=lcd.y;

	draw_line(x1,y1,x2,y2,RED);	//x axis  red
	draw_line(x1,y1,x3,y3,DARK_GREEN);	//y axis  green
	draw_line(x1, y1, x4, y4,BLUE);  	//z axis  blue
}

void rotate_point(int *x, int *y, float angle)
{
	int temp_x = *x , temp_y = *y;
	angle = angle*(3.14285/180);
	float cos_rotation = cos(angle);
	float sin_rotation = sin(angle);

	*x = temp_x*cos_rotation - temp_y*sin_rotation;
	*y = temp_x*sin_rotation + temp_y*cos_rotation;
}

void draw_cube(int start_pnt, int size)
{
	struct coordinate_pnt lcd;
	int x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7;
	cam_x = 120;
	cam_y = 120;
	cam_z = 120;

	lcd = project_coordinates (start_pnt,start_pnt,(size+start_pnt));
	x1=lcd.x;
	y1=lcd.y;
	lcd = project_coordinates ((size+start_pnt),start_pnt,(size+start_pnt));
	x2=lcd.x;
	y2=lcd.y;
	lcd = project_coordinates ((size+start_pnt),(size+start_pnt),(size+start_pnt));
	x3=lcd.x;
	y3=lcd.y;
	lcd = project_coordinates (start_pnt,(size+start_pnt),(size+start_pnt));
	x4=lcd.x;
	y4=lcd.y;
	lcd = project_coordinates ((size+start_pnt),start_pnt,start_pnt);
	x5=lcd.x;
	y5=lcd.y;
	lcd = project_coordinates ((size+start_pnt),(size+start_pnt),start_pnt);
	x6=lcd.x;
	y6=lcd.y;
	lcd = project_coordinates (start_pnt,(size+start_pnt),start_pnt);
	x7=lcd.x;
	y7=lcd.y;
	draw_line(x1, y1, x2, y2,B_BLACK);
	draw_line(x2, y2, x3, y3,B_BLACK);
	draw_line(x3, y3, x4, y4,B_BLACK);
	draw_line(x4, y4, x1, y1,B_BLACK);
	draw_line(x2, y2, x5, y5,B_BLACK);
	draw_line(x5, y5, x6, y6,B_BLACK);
	draw_line(x6, y6, x3, y3,B_BLACK);
	draw_line(x6, y6, x7, y7,B_BLACK);
	draw_line(x7, y7, x4, y4,B_BLACK);
}

void draw_rotated_cube(int start_x, int start_y ,int start_z, int size,float angle)
{
	struct coordinate_pnt lcd;
	int x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7,x8,y8;
	int x[8],y[8],z[8];
	double xs[8] = {0}, ys[8] = {0}, zs[8] = {0};
	cam_x = 120;
	cam_y = 120;
	cam_z = 120;

	x[0] = start_x;
	y[0] = start_y;
	z[0] = start_z;
	x[1] = start_x;
	y[1] = start_y;
	z[1] = size+start_z;
	x[2] = size+start_x;
	y[2] = start_y;
	z[2] = size+start_z;
	x[3] = start_x+size;
	y[3] = start_y;
	z[3] = start_z;
	x[4] = size+start_x;
	y[4] = start_y+size;
	z[4] = start_z;
	x[5] = size+start_x;
	y[5] = size+start_y;
	z[5] = start_z+size;
	x[6] = start_x;
	y[6] = start_y+size;
	z[6] = start_z+size;
	x[7] = start_x;
	y[7] = start_y+size;
	z[7] = start_z;

	rotate_point(&x[0], &y[0], angle);
	rotate_point(&x[1], &y[1], angle);
	rotate_point(&x[2], &y[2], angle);
	rotate_point(&x[3], &y[3], angle);
	rotate_point(&x[4], &y[4], angle);
	rotate_point(&x[5], &y[5], angle);
	rotate_point(&x[6], &y[6], angle);
	rotate_point(&x[7], &y[7], angle);

	lcd = project_coordinates (x[0],y[0],z[0]);
	x1=lcd.x;
	y1=lcd.y;
	lcd = project_coordinates (x[1],y[1],z[1]);
	x2=lcd.x;
	y2=lcd.y;
	lcd = project_coordinates (x[2],y[2],z[2]);
	x3=lcd.x;
	y3=lcd.y;
	lcd = project_coordinates (x[3],y[3],z[3]);
	x4=lcd.x;
	y4=lcd.y;
	lcd = project_coordinates (x[4],y[4],z[4]);
	x5=lcd.x;
	y5=lcd.y;
	lcd = project_coordinates (x[5],y[5],z[5]);
	x6=lcd.x;
	y6=lcd.y;
	lcd = project_coordinates (x[6],y[6],z[6]);
	x7=lcd.x;
	y7=lcd.y;
	lcd = project_coordinates (x[7],y[7],z[7]);
	x8=lcd.x;
	y8=lcd.y;
	draw_line(x1, y1, x2, y2,B_BLACK);
	draw_line(x1, y1, x4, y4,B_BLACK);
	draw_line(x1, y1, x8, y8,B_BLACK);
	draw_line(x2, y2, x3, y3,B_BLACK);
	draw_line(x2, y2, x7, y7,B_BLACK);
	draw_line(x6, y6, x3, y3,B_BLACK);
	draw_line(x5, y5, x6, y6,B_BLACK);
	draw_line(x4, y4, x5, y5,B_BLACK);
	draw_line(x5, y5, x8, y8,B_BLACK);
	draw_line(x3, y3, x4, y4,B_BLACK);
	draw_line(x6, y6, x7, y7,B_BLACK);
	draw_line(x8, y8, x7, y7,B_BLACK);

	fill_Triangle(x1, y1, x1-10, y1+5, x4-10, y4+10, BLACK);
	fill_Triangle(x1, y1, x4, y4, x4-10, y4+10, BLACK);
	draw_shadow(xs, ys, zs, size, -1000, 0, 1000);

}

void draw_square(int angle)
{
	int x0,y0,y1,x1,x2,y2,x3,y3,size,intColor=0,i=0;
	struct coordinate_pnt lcd;
	uint32_t color, colorArray [12]={B_BLACK,0x0000FFFF,0x00FF007F,0x00FF8000,0x0000FF80,0x000000FF,0x00FFFF00,0x00330066,0x0000FF80,0x00FF00FF,0x0000FF00,0x000080FF};
	while(i<2)
	{
		i++;
		x0 = 1+ rand() % (angle/2);
		y0=1+ rand() % (angle/2);
		size = 30 + rand() % (angle/4);
		if(intColor>12)
			intColor =0;

		color = colorArray[intColor];
		intColor++;
		x1=size+x0;

		if(x1>angle)
			x1=angle-1;

		x2=x1;
		x3=x0;
		y1=y0;
		y2=size+y1;
		if(y2>angle)
			y2=angle-1;

		y3=y2;

		lcd = project_coordinates (angle,x0,y0);
		x0=lcd.x;
		y0=lcd.y;
		lcd = project_coordinates (angle,x1,y1);
		x1=lcd.x;
		y1=lcd.y;
		lcd = project_coordinates (angle,x2,y2);
		x2=lcd.x;
		y2=lcd.y;
		lcd = project_coordinates (angle,x3,y3);
		x3=lcd.x;
		y3=lcd.y;

		draw_line(x0, y0, x1, y1,color);
		draw_line(x1, y1, x2, y2,color);
		draw_line(x2, y2, x3, y3,color);
		draw_line(x3, y3, x0, y0,color);
		//for rotation
		int it;
		for(it=0;it<3;it++)
		{
			x0=(x0+(0.4*(x1-x0)));
			y0=(y0+(0.4*(y1-y0)));
			x1=(x1+(0.4*(x2-x1)));
			y1=(y1+(0.4*(y2-y1)));
			x2=(x2+(0.4*(x3-x2)));
			y2=(y2+(0.4*(y3-y2)));
			x3=(x3+(0.4*(x0-x3)));
			y3=(y3+(0.4*(y0-y3)));

			draw_line(x0, y0, x1, y1,color);
			draw_line(x1, y1, x2, y2,color);
			draw_line(x2, y2, x3, y3,color);
			draw_line(x3, y3, x0, y0,color);
		}
	}
}

void draw_rotated_xz_square(int start_x, int start_y ,int start_z, int len, float angle)
{
	int x0,y0,y1,x1,x2,y2,x3,y3,size,intColor=0,i=0;
	struct coordinate_pnt lcd;
	int x,y;
	uint32_t color, colorArray [12]={B_BLACK,0x0000FFFF,0x00FF007F,0x00FF8000,0x0000FF80,0x000000FF,0x00FFFF00,0x00330066,0x0000FF80,0x00FF00FF,0x0000FF00,0x000080FF};
	while(i<1)
	{
		i++;
		x0 = start_x + len;
		y0 = start_z;
		size = 20;
		if(intColor>12)
			intColor =0;

		color = colorArray[intColor];
		intColor++;
		x1=size+x0;

		if(x1> start_x)
			x1=start_x;

		x2=x1;
		x3=x0;
		y1=y0;
		y2=size+y1;
		if(y2> start_z + len)
			y2= start_z + len-1;

		y3=y2;

		x = x0;
		y = start_y;
		rotate_point(&x, &y, angle);
		lcd = project_coordinates (x,y,y0);
		x0=lcd.x;
		y0=lcd.y;

		x = x1;
		y = start_y;
		rotate_point(&x, &y, angle);
		lcd = project_coordinates (x, y,y1);
		x1=lcd.x;
		y1=lcd.y;

		x = x2;
		y = start_y;
		rotate_point(&x, &y, angle);
		lcd = project_coordinates(x, y,y2);
		x2=lcd.x;
		y2=lcd.y;

		x = x3;
		y = start_y;
		rotate_point(&x, &y, angle);
		lcd = project_coordinates(x, y, y3);
		x3=lcd.x;
		y3=lcd.y;

		draw_line(x0, y0, x1, y1,color);
		draw_line(x1, y1, x2, y2,color);
		draw_line(x2, y2, x3, y3,color);
		draw_line(x3, y3, x0, y0,color);
		//for rotation
		int it;
		for(it=0;it<3;it++)
		{
			x0=(x0+(0.4*(x1-x0)));
			y0=(y0+(0.4*(y1-y0)));
			x1=(x1+(0.4*(x2-x1)));
			y1=(y1+(0.4*(y2-y1)));
			x2=(x2+(0.4*(x3-x2)));
			y2=(y2+(0.4*(y3-y2)));
			x3=(x3+(0.4*(x0-x3)));
			y3=(y3+(0.4*(y0-y3)));

			draw_line(x0, y0, x1, y1,color);
			draw_line(x1, y1, x2, y2,color);
			draw_line(x2, y2, x3, y3,color);
			draw_line(x3, y3, x0, y0,color);
		}
	}
}

void draw_tree(uint32_t color,int start_pnt, int size)
{
	int i=0, angle;
	struct coordinate_pnt lcd;
	int tree_branch[3][3]={{start_pnt,start_pnt+20,0.5*size},
			{start_pnt+10,start_pnt+20,0.3*size},
			{start_pnt+15,start_pnt+37,0.8*size}};
	while(i<2)
	{
		int x0, y0, y1, x1,xp0,xp1,yp0,yp1;
		angle = start_pnt+size;
		x0=tree_branch[i][0];
		x1=tree_branch[i][1];
		y0=tree_branch[i][2];
		y1=y0;
		i++;
		lcd = project_coordinates (y0,angle,x0);
		xp0=lcd.x;
		yp0=lcd.y;
		lcd = project_coordinates (y1,angle,x1);
		xp1=lcd.x;
		yp1=lcd.y;
		draw_line(xp0, yp0, xp1, yp1,BROWN);	//level 0 straight line
		draw_line((xp0+1), (yp0+1), (xp1+1), (yp1+1),BROWN);	//level 0 straight line
		draw_line((xp0-1), (yp0-1), (xp1-1), (yp1-1),BROWN);	//level 0 straight line

		int it=0;
		for(it=0;it<4;it++){
			int16_t x2=(0.6*(x1-x0))+x1; 	// length of level 1 = 0.8 of previous level
			int16_t y2=y1;
			lcd = project_coordinates (y2,angle,x2);
			int xp2=lcd.x;
			int yp2=lcd.y;
			draw_line(xp1, yp1, xp2, yp2,color);	//level 1 straight line

			//for right rotated angle 30 degree
			int16_t xr= ((0.134*x1)+(0.866*x2)-(0.5*y2)+(0.5*y1));
			int16_t yr=((0.5*x2)-(0.5*x1)+(0.866*y2)-(0.866*y1)+y1);
			lcd = project_coordinates (yr,angle,xr);
			int xpr=lcd.x;
			int ypr=lcd.y;

			//for left rotated angle 30 degree
			int16_t xl=((0.134*x1)+(0.866*x2)+(0.5*y2)-(0.5*y1));
			int16_t yl=((0.5*x1)-(0.5*x2)+(0.134*y2)+(0.866*y1));
			lcd = project_coordinates (yl,angle,xl);
			int xpl=lcd.x;
			int ypl=lcd.y;

			draw_line(xp1, yp1, xpr, ypr,color);
			draw_line(xp1, yp1, xpl, ypl,color);

			//for branches on right rotated branch angle 30 degree
			int16_t xrLen = sqrt(pow((xr-x1),2)+pow((yr-y1),2)) ;	//length of right branch
			int16_t xrImag= (0.8*xrLen)+xr;	//imaginary vertical line x coordinate, y= yr
			int16_t xr1 = ((0.134*xr)+(0.866*xrImag)-(0.5*yr)+(0.5*yr));
			int16_t yr1 = ((0.5*xrImag)-(0.5*xr)+(0.866*yr)-(0.866*yr)+yr);
			lcd = project_coordinates (yr1,angle,xr1);
			int xpr1=lcd.x;
			int ypr1=lcd.y;

			//for right branch
			int16_t xrr,xrl,yrr,yrl;
			xrr = ((0.134*xr)+(0.866*xr1)-(0.5*yr1)+(0.5*yr));
			yrr = ((0.5*xr1)-(0.5*xr)+(0.866*yr1)-(0.866*yr)+yr);
			lcd = project_coordinates (yrr,angle,xrr);
			int xprr=lcd.x;
			int yprr=lcd.y;

			//for left branch
			xrl = ((0.134*xr)+(0.866*xr1)+(0.5*yr1)-(0.5*yr));
			yrl = ((0.5*xr)-(0.5*xr1)+(0.134*yr)+(0.866*yr1));
			lcd = project_coordinates (yrl,angle,xrl);
			int xprl=lcd.x;
			int yprl=lcd.y;

			//for branches on left rotated branch angle 30 degree
			int16_t xlImag= (0.8*xrLen)+xl;	//imaginary vertical line x coordinate, y= yr
			int16_t xl1 = ((0.134*xl)+(0.866*xlImag)+(0.5*yl)-(0.5*yl));
			int16_t yl1 = ((0.5*xl)-(0.5*xlImag)+(0.134*yl)+(0.866*yl));
			lcd = project_coordinates (yl1,angle,xl1);
			int xpl1=lcd.x;
			int ypl1=lcd.y;

			//for right branch
			int16_t xlr,xll,ylr,yll;
			xlr = ((0.134*xl)+(0.866*xl1)-(0.5*yl1)+(0.5*yl));
			ylr = ((0.5*xl1)-(0.5*xl)+(0.866*yl1)-(0.866*yl)+yl);
			lcd = project_coordinates (ylr,angle,xlr);
			int xplr=lcd.x;
			int yplr=lcd.y;
			//for left branch
			xll = ((0.134*xl)+(0.866*xl1)+(0.5*yl1)-(0.5*yl));
			yll = ((0.5*xl)-(0.5*xl1)+(0.134*yl)+(0.866*yl1));
			lcd = project_coordinates (yll,angle,xll);
			int xpll=lcd.x;
			int ypll=lcd.y;
			draw_line(xpr, ypr, xpr1, ypr1,color);
			draw_line(xpr, ypr, xprr, yprr,color);
			draw_line(xpr, ypr, xprl, yprl,color);
			draw_line(xpl, ypl, xpl1, ypl1,color);
			draw_line(xpl, ypl, xplr, yplr,color);
			draw_line(xpl, ypl, xpll, ypll,color);

			x0=x1;
			x1=x2;
		}
	}
}

void fill_rotated_cube(int start_x, int start_y, int start_z, int size , float angle)
{
	struct coordinate_pnt s1;
	int xsize = start_x + size, ysize = start_y + size, zsize = start_z + size;
	int i,j;
	int x,y;

	for(i = start_x; i < xsize; i++)
	{
		for(j = start_y; j < ysize; j++)
		{
			x = i;
			y = j;
			rotate_point(&x, &y, angle);
			s1=project_coordinates(x,y, zsize);	//top fill green
			draw_pixel(s1.x,s1.y,LIGHTRED);

		}
	}

	for(i = start_y; i < ysize; i++)
	{
		for(j = start_z; j < zsize; j++)
		{
			x = xsize; y = i;
			rotate_point(&x, &y, angle);
			s1=project_coordinates(x, y, j);	// left fill pink
			draw_pixel(s1.x,s1.y,LIGHTBLUE);
		}
	}

	for(i = start_x; i < xsize; i++)
	{
		for(j = start_z; j < zsize; j++)
		{
			x = i;
			y = start_y;
			rotate_point(&x, &y, angle);
			s1=project_coordinates(x, y, j);	// right fill yellow
			draw_pixel(s1.x,s1.y,GREEN);
		}
	}
}

void draw_HorizontalLine(int16_t x, int16_t y, int16_t width, uint32_t color)
{
	draw_line(x, y, x+width-1, y, color);
}

void fill_Triangle(int16_t x0, int16_t y0,int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t color) {
	int16_t x, y, j, l;
	if (y0 > y1) {
		swap(y0, y1);
		swap(x0, x1);
	}
	if (y1 > y2) {
		swap(y2, y1);
		swap(x2, x1);
	}
	if (y0 > y1) {
		swap(y0, y1);
		swap(x0, x1);
	}
	if(y0 == y2) {
		x = y = x0;
		if(x1 < x) x = x1;
		else if(x1 > y) y = x1;

		if(x2 < x) x = x2;
		else if(x2 > y) y = x2;
		draw_HorizontalLine(x, y0, y-x+1, color);
		return;
	}

	int16_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0, dx12 = x2 - x1, dy12 = y2 - y1;
	int32_t sa = 0, sb = 0;

	if(y1 == y2) l = y1;
	else l = y1-1;

	for(j=y0; j<=l; j++) {
		x = x0 + sa / dy01;
		y = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;
		if(x > y) swap(x,y);
		draw_HorizontalLine(x, j, y-x+1, color);
	}
	sa = dx12 * (j - y1);
	sb = dx02 * (j - y0);
	for(; j<=y2; j++) {
		x = x1 + sa / dy12;
		y = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		if(x > y) swap(x,y);
		draw_HorizontalLine(x, j, y-x+1, color);
	}
}

void draw_shadow(double x[], double y[], double z[], int size, double xShad, double yShad, double zShad)
{
	int xs[8]={0}, ys[8]={0}, zs[8]={0};
	struct coordinate_pnt s5,s6,s7,s8;
	int i;
	for(i=4; i<8; i++){
		xs[i]=x[i]-((z[i]/(zShad-z[i]))*(xShad-x[i]));
		ys[i]=y[i]-((z[i]/(zShad-z[i]))*(yShad-y[i]));
		zs[i]=z[i]-((z[i]/(zShad-z[i]))*(zShad-z[i]));
	}
	s5 = project_coordinates (xs[4],ys[4],zs[4]);
	s6 = project_coordinates (xs[5],ys[5],zs[5]);
	s7 = project_coordinates (xs[6],ys[6],zs[6]);
	s8 = project_coordinates (xs[7],ys[7],zs[7]);

	draw_line(s5.x, s5.y, s6.x, s6.y, BLACK);
	draw_line(s6.x, s6.y, s7.x, s7.y, BLACK);
	draw_line(s7.x, s7.y, s8.x, s8.y, BLACK);
	draw_line(s8.x, s8.y, s5.x, s5.y, BLACK);

	fill_Triangle(s5.x, s5.y, s6.x, s6.y, s7.x, s7.y,BLACK);
	fill_Triangle(s5.x, s5.y, s7.x, s7.y, s8.x, s8.y,BLACK);
}
void draw_rotated_A(int start_x , int start_y , int start_z, int size , int rotated_angle)
{
	int x0,y0,x1,y1,x2,y2;
	int x3,y3,x4,y4,x5,y5;
	int x,y;
	struct coordinate_pnt p1;
	x0 = start_x + size/2;
	y0 = start_y + size/10;
	x1 = start_x + size*0.8;
	y1 = start_y + size*0.8;
	x2 = start_x + size/10;
	y2 = start_y + size*0.8;

	x = x0 ; y = y0 ;
	rotate_point(&x, &y, rotated_angle);
	p1 = project_coordinates(x, y, start_z+size);
	x3 = p1.x;
	y3 = p1.y;
	x = x1 ; y = y1 ;
	rotate_point(&x, &y, rotated_angle);
	p1 = project_coordinates(x, y, start_z+size);
	x4 = p1.x;
	y4 = p1.y;
	x = x2 ; y = y2 ;
	rotate_point(&x, &y, rotated_angle);
	p1 = project_coordinates(x, y, start_z+size);
	x5 = p1.x;
	y5 = p1.y;

	printf("Rotated values\n x3 %d y3 %d x4 %d y4 %d x5 %d y3 %d",x3,y3,x4,y4,x5,y5);
	draw_line(x3, y3, x4, y4, BLACK);
	draw_line(x3, y3, x5, y5, BLACK);
	//draw_line(x0, y0, x1, y1, BLACK);
}
void draw_H(int start_pnt, int size)
{
	struct coordinate_pnt p1;
	int i,j;
	size=size+start_pnt;
	int map[size][size];

	for(i = start_pnt; i < size;i++)
	{
		for(j = start_pnt; j < size;j++)
		{
			if(i>=6 && i<=13 && j>=6 && j<=44)
							map[i][j]=1;
			else if(i>=17 && i<=34 && j>=20 && j<=30)
							map[i][j]=1;
			else if(i>=34 && i<=41 && j>=6 && j<=44)
							map[i][j]=1;
			else
							map[i][j]=0;
		}
	}

	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
		{
			if(map[i][j]==1)
			{
				p1 = project_coordinates(j,i,size);
				draw_pixel(p1.x,p1.y,YELLOW);
			}
			else if(map[i][j]==0)
			{
				p1 = project_coordinates(j,i,size);
			}
		}
	}
}

void initial_tilted_H(int x_dist, int y_dist,int z_dist,int size)
{
	struct coordinate_pnt p1;

	int i,j;
	int map[size][size];
	double gamma = 3.14/9;

	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
		{
			if(i>=4 && i<=9 && j>=5 && j<=25)
				map[i][j]=1;
			else if( i>=9 && i<=17 && j>=13 && j<=17)   //for H
				map[i][j]=1;
			else if(i>=17 && i<=22 && j>=5 && j<=25)
				map[i][j]=1;
			else
				map[i][j]=0;
		}
	}

for(i=0;i<size;i++)
{
  for(j=0;j<size;j++)
  {
	if(map[i][j]==1)
	{
		p1= project_coordinates(j+x_dist,i*cos(gamma)+size*sin(gamma)+y_dist,i*(-sin(gamma))+size*cos(gamma)+z_dist);
		draw_pixel(p1.x,p1.y,YELLOW);
	}
  }
 }
}

int calIDiff(int16_t xPs, int16_t yPs, int16_t zPs, int16_t xPi, int16_t yPi,
	int16_t zPi, int16_t k) {
	double cosVal;
	double r = sqrt(
	pow((zPs - zPi), 2) + pow((yPs - yPi), 2) + pow((xPs - xPi), 2));
	double rcos = sqrt(pow((zPs - zPi), 2));
	cosVal = rcos / r;
	return (255 * k * cosVal) / pow(r, 2);
}

void diffused_reflection(int size)
{
	struct coordinate_pnt lcd;
	struct coordinate_pnt tmp;
	int x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6, x7, y7, x8, y8;
	lcd = project_coordinates(0, 0, 0);
	x1 = lcd.x;
	y1 = lcd.y;
	lcd = project_coordinates(size, 0, 0);
	x2 = lcd.x;
	y2 = lcd.y;
	lcd = project_coordinates(0, size, 0);
	x3 = lcd.x;
	y3 = lcd.y;
	lcd = project_coordinates(0, 0, size);
	x4 = lcd.x;
	y4 = lcd.y;
	lcd = project_coordinates(size, 0, size);
	x5 = lcd.x;
	y5 = lcd.y;
	lcd = project_coordinates(size, size, 0);
	x6 = lcd.x;
	y6 = lcd.y;
	lcd = project_coordinates(size, size, size);
	x7 = lcd.x;
	y7 = lcd.y;
	lcd = project_coordinates(0, size, size);
	x8 = lcd.x;
	y8 = lcd.y;

	for (int i = 0; i <= size; i++) {
		for (int j = 0; j <= size; j++)
		{
			tmp = project_coordinates(i, j, size);
			int kR = calIDiff(light_x, light_y, light_z, i, j, size, 255);

			if (kR + 170 > 255)
				kR = 255;
			else
				kR += 170;

			uint32_t color = 0x000000;
			color |= ((kR |= 0x000000) << 16);
			color |= (0x000000 << 8);
			color |= 0x000000;
			draw_pixel(tmp.x, tmp.y, color);
		}
	}
    for (int i = 0; i <= size; i++)
    {
    	for (int j = 0; j <= size; j++)
    	{
    	tmp = project_coordinates (size,i,j);
    	int kR = calIDiff(light_x, light_y, light_z, size, i, j, 255);
    	if (kR + 170 > 255)
    	{
    	    kR = 255;
    	    }
    	else
    	    {
    	    	kR += 170;
    	    }
    	uint32_t color = 0x000000;
    	color |= (0x000000 << 16);
    	color |= ((kR |= 0x000000) << 8);
    	color |= 0x000000;
    	draw_pixel(tmp.x,tmp.y,color);
    	}
    }
    for (int i = 0; i <= size; i++)
    {
		for (int j = 0; j <= size; j++) {
		tmp = project_coordinates (i,size,j);
		int kR = calIDiff(light_x, light_y, light_z, size, i, j, 255);
		if (kR + 170 > 255)
		{
			kR = 255;
			}
		else
			{
				kR += 170;
			}
		uint32_t color = 0x000000;
		color |= (0x000000 << 16);
		color |= (0x000000 << 8);
		color |= (kR |= 0x000000);
		draw_pixel(tmp.x,tmp.y,color);
		}
      }
}

int main(void)
{
	uint32_t pnum = PORT_NUM;

	int size = 45, start_pnt = 0;
	double x[8] = {start_pnt,(start_pnt+size),(start_pnt+size),start_pnt,start_pnt,(start_pnt+size),(start_pnt+size),start_pnt};
	double y[8] = {start_pnt, start_pnt, start_pnt+size, start_pnt+size, start_pnt, start_pnt, (start_pnt+size), (start_pnt+size) };
	double z[8] = {start_pnt, start_pnt, start_pnt, start_pnt, (start_pnt+size), (start_pnt+size), (start_pnt+size), (start_pnt+size)};

	pnum = 0;
	if (pnum == 0)
		SSP0Init();
	else
		puts("Port number is not correct");

	//To initialize LCD
	lcd_init();
	fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, LT_GRAY);

	//Draw XYZ axes
	draw_coordinates();
	//Draw shadow for first cube
	draw_shadow(x, y, z, size, -500,0,500);
	draw_cube(start_pnt,size);

	//For shifted and rotated cube
	draw_rotated_cube(20, 60, -50, 30, 90);
	fill_rotated_cube(20, 60, -50, 30, 90);
	draw_rotated_xz_square(20, 60, -50, 30, 90);

	//For shifted and tilted cube
	draw_rotated_cube(70, -60, -40, 30, 90);
	fill_rotated_cube(70, -60, -40, 30, 90);
	draw_rotated_xz_square(70, -60, -40, 30, 90);

	//Diffuse reflection on the big cube
	diffused_reflection(size);
	draw_H(start_pnt, size);
	initial_tilted_H(20, 60, -50, 30);
	draw_square(size + start_pnt);
	draw_tree(GREEN,start_pnt,size);
	initial_tilted_H(20, 70, 45, 15);

	return 0;
}
