//-----------------------------------------------------
// RADAR.C: 
// TRACKING OBJECTS PROGRAM
//-----------------------------------------------------
#include <stdlib.h> // include standard lib first
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <allegro.h>
#include <math.h>
#include <time.h>
#include "ptask.h"  // a lib for periodic tasks
//-------------------------------------------------------------
// GLOBAL CONSTANTS
//-------------------------------------------------------------
#define XWIN 864 // window x resolution
#define YWIN 600 // window y resolution
#define BKG 0    // background color
#define WHITE 15    // white color
//-----------------------------------------------------
//-------------------------------------------------------------
#define MAXT 50 // max number of balls tasks
#define LEN 80 // max message length
#define PER 20 // base period
#define PERRADAR 5 // base period
#define PERRANDOMIZER 5000 //
#define PI 3.1415926536 // pi greco
#define RR 280.0 // radar display radius
#define RRMIN  0 // min radius radar
#define RRMAX  280 // max radius radar
#define RSTEP  1 
//-----------------------------------------------------
// BALL CONSTANTS
//-----------------------------------------------------
#define MAX_BALLS 20 // max number of tracked balls
#define MAX_TRACK_SAMPLES 10 // max number of sample per track
#define VMIN -0.6 // min initial hor. speed
#define VMAX 0.6 // max initial hor. speed
#define AMIN -0.02 //  -0.2  min initial hor. speed
#define AMAX 0.02 //   0.2 max initial hor. speed
#define TSCALE 5 // time scale factor
#define RMIN  1 // min radius
#define RMAX  1 // max radius
//-----------------------------------------------------
// TRACKING TASK CONSTANTS
//-----------------------------------------------------
#define N 10 // max number of tracking measures per object
#define NOT_TRACKING 0
#define TRACKING 1
#define TRACKING_END_OF_BURST 2
#define DISTANCE_TRHESHOLD 50  // maximum radius of recognition
//                                artifacts if too little
#define ON  1
#define OFF 2
#define SHUTTING_DOWN 3
//-------------------------------------------------------------
// GLOBAL VARIABLES
//-------------------------------------------------------------

int end = 0; // end flag
int tasks = 0; // number of balls tasks
int k_samples = 0; // selected at run time by user


struct status  // ball structure
{ 
	int out_of_radar;
	int c; // color [1,15]
	float r; // radius (m)
	float x; // x coordinate (m)
	float y; // y coordinate (m)
	float x_old; // x coordinate (m)
	float y_old; // y coordinate (m)
	float vx; // horizontal velocity (m/s)
	float vy; // vertical velocity (m/s)
	float al; // longitudal acceleration (m/s^2)
	float ac; // centripetal acceleration (m/s^2)
};
struct status ball[MAXT]; // balls status buffer

BITMAP *buffer;
BITMAP *sky;
int     width = YWIN;
int     height = YWIN;

struct misure_t  // contains newest interceptions
{           
	pthread_cond_t ptrt[MAX_BALLS];
	int object[MAX_BALLS] ; //
	int ix[MAX_BALLS]; // last x ccordinate intercepted by beam
	int iy[MAX_BALLS];
	clock_t ct[MAX_BALLS];  // time of hit
	int burst[MAX_BALLS];   // true when there are open burst
} m;

struct track_t // contains positions computed by tracking tasks
{           
	int status[MAX_BALLS];
	float ix[MAX_BALLS][MAX_TRACK_SAMPLES];
	float iy[MAX_BALLS][MAX_TRACK_SAMPLES];
	int oldest_index[MAX_BALLS];
	int newest_index[MAX_BALLS];
	float prvs_prdct_ix[MAX_BALLS];  //previous predicted ix
	float prvs_prdct_iy[MAX_BALLS];  //previous predicted iy
	float prdct_ix[MAX_BALLS];
	float prdct_iy[MAX_BALLS];
	int prdct_flag[MAX_BALLS];
} t;

pthread_mutex_t mutex;
pthread_mutexattr_t attr;
pthread_condattr_t cattr;


void init(void)
{
	int i,j;
	char s[LEN];
	allegro_init();
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN, 0, 0);
	clear_to_color(screen, BKG);
	install_keyboard();
	ptask_init(SCHED_FIFO, GLOBAL, PRIO_INHERITANCE);
	buffer = create_bitmap(width, height);
	clear_bitmap(buffer);
	sky = create_bitmap(width, height);
	clear_bitmap(sky);

	for (i=0; i<MAXT; i++)
	{
		ball[i].out_of_radar = TRUE;
	}

	pthread_condattr_init(&cattr);
	pthread_mutex_init(&mutex, &attr);
	for (i=0; i<MAX_BALLS; i++)
	{
		m.object[i] = 0;
		m.burst[i] = FALSE;
		pthread_cond_init(&m.ptrt[i], &cattr);
		t.status[i] = OFF;
		t.oldest_index[i] = 0;
		t.newest_index[i] = 0;
	}
}
//----------------------------------------------------
// FRAND: returns a random float in [xmi,xma)
//----------------------------------------------------
float frand(float xmi, float xma)
{
	float r;
	r = rand() / (float)RAND_MAX; //rand in [0,1)
	return xmi + (xma - xmi) * r;
}

double distance(int x1, int y1, int x2, int y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)); 
}


void init_ball(int i, float dt)
{
	float alpha;
	float x, y;
	ball[i].out_of_radar = FALSE;
	ball[i].c = 2 + i % 14; // color in [2, 15]
	ball[i].r = frand(RMIN, RMAX);
	alpha = 2 * PI * frand(0.0, 1.0);
	ball[i].x = YWIN/2 + (RR-.5) * cos(alpha);
	ball[i].y = YWIN/2 + (RR-.5) * sin(alpha);
	ball[i].vx = frand(VMIN, VMAX);
	ball[i].vy = frand(VMIN, VMAX);
	ball[i].al = 0;
	ball[i].ac = 0;
	x= ball[i].x + ball[i].vx * 50 * dt;
	y= ball[i].y + ball[i].vy * 50 * dt;
//
// if initial random speed is outbound then inversion
//
	if (distance(YWIN/2, YWIN/2, x,y) > distance(YWIN/2, YWIN/2, ball[i].x,
	 ball[i].y))
	{
		ball[i].vx *= -1;
		ball[i].vy *= -1;
	}
}

//----------------------------------------------------
// DRAW_BALL: draw ball i in graphic coordinates
//----------------------------------------------------
void draw_ball(int i)
{
	int x, y;
	int x_old, y_old;
	x =  ball[i].x;
	y = YWIN - ball[i].y;
	x_old = ball[i].x_old;
	y_old = YWIN - ball[i].y_old;
	circlefill(sky, x_old, y_old, ball[i].r, BKG);
	circlefill(sky, x, y, ball[i].r, ball[i].c);
}

void clear_ball(int i)
{
	int x_old, y_old;
	x_old = ball[i].x_old;
	y_old = YWIN - ball[i].y_old;
	circlefill(sky, x_old, y_old, ball[i].r, BKG);
	circle(sky, YWIN/2, YWIN/2, RR, 50);
}

void draw_arrow(int x1, int y1, int x2, int y2, int c){
	if (distance(x2, y2, YWIN/2, YWIN/2) <= RR)
	{
		line(buffer, x1, y1, x2, y2, c);
	}
}

void ball_task()
{
	char s[LEN];
	double ax;  // acceleration
	double ay;
	double vel_modulus;
	double vel_modulus_after;
	double alpha;

	int i; // task index
	i = ptask_get_index();
	float dt; // integration interval

	dt = TSCALE*(float) ptask_get_period(i, MILLI)/1000;

	init_ball(i, dt);



	while (!end && !ball[i].out_of_radar) {
		vel_modulus = sqrt(ball[i].vx*ball[i].vx+ball[i].vy*ball[i].vy);
		alpha = asin(ball[i].vy/vel_modulus); //direction of speed vector

		ax = ball[i].ac*cos(alpha+PI/2); // banking
		ay = ball[i].ac*sin(alpha+PI/2);

		ball[i].vx+= ax*dt;
		ball[i].vy+= ay*dt;

		vel_modulus_after = sqrt(ball[i].vx*ball[i].vx+ball[i].vy*ball[i].vy);


		ball[i].vx *= vel_modulus/vel_modulus_after; 
		ball[i].vy *= vel_modulus/vel_modulus_after;


		ax = ball[i].al*cos(alpha);   //longitudinal acceleration
		ay = ball[i].al*sin(alpha);

		ball[i].vx += ax*dt;
		ball[i].vy += ay*dt;



		ball[i].x_old = ball[i].x;
		ball[i].y_old = ball[i].y;
		ball[i].x += ball[i].vx*dt;
		ball[i].y += ball[i].vy*dt;

		float dist = distance(ball[i].x,ball[i].y,YWIN/2,YWIN/2);

		if (dist > RR) {
			ball[i].out_of_radar = TRUE;
			clear_ball(i);
		}
		else draw_ball(i);

		ptask_wait_for_period(i);
	}
	tasks--;
	printf("    terminato task %d  colore %d coordinate %f %f\n",i,ball[i].c,ball[i].x,ball[i].y);
}




void line_scan(int x0,int y0, int a)
{
	char s[LEN];
	int lx,ly;
	int d;
	int target;
	int i;
	int j;
	float alpha;
	alpha = a * PI/1800;
	clock_t scanning_time = clock();
	double cpu_time_elapsed;
	long elapsed_clocks;


	for (i = 0;i<MAX_BALLS;i++){      // scanning array looking for too old objects
		if (m.object[i] != 0){        // and looking for end of bursts 
			elapsed_clocks = scanning_time-m.ct[i];
			cpu_time_elapsed = ((double)(elapsed_clocks))/CLOCKS_PER_SEC;
			if (elapsed_clocks>10000&&m.burst[i] == TRUE){
				pthread_mutex_lock(&mutex);
				m.burst[i] = FALSE;
				m.ct[i] = scanning_time;
				pthread_mutex_unlock(&mutex);
				pthread_cond_signal(&m.ptrt[i]);
			}
			if (cpu_time_elapsed > 3.) {
				pthread_mutex_lock(&mutex);
				m.object[i] = 0;      // signalling tracking task that contact is lost
				m.burst[i] = FALSE;
				m.ct[i] = scanning_time;
				pthread_mutex_unlock(&mutex);
				pthread_cond_signal(&m.ptrt[i]);
			}
		}
	}


	for (d = RRMIN; d<RRMAX-1; d += RSTEP){
		lx = x0 + d*cos(alpha);
		ly = y0 + d*sin(alpha);
		target = getpixel(sky,lx,ly);
		if (target>0){
			j = -1;
			for (i = 0; i < MAX_BALLS; i++){
				if (m.object[i] == 0 && j == -1) j = i; // first free index
				if (m.object[i] == target && (int)distance(lx, ly, m.ix[i],
				 m.iy[i]) < DISTANCE_TRHESHOLD  ){ // hit known object
					j=i;
					break;
				}
			}
			if (j != -1){
				pthread_mutex_lock(&mutex);
				m.object[j] = target;
				m.ix[j] = lx;
				m.iy[j] = ly;
				m.ct[j] = scanning_time;
				m.burst[j] = TRUE;
				pthread_mutex_unlock(&mutex);
				pthread_cond_signal(&m.ptrt[j]);
			}
		}
	}


}

int add_index(int a, int k)
{
	if (k<0) k += MAX_TRACK_SAMPLES;
	int sidx = (a + k) % MAX_TRACK_SAMPLES;
	return sidx;
}


void radartask()
{
	int i; // task index
	int a = 0; // scanning direction (deg*10)
	int j; // array index
	int k;
	int sidx;   // sample index
	int filling_colour;
	int samples_to_display;
	int radius;
	float alpha = 0;

	i = ptask_get_index();
	while (!end) {


		alpha = a * PI /1800;
		line(buffer, YWIN/2, YWIN/2, YWIN/2 + RR * cos(alpha), YWIN/2 + RR*sin(alpha),
		BKG);

		a = a + 5;
		if (a == 3600) a = 0;

		line_scan(YWIN/2, YWIN/2, a);

		alpha = a * PI /1800;
		line(buffer, YWIN/2, YWIN/2, YWIN/2 + RR * cos(alpha),
		YWIN/2 + RR*sin(alpha), WHITE);

// for (j = 0;j<MAX_BALLS;j++){
//    if (m.object[j] != 0) circlefill(buffer, m.ix[j], m.iy[j],3,m.object[j]);
//  }

		for (j = 0; j<MAX_BALLS; j++){
			samples_to_display = t.newest_index[j] - t.oldest_index[j]+1;
			if (samples_to_display <= 0) samples_to_display = MAX_TRACK_SAMPLES;
			if (t.status[j] == OFF) samples_to_display = 0;

//            if (t.active[j] && a = =0)
//               printf("old ix %d new ix %d samples_to_display %d \n",t.oldest_ index[j],t.newest_index[j],samples_to_display);

			radius = 1;
			for (k = 0; k<samples_to_display; k++)
			{

				sidx = add_index(t.oldest_index[j], k);

				if (k == 1) draw_arrow(t.ix[j][add_index(t.newest_index[j], -1)], 
				t.iy[j][add_index(t.newest_index[j], -1)], t.prvs_prdct_ix[j],  
				t.prvs_prdct_iy[j], BKG);

				if (k == 0 && samples_to_display == MAX_TRACK_SAMPLES || 
				t.status[j] == SHUTTING_DOWN) 
				{
					circlefill(buffer,t.ix[j][sidx],t.iy[j][sidx],3,BKG);              // clearing not active tracks or samples
				}
				else 
				{
					circlefill(buffer, t.ix[j][sidx], t.iy[j][sidx], 3, 
					BKG);              // clearing before drawing needed for resizing
					circlefill(buffer, t.ix[j][sidx], t.iy[j][sidx], radius,
					m.object[j]); // drawing active tracks
				}
				if (k >= samples_to_display - 3) radius++; // newest sample has radius 3 the oldest ones a smaller radius


//            if (samples_to_display==9) circlefill(buffer,t.ix[j][t.oldest_index[j]],t.iy[j][t.oldest_index[j]],3,BKG);
//            if (t.active[j])}{
//                circlefill(buffer,t.ix[j][t.newest_index[j]],t.iy[j][t.newest_index[j]],3,m.object[j]);
//            }
			}
			if (samples_to_display >= 2)
			{
				draw_arrow(t.ix[j][t.newest_index[j]], t.iy[j][t.newest_index[j]], 
				t.prdct_ix[j], t.prdct_iy[j], m.object[j]);
			} 
			if (t.status[j]==SHUTTING_DOWN)
			{
				t.status[j] = OFF;
				t.oldest_index[j] = 0;
				t.newest_index[j] = 0;
			}
		}

		circle(buffer, YWIN/2, YWIN/2, RR, 50);


		vsync();

		blit(buffer, screen, 0, 0, 0, 0, width, height);

	ptask_wait_for_period(i);
	}
}


void get_keycodes(char *scan, char *ascii)
{
	int k;
	k = readkey();  // block until a key is pressed
	*ascii = k;     // get ascii code
	*scan = k >> 8; // get scan code
}


void get_string(char *str, int x, int y, int c, int b) {
	char ascii, scan, s[2];
	int i = 0;
	do {
	get_keycodes(&scan, &ascii);
	if (scan != KEY_ENTER) {
		s[0] = ascii; // put ascii in s for echoing
		s[1] = '\0';
		textout_ex(buffer, font, s, x, y, c, b); // echo
		x = x + 8;
		str[i++] = ascii; // insert character in string
	}
	} while (scan != KEY_ENTER);
	str[i] = '\0';
}
//
//
void compute_predicted_data(int i){
	int k;

	int samples = t.newest_index[i] - t.oldest_index[i]+1;
	if (samples <= 0) samples = MAX_TRACK_SAMPLES;

//    printf("tt %d: ",i);
//    for (k=0;k<samples;k++){
//        printf(" %d ",t.ix[i][add_index(t.oldest_index[i],k)]);
//        printf(" %d - ",t.iy[i][add_index(t.oldest_index[i],k)]);
//    }
//    printf(" pp:%d %d pr:%d %d\n",t.prvs_prdct_ix[i],t.prvs_prdct_iy[i],t.prdct_ix[i],t.prdct_iy[i]);


	t.prvs_prdct_ix[i] = t.prdct_ix[i];
	t.prvs_prdct_iy[i] = t.prdct_iy[i];
	if (samples >= k_samples && k_samples >= 2 && k_samples <= 5)
	{
		t.prdct_ix[i] = 2 * t.ix[i][t.newest_index[i]] - 
		t.ix[i][add_index(t.newest_index[i], -1)];
		t.prdct_iy[i] = 2 * t.iy[i][t.newest_index[i]]
		 - t.iy[i][add_index(t.newest_index[i], -1)];
	}


//    printf("tt %d: ",i);
//    for (k=0;k<samples;k++){
//        printf(" %d ",t.ix[i][add_index(t.oldest_index[i],k)]);
//        printf(" %d - ",t.iy[i][add_index(t.oldest_index[i],k)]);
//    }
//    printf(" pp:%d %d pr:%d %d\n",t.prvs_prdct_ix[i],t.prvs_prdct_iy[i],t.prdct_ix[i],t.prdct_iy[i]);



}
// 
//
void update_computed_data(int i, float fix, float fiy, int status, int object)
{
	float pix, piy;
	char s[LEN];
	if (status == NOT_TRACKING) 
	{
		t.status[i] = SHUTTING_DOWN;
	}
	else
	{
		if (t.status[i] == ON)
		{
			t.newest_index[i] = (t.newest_index[i] + 1) % MAX_TRACK_SAMPLES;
			if (t.newest_index[i] == t.oldest_index[i]) 
				t.oldest_index[i] = (t.oldest_index[i] + 1) % MAX_TRACK_SAMPLES;
			t.ix[i][t.newest_index[i]] = fix;
			t.iy[i][t.newest_index[i]] = fiy;
		}
		else
		{
			t.oldest_index[i] = 0;
			t.newest_index[i] = 0;
			t.ix[i][t.newest_index[i]] = fix;
			t.iy[i][t.newest_index[i]] = fiy;
			t.status[i] = ON;
		}
	}


	compute_predicted_data(i);

	pix = t.prdct_ix[i];
	piy = t.prdct_iy[i];

	sprintf(s, "Coordinates");
	textout_ex(screen, font, s, 600, 10, 14, BKG);
	sprintf(s, "    Measured       Predicted");
	textout_ex(screen, font, s, 600, 20, 14, BKG);
	sprintf(s, "%2d) %6.2f %6.2f  %6.2f %6.2f", i, fix, fiy, pix, piy);  //,t.newest_index[i]
	textout_ex(screen, font, s, 600, 30+10*i, object, BKG);
}


void path_randomizer()
{
	int i;
	int k;
	int r;
	float m;
	float vel_modulus;
	i = ptask_get_index();
	while(!end)
	{
		for (k = 0; k < MAXT; k++)
		{
			if(!ball[k].out_of_radar)
			{
				vel_modulus = sqrt(ball[k].vx * ball[k].vx + ball[k].vy * ball[k].vy);
				r = rand()%5;
				switch(r)
				{
					case 0:
						ball[k].al = 0;
						ball[k].ac = 0;
						break;
					case 1:
						ball[k].al = 0;
						ball[k].ac = 0;
						break;
					case 2: // banking
						if (ball[k].al == 0 && ball[k].ac == 0 )
						{
							ball[k].ac = frand(AMIN, AMAX);
							ball[k].al = 0;
						}
						break;
					case 3: // acceleration
						if (ball[k].al == 0 && ball[k].ac == 0 ){
							if (vel_modulus < VMAX) ball[k].al = frand(0, AMAX);
							ball[k].ac = 0;
						}
						break;
					case 4: // braking
						if (ball[k].al == 0 && ball[k].ac == 0 ){
							if (vel_modulus > VMAX/4) ball[k].al = frand(AMIN, 0);
							ball[k].ac = 0;
						}
						break;

				}
				printf("acceleration k %d  al: %f ac: %f vel: %f\n", k,
				 ball[k].al, ball[k].ac, vel_modulus);
			}
		}
		ptask_wait_for_period(i);
	}
}



void * tracking_task(void* arg){
	int i = *(int*)arg;
	int status = NOT_TRACKING;
	int samples = 0;
	int object;                              // colour of tracked object
	int ix;
	int iy;
	float fix;
	float fiy;
	int burst;
	clock_t ct;
	clock_t prior_scanning_time = 0;
	int nr_sub_samples = 0;
	int sub_samples_x[10];
	int sub_samples_y[10];
	char ss[LEN];
	printf("%s\n", ss);
	while(!end)
	{
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&m.ptrt[i], &mutex);
			object = m.object[i];
			ix = m.ix[i];                          // copy of shared data 
			iy = m.iy[i];
			burst = m.burst[i];
			ct = m.ct[i];
		pthread_mutex_unlock(&mutex);


		if (object != 0)
		{
			if (burst && nr_sub_samples < 10)
			{
				status = TRACKING;
				sub_samples_x[nr_sub_samples] = ix;
				sub_samples_y[nr_sub_samples] = iy;
				nr_sub_samples++;
			}
			if (!burst)
			{
				status = TRACKING_END_OF_BURST;
			}
		}
		else
		{
			status = NOT_TRACKING;
			samples = 0;
			nr_sub_samples = 0;
			update_computed_data(i, fix, fiy, status, object);
		}


//        sprintf(ss, "intercettato %d scanning time %lu %ld %d %d %d",i,ct,(ct-prior_scanning_time),ix,iy,burst);
//        textout_ex(buffer, font, ss, 10, 30, 3, 0);
//        printf("%s\n",ss);
//        printf("status %d nr_sub_samples %d \n",status,nr_sub_samples);

		prior_scanning_time = ct;


		if (status == TRACKING_END_OF_BURST)
		{
			fix = 0.;
			fiy = 0.;
			for (int k = 0; k < nr_sub_samples; k++) 
			{
				fix += sub_samples_x[k];
				fiy += sub_samples_y[k];
			}
			fix = fix/nr_sub_samples;
			fiy = fiy/nr_sub_samples;
//            sprintf(ss, "Intercettato %d nr_sub_samples %d  coordinate medie %.2f %.2f",i,nr_sub_samples,fix,fiy);
//            printf("%s\n",ss);
			nr_sub_samples = 0;
//
//      updating shared area for display
//
			update_computed_data(i, fix,fiy, status, object);
		}

	}
	return 0;

}


int main(void)
{
	int i;
	int j;
	int x;
	char ss[10];
	char str[5];
	char scan;
	pthread_attr_t a;
	pthread_t p;

	init();

	pthread_attr_init(&a);

	pthread_attr_setdetachstate(&a, PTHREAD_CREATE_DETACHED);
	int balls[MAX_BALLS];
	for (j = 0; j < MAX_BALLS; j++)
	{
		balls[j] = j;
		int res = pthread_create(&p, &a, tracking_task, (void*)&balls[j]);
	}


	i = ptask_create_prio(radartask, PERRADAR, 70, NOW);
	i = ptask_create_prio(path_randomizer, PERRANDOMIZER, 10, NOW);
	textout_ex(buffer, font, "Press I for entering k ", 10, 30, 3, 0);
	textout_ex(buffer, font, "Press SPACE to create a moving object", 10, 10,
	 11, BKG);

	tasks = 0;

	do 
	{
		scan = 0;
		if (keypressed()) scan = readkey() >> 8;

		if (scan == KEY_SPACE)
		{
			if  (tasks < MAXT) 
			{
				i = ptask_create_prio(ball_task, PER, 50, NOW);
				tasks++;
				printf("            creato task %d nr totale task palle %d\n",
				 i, tasks);
			} else 
			{
				printf("            IMPOSSIBILE CREARE NUOVI TASKS raggiunto nr max %d \n", 
				tasks);
			}
		}
		if (scan == KEY_I)
		{
			textout_ex(buffer, font, "Enter k:          ", 10, 50, 2, BKG);
			textout_ex(buffer, font, "                ", 10, 60, 2, BKG);
			get_string(str, 80, 50, 3, 0);
			sscanf(str, "%d", &k_samples);
			if (k_samples < 2) textout_ex(buffer, font, "too little      ", 10,
			 60, 2, BKG);
			if (k_samples > 5) textout_ex(buffer, font, "too big         ", 10,
			 60, 2, BKG);
			if (k_samples >= 2 && k_samples <= 5)
			{
				sprintf(ss, "Accepted : %d", k_samples);
				textout_ex(buffer, font, ss, 10, 60, 2, BKG);
			}
		}

		sleep(0.2);
	} while (scan != KEY_ESC);
	end = 1;

	sleep(1); /* se i task non finiscono il processo va in segmentation fault*/

/*  wait_for_task_end(i);       */

	allegro_exit();
	return 0;
}

 
