//------------------------------------------------------------
// RADAR.C: 
// TRACKING OBJECTS PROGRAM
//------------------------------------------------------------
#include <stdlib.h>		// include standard lib first
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <allegro.h>
#include <math.h>
#include <time.h>
#include "ptask.h"		// a lib for periodic tasks
//------------------------------------------------------------------------------
// GLOBAL CONSTANTS
//------------------------------------------------------------------------------
#define XWIN 864	// window x resolution
#define YWIN 600	// window y resolution
#define BKG 0		// background color
#define WHITE 15	// white color
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define MAXT 25		// max number of balls tasks
#define LEN 80		// max message length
#define PER 20		// base period 
#define PERRADAR 5  // base period
#define PERRANDOMIZER 5000	// randomizer period
#define PI 3.1415926536
#define RR 280.0	// radar display radius
#define RRMIN  0	// min radius radar
#define RRMAX  280 	// max radius radar
#define RSTEP  1 
//------------------------------------------------------------------------------
// BALL CONSTANTS
//------------------------------------------------------------------------------
#define MAX_BALLS 20			// max number of tracked balls
#define MAX_TRACK_SAMPLES 10	// max number of sample per track
#define VMIN -0.6				// min initial hor. speed
#define VMAX 0.6				// max initial hor. speed
#define AMIN -0.02				// -0.2  min initial hor. speed
#define AMAX 0.02				// 0.2 max initial hor. speed
#define TSCALE 5				// time scale factor
#define RMIN  1					// min radius
#define RMAX  1					// max radius
//------------------------------------------------------------------------------
// TRACKING TASK CONSTANTS
//------------------------------------------------------------------------------
#define N 10					// max number of tracking measures per object
#define NOT_TRACKING 0
#define TRACKING 1
#define TRACKING_END_OF_BURST 2
#define DISTANCE_TRHESHOLD 50	// maximum radius of recognition
								// artifacts if too little
#define ON  1
#define OFF 2
#define SHUTTING_DOWN 3
//------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------

int	end = 0;		// end flag
int tasks = 0;		// number of balls tasks
int k_samples = 0;	// selected at run time by user

// ball structure
struct status {  
	int out_of_radar;			// out of radar flag
	int c;						// color [1,15]
	float r;    				// radius (m)
	float x, y, x_old, y_old;   // ball coordinates
	float vx;   				// horizontal velocity (m/s)
	float vy;   				// vertical velocity (m/s)
	float al;   				// longitudal acceleration (m/s^2)
	float ac;   				// centripetal acceleration (m/s^2)
};

struct status ball[MAXT]; // balls status buffer

// bitmap globals

BITMAP *buffer;
BITMAP *sky;
int		width = YWIN;
int     height = YWIN;

// contains newest interceptions
struct misure_t {           
	pthread_cond_t ptrt[MAX_BALLS];	// used to synchronize balls with radar 
									// task tracking
	int object[MAX_BALLS];			// used to indicate which balls are
									// currently active
									
	int ix[MAX_BALLS];  			// last x coordinate intercepted by beam
	int iy[MAX_BALLS];				// last y coordinate intercepted by beam
	clock_t ct[MAX_BALLS];  		// time of hit
	int burst[MAX_BALLS];   		// true when there are open burst
} m;

// contains positions computed by tracking tasks
struct track_t {            
	int status[MAX_BALLS];			// can be ON or OFF
	float ix[MAX_BALLS][MAX_TRACK_SAMPLES];
	float iy[MAX_BALLS][MAX_TRACK_SAMPLES];
	int oldest_index[MAX_BALLS];
	int newest_index[MAX_BALLS];
	float prvs_prdct_ix[MAX_BALLS];	// previous predicted ix
	float prvs_prdct_iy[MAX_BALLS]; // previous predicted iy
	float prdct_ix[MAX_BALLS];		// predicted ix
	float prdct_iy[MAX_BALLS];		// predicted iy
} t;

pthread_mutex_t mutex;
pthread_mutexattr_t attr;
pthread_condattr_t cattr;


//------------------------------------------------------------------------------
// INIT: initalises bitmap and condition variables
//------------------------------------------------------------------------------
void init(void) {
	int		i;
	allegro_init();
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN, 0, 0);
	clear_to_color(screen, BKG);
	install_keyboard();
	ptask_init(SCHED_FIFO, GLOBAL, PRIO_INHERITANCE);
	buffer = create_bitmap(width, height);
	clear_bitmap(buffer);
	sky = create_bitmap(width, height);
	clear_bitmap(sky);

	for (i=0; i<MAXT; i++) {
		ball[i].out_of_radar = TRUE;
	}

	pthread_condattr_init(&cattr);
	pthread_mutex_init(&mutex, &attr);

	for (i=0; i < MAX_BALLS; i++) {
		m.object[i] = 0;
		m.burst[i] = FALSE;
		pthread_cond_init(&m.ptrt[i], &cattr);
		t.status[i] = OFF;
		t.oldest_index[i] = 0;
		t.newest_index[i] = 0;
	}
}

//------------------------------------------------------------------------------
// FRAND: returns a random float in [xmi,xma)
//------------------------------------------------------------------------------
float frand(float xmi, float xma) {
	float	r;
	r = rand() / (float)RAND_MAX; //rand in [0,1)
	return xmi + (xma - xmi) * r;
}

//------------------------------------------------------------------------------
// DISTANCE: function for cartesian distance
//------------------------------------------------------------------------------
double distance(int x1, int y1, int x2, int y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)); 
}

//------------------------------------------------------------------------------
// INIT_BALL: Initialises balls position and speed
//------------------------------------------------------------------------------
void init_ball(int i, float dt) {
	float	alpha;	// angle in radiants used for the ball starting direction
	float	x, y;
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
	x = ball[i].x + ball[i].vx * 50 * dt;
	y = ball[i].y + ball[i].vy * 50 * dt;
//
// if initial random speed is outbound then inversion
//
	if (distance(YWIN/2, YWIN/2, x,y) > distance(YWIN/2, YWIN/2, ball[i].x,
	 ball[i].y)) {
		ball[i].vx *= -1;
		ball[i].vy *= -1;
	}
}

//------------------------------------------------------------------------------
// DRAW_BALL: draw ball i in graphic coordinates
//------------------------------------------------------------------------------
void draw_ball(int i) {
	int		x, y;
	int		x_old, y_old;

	x =  ball[i].x;
	y = YWIN - ball[i].y;
	x_old = ball[i].x_old;
	y_old = YWIN - ball[i].y_old;
	
	// clear the old coordinates of the ball before drawing new ones
	circlefill(sky, x_old, y_old, ball[i].r, BKG);
	circlefill(sky, x, y, ball[i].r, ball[i].c);
}

//------------------------------------------------------------------------------
// CLEAR_BALL: clears the old coordinates of the ball
//------------------------------------------------------------------------------
void clear_ball(int i) {
	int		x_old, y_old;

	x_old = ball[i].x_old;
	y_old = YWIN - ball[i].y_old;
	circlefill(sky, x_old, y_old, ball[i].r, BKG);
}

//------------------------------------------------------------------------------
// DRAW_ARROW: draw arrow pointing to predicted ball location
//------------------------------------------------------------------------------
void draw_arrow(int x1, int y1, int x2, int y2, int c){
	if (distance(x2, y2, YWIN/2, YWIN/2) <= RR) {
		line(buffer, x1, y1, x2, y2, c);
	}
}

//------------------------------------------------------------------------------
// UPDATE_BALL_POSITION: updates position of ball i using parameter dt as time
// passed
//------------------------------------------------------------------------------
void update_ball_position(int i, float dt){
	double		ax, ay;		// acceleration
	double		vel_modulus;
	double		vel_modulus_after;
	double		alpha;		//angle
	vel_modulus = sqrt(ball[i].vx * ball[i].vx + ball[i].vy * ball[i].vy);
	alpha = asin(ball[i].vy/vel_modulus); //direction of speed vector
	
	// banking
	ax = ball[i].ac * cos(alpha + PI/2); 
	ay = ball[i].ac * sin(alpha + PI/2);

	ball[i].vx += ax * dt;
	ball[i].vy += ay * dt;
	vel_modulus_after = sqrt(ball[i].vx * ball[i].vx + ball[i].vy
	 * ball[i].vy);
	ball[i].vx *= vel_modulus / vel_modulus_after; 
	ball[i].vy *= vel_modulus / vel_modulus_after;
	ax = ball[i].al * cos(alpha);   //longitudinal acceleration
	ay = ball[i].al * sin(alpha);
	ball[i].vx += ax * dt;
	ball[i].vy += ay * dt;
	ball[i].x_old = ball[i].x;
	ball[i].y_old = ball[i].y;
	ball[i].x += ball[i].vx * dt;
	ball[i].y += ball[i].vy * dt;
	float dist = distance(ball[i].x, ball[i].y, YWIN/2, YWIN/2);

	if (dist > RR) {
		ball[i].out_of_radar = TRUE;
		clear_ball(i);
	}
	else draw_ball(i);
	ptask_wait_for_period(i);
	
}

//------------------------------------------------------------------------------
// BALL_TASK: task that handles all the balls movement after getting the period
//------------------------------------------------------------------------------
void ball_task() {
	int		i; 		// task index
	float	dt; 	// integration interval
	i = ptask_get_index();
	dt = TSCALE * (float)ptask_get_period(i, MILLI) / 1000;

	init_ball(i, dt);

	while (!end && !ball[i].out_of_radar) 
		update_ball_position(i, dt);
	tasks--;
	printf("\t\tTask %d ended. Color %d coordinates %f %f\n", i, ball[i].c, 
	 ball[i].x, ball[i].y);
}

//------------------------------------------------------------------------------
// BURST_CHECK: if the last recorded measure for ball i is too old, set the
// burst as false
//------------------------------------------------------------------------------
void burst_check(clock_t scanning_time, int i) {
	long	elapsed_clocks;		// number of clocks since the last measure
	double	cpu_time_elapsed;	// elapsed time in seconds
	
	// if an object has been intercepted
	if (m.object[i] != 0) {
		elapsed_clocks = scanning_time - m.ct[i];
		// posix defines CLOCKS_PER_SEC as one million regardless of hardware
		cpu_time_elapsed = ((double)(elapsed_clocks)) / CLOCKS_PER_SEC;
		
		// after 0.01 sec have passed sets burst to false and records time
		// signals to the tracking threads that have a ball color that
		// the burst of measures is over
		if (elapsed_clocks > 10000 && m.burst[i] == TRUE) {
			pthread_mutex_lock(&mutex);
			m.burst[i] = FALSE;
			m.ct[i] = scanning_time;
			pthread_mutex_unlock(&mutex);
			pthread_cond_signal(&m.ptrt[i]);
		}
		
		// signalling tracking task that contact is lost by setting
		// m.object = 0 when 3 sec have passed (it takes less than 3 sec to
		// perform a full rotation)
		if (cpu_time_elapsed > 3.) {
			pthread_mutex_lock(&mutex);
			m.object[i] = 0;      
			m.burst[i] = FALSE;
			m.ct[i] = scanning_time;
			pthread_mutex_unlock(&mutex);
			pthread_cond_signal(&m.ptrt[i]);
		}
	}
}

//------------------------------------------------------------------------------
// ASSIGN_MEASURE: used to record in the measure resource the coordinates, the
// color and the scanning time of ball j when inside burst period
//------------------------------------------------------------------------------
void assign_measure(int lx, int ly, clock_t scanning_time, int target, int j){
	pthread_mutex_lock(&mutex);
	m.object[j] = target;
	m.ix[j] = lx;
	m.iy[j] = ly;
	m.ct[j] = scanning_time;
	m.burst[j] = TRUE;
	pthread_mutex_unlock(&mutex);
}

//------------------------------------------------------------------------------
// LINE_SCAN: used to scan objects in a straigh	t line. x0 and y0 are the 
// coordinates of the starting point of the line and a is the angle.
//------------------------------------------------------------------------------
void line_scan(int x0, int y0, int a) {
	int		lx,ly;	// line coordinates
	int		d;	
	int		target;	// color to get using getpixel
	int		i, j;	// i used to loop through the balls, while j is used to 
					// register the first known index
	float 	alpha;	// angle in radiants
	alpha = a * PI / 1800;
	clock_t scanning_time = clock();

// scanning array looking for old objects and looking for end of bursts 
	for (i = 0; i < MAX_BALLS; i++) {      
		burst_check(scanning_time, i);
	}

// scan through the entire radar length for intersection points with the balls
	for (d = RRMIN; d < RRMAX - 1; d += RSTEP) {
		lx = x0 + d * cos(alpha);
		ly = y0 + d * sin(alpha);
		target = getpixel(sky, lx, ly);
		if (target > 0) {
			j = -1;
			for (i = 0; i < MAX_BALLS; i++) {
				if (m.object[i] == 0 && j == -1) j = i; // first free index
				if (m.object[i] == target && (int)distance(lx, ly, m.ix[i],
				 m.iy[i]) < DISTANCE_TRHESHOLD){ // hit known object
					j = i;
					break;
				}
			}
			if (j != -1) {
			
			// waking up the thread when ball is intercepted
				assign_measure(lx, ly, scanning_time, target, j);
				pthread_cond_signal(&m.ptrt[j]);
			}
		}
	}
}

//------------------------------------------------------------------------------
// ADD_INDEX: calculates next index of the circular array used for balls
// positions.
//------------------------------------------------------------------------------
int add_index(int a, int k) {
	if (k < 0) k += MAX_TRACK_SAMPLES;
	int sidx = (a + k) % MAX_TRACK_SAMPLES;
	return sidx;
}

//------------------------------------------------------------------------------
// FIND_BALL_SAMPLES: given ball index i returns the number of ball samples to
// print
//------------------------------------------------------------------------------
int find_ball_sample(int j){
	int		samples_to_display;
	
	samples_to_display = t.newest_index[j] - t.oldest_index[j] + 1;
	
	if (samples_to_display <= 0) 
		samples_to_display = MAX_TRACK_SAMPLES;
	
	if (t.status[j] == OFF) 
		samples_to_display = 0;
		
	return samples_to_display;
}

//------------------------------------------------------------------------------
// ROTATE_RADAR: handles the radar rotation and scans along a direction. The
// rotation of the angle at each cycle is determined by the inc parameter
//------------------------------------------------------------------------------
void rotate_radar(int inc, int* a){
	
	float alpha;
	alpha = *a * PI / 1800;
	// clears old radar beam
	line(buffer, YWIN/2, YWIN/2, YWIN/2 + RR * cos(alpha), 
	YWIN/2 + RR * sin(alpha), BKG);

	*a = *a + inc;
	if (*a == 3600) *a = 0;

	line_scan(YWIN/2, YWIN/2, *a);

	alpha = *a * PI /1800;
	// draws radar beam
	line(buffer, YWIN/2, YWIN/2, YWIN/2 + RR * cos(alpha),
	 YWIN/2 + RR * sin(alpha), WHITE);
}

//------------------------------------------------------------------------------
// RADAR_TASK: handles the rotation of the randar and clears old tracks and
// ball positions
//------------------------------------------------------------------------------
void radartask(void) {
	int		i, k;	// task index i and sample index k
	int		a = 0;	// scanning direction (deg*10)
	int		j;		// array index
	int		sidx;	// sample index
	int		samples_to_display;
	int		radius;

	i = ptask_get_index();
	while (!end) {
		
		rotate_radar(5, &a);
		
		for (j = 0; j < MAX_BALLS; j++) {
			samples_to_display = find_ball_sample(j);
			radius = 1;
			for (k = 0; k < samples_to_display; k++) {

				sidx = add_index(t.oldest_index[j], k);

				if (k == 1) // clearing old arrow
					draw_arrow(t.ix[j][add_index(t.newest_index[j], -1)], 
					t.iy[j][add_index(t.newest_index[j], -1)],
					t.prvs_prdct_ix[j],  t.prvs_prdct_iy[j], BKG);

				// clearing not active tracks or samples
				if ((k == 0 && samples_to_display == MAX_TRACK_SAMPLES) || 
				t.status[j] == SHUTTING_DOWN) {
					circlefill(buffer,t.ix[j][sidx],t.iy[j][sidx],3,BKG);
				} else {
				// clearing before drawing needed for resizing
					circlefill(buffer, t.ix[j][sidx], t.iy[j][sidx], 3, BKG);
					circlefill(buffer, t.ix[j][sidx], t.iy[j][sidx], radius,
					m.object[j]); // drawing active tracks
				}
				
				// newest sample has radius 3 the oldest ones a smaller radius
				if (k >= samples_to_display - 3) radius++; 
			}
			if (samples_to_display >= 2) {
				draw_arrow(t.ix[j][t.newest_index[j]],
				 t.iy[j][t.newest_index[j]], t.prdct_ix[j], t.prdct_iy[j],
				  m.object[j]);
			} 
			if (t.status[j] == SHUTTING_DOWN) {
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

//------------------------------------------------------------------------------
// GET_KEYCODES: gets ascii code from pressed key
//------------------------------------------------------------------------------
void get_keycodes(char *scan, char *ascii) {
	int	k;
	k = readkey();  
	*ascii = k;     
	*scan = k >> 8; 
}

//------------------------------------------------------------------------------
// GET_STRING: reads a string from keyboard and displays the echo in graphic
// mode at position (x, y) color c and background b
//------------------------------------------------------------------------------
void get_string(char *str, int x, int y, int c, int b) {
	char ascii, scan, s[2];
	int i = 0;
	do {
		get_keycodes(&scan, &ascii);
		if (scan != KEY_ENTER) {
			s[0] = ascii;
			s[1] = '\0';
			textout_ex(buffer, font, s, x, y, c, b);
			x = x + 8;
			str[i++] = ascii; 
		}
	} while (scan != KEY_ENTER);
	str[i] = '\0';
}

//------------------------------------------------------------------------------
// COMPUTED_PREDICTED_DATA: compute predicted balls coordinates when samples
// is greater than k_samples
//------------------------------------------------------------------------------
void compute_predicted_data(int i) {

	int samples = t.newest_index[i] - t.oldest_index[i] + 1;
	
	if (samples <= 0) samples = MAX_TRACK_SAMPLES;
	t.prvs_prdct_ix[i] = t.prdct_ix[i];
	t.prvs_prdct_iy[i] = t.prdct_iy[i];
	
	if (samples < k_samples){
		t.prdct_ix[i] = 0;
		t.prdct_iy[i] = 0;
	}
	
	// prediction with only 2 samples
	if (samples >= k_samples && k_samples == 2) {
		t.prdct_ix[i] = 2 * t.ix[i][t.newest_index[i]] -
		 t.ix[i][add_index(t.newest_index[i], -1)];
		
		t.prdct_iy[i] = 2 * t.iy[i][t.newest_index[i]]
		 - t.iy[i][add_index(t.newest_index[i], -1)];
	}
	
	// predictiong using more than 3 samples
	if (samples >= k_samples && k_samples >= 3) {
		t.prdct_ix[i] = 3 * t.ix[i][t.newest_index[i]] -
		 3 * t.ix[i][add_index(t.newest_index[i], -1)] + 
		 t.ix[i][add_index(t.newest_index[i], -2)];
		
		t.prdct_iy[i] = 3 * t.iy[i][t.newest_index[i]] -
		 3 * t.iy[i][add_index(t.newest_index[i], -1)] +
		 t.iy[i][add_index(t.newest_index[i], -2)];
	}
}

//------------------------------------------------------------------------------
// UPDATE_COMPUTED_DATA: print predicted data on screen after calling
// compute_predicted_data
//------------------------------------------------------------------------------
void update_computed_data(int i, float fix, float fiy, int status, int object) {
	float	pix, piy;	// will contain predicted data
	char	s[LEN];		// buffer for text to be printed on screen
	
	if (status == NOT_TRACKING) {
		t.status[i] = SHUTTING_DOWN;
	} else {
		if (t.status[i] == ON) {
			t.newest_index[i] = (t.newest_index[i] + 1) % MAX_TRACK_SAMPLES;
			
			// if oldest = newest it means that the array is full so the oldest
			// index needs to be increased
			if (t.newest_index[i] == t.oldest_index[i]) 
				t.oldest_index[i] = (t.oldest_index[i] + 1) % MAX_TRACK_SAMPLES;
			t.ix[i][t.newest_index[i]] = fix;
			t.iy[i][t.newest_index[i]] = fiy;
		} else {
			// if array was empty it must be initialised
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
	sprintf(s, "%2d) %6.2f %6.2f  %6.2f %6.2f", i, fix, fiy, pix, piy);
	textout_ex(screen, font, s, 600, 30+10*i, object, BKG);
}

//------------------------------------------------------------------------------
// PATH_RANDOMIZER: makes the balls assume a random direction and speed by
// varying the balls'acceleration.
//------------------------------------------------------------------------------
void path_randomizer() {
	int		i, k, r;		// i:ptask index k: used as ball index r: rand
	float	vel_modulus;	// modulus of ball speed
	i = ptask_get_index();
	
	while(!end) {
		for (k = 0; k < MAXT; k++) {
			if(!ball[k].out_of_radar) {
				vel_modulus = sqrt(ball[k].vx * ball[k].vx + ball[k].vy
				 * ball[k].vy);
				r = rand()%5;
				switch(r) {
					case 0:	// in 2 out of 5 cases the ball does not change
					case 1: // its acceleration
						ball[k].al = 0;
						ball[k].ac = 0;
						break;		
					// it changes acceleration only when the ball was in 
					// uniform straight motion before
					case 2:		// banking
						if (ball[k].al == 0 && ball[k].ac == 0 ) {
							ball[k].ac = frand(AMIN, AMAX);
							ball[k].al = 0;
						}
						break;
					case 3:		// acceleration
						if (ball[k].al == 0 && ball[k].ac == 0 ) {
							if (vel_modulus < VMAX) 
								ball[k].al = frand(0, AMAX);
							ball[k].ac = 0;
						}
						break;
					case 4: 	// braking
						if (ball[k].al == 0 && ball[k].ac == 0 ) {
							if (vel_modulus > VMAX/4) 
								ball[k].al = frand(AMIN, 0);
							ball[k].ac = 0;
						}
						break;
					default:
						break;
				}
				printf("acceleration k: %d  al: %f ac: %f vel: %f\n", k,
				 ball[k].al, ball[k].ac, vel_modulus);
			}
		}
		ptask_wait_for_period(i);
	}
}

//------------------------------------------------------------------------------
//	WAIT_FOR_DATA: wakes up when data from the scanning beam is taken
//------------------------------------------------------------------------------
void wait_for_data(int i, int* ix, int* iy, int* object, int* burst){

	pthread_mutex_lock(&mutex);
	pthread_cond_wait(&m.ptrt[i], &mutex);
	*object = m.object[i];
	*ix = m.ix[i]; 	// copy of shared data 
	*iy = m.iy[i];
	*burst = m.burst[i];
	pthread_mutex_unlock(&mutex);
}

//------------------------------------------------------------------------------
// COMPUTE_MEAN_POSITIONS: used to calcolate mean positions from the samples
// contained in ssx and ssy, which are the samples found during the burst 
// period
//------------------------------------------------------------------------------
void compute_mean_positions(int *max_samples, int* ssx, int* ssy, float* fix,
 float* fiy){
	int	k;		// samples index
	*fix = 0.;	// sample x coordinate
	*fiy = 0.;	// sample y coordinate
	for (k = 0; k < *max_samples; k++) {
		*fix += ssx[k];
		*fiy += ssy[k];
	}
	*fix = *fix / *max_samples;
	*fiy = *fiy / *max_samples;

}

//------------------------------------------------------------------------------
// TRACKING_THREAD: handles the tracking of the position of a ball and updates
// it after it has calculated the mean position using all samples 
//------------------------------------------------------------------------------
void* tracking_thread(void* arg) {
	int		i = *(int*)arg;
	int		object;		// colour of tracked object
	int		ix, iy;
	float	fix, fiy;
	int		burst;		// used to signal if we are inside a burst
	
	int		status = NOT_TRACKING;
	int		nr_sub_samples = 0;
	int		sub_samples_x[10], sub_samples_y[10];

	while(!end) {
		wait_for_data(i, &ix, &iy, &object, &burst);

		if (object != 0) {
			if (burst && nr_sub_samples < 10) {
				status = TRACKING;
				sub_samples_x[nr_sub_samples] = ix;
				sub_samples_y[nr_sub_samples] = iy;
				nr_sub_samples++;
			}
			if (!burst) {
				status = TRACKING_END_OF_BURST;
			}
		}
		else {
			status = NOT_TRACKING;
			nr_sub_samples = 0;
			update_computed_data(i, fix, fiy, status, object);
		}

		if (status == TRACKING_END_OF_BURST) {
			
			compute_mean_positions(&nr_sub_samples, sub_samples_x,
			 sub_samples_y, &fix, &fiy);
			nr_sub_samples = 0;
			
			// updating shared area for display
			update_computed_data(i, fix,fiy, status, object);
		}
	}
	return 0;
}

//------------------------------------------------------------------------------
// HANDLES_I: called when i is pressed, which is used to decide the number of
// samples the program has to take before predicting the next position 
//------------------------------------------------------------------------------
void handles_i(void){
	char str[5], ss[24];	// str: used for scanf buffer, ss: used for output
	textout_ex(buffer, font, "Enter k:          ", 10, 50, 2, BKG);
	textout_ex(buffer, font, "                ", 10, 60, 2, BKG);
	get_string(str, 80, 50, 3, 0);
	sscanf(str, "%d", &k_samples);
	if (k_samples < 2) textout_ex(buffer, font, "too little      ", 10, 60, 2, 
	 BKG);
	if (k_samples > 5) textout_ex(buffer, font, "too big         ", 10, 60, 2, 
	 BKG);
	if (k_samples >= 2 && k_samples <= 5) {
		sprintf(ss, "Accepted : %d", k_samples);
		textout_ex(buffer, font, ss, 10, 60, 2, BKG);
	}
}

//------------------------------------------------------------------------------
// HANDLES_SPACES: called when the space key is pressed, spawns the balls on the
// screen
//------------------------------------------------------------------------------
void handles_spaces(void){
	int i;
	if (tasks < MAXT) {
		i = ptask_create_prio(ball_task, PER, 50, NOW);
		tasks++;
		printf("\tTask %d created. Total balls num: %d\n", i, tasks);
	} else {
		printf("\tCANNOT CREATE MORE TASK: max number %d reached\n", tasks);
	}
}

int main(void) {
	int				j;		// loop indexes
	char			scan;	//ascii character for scan ball
	pthread_attr_t	a;
	pthread_t		p;

	init();
	pthread_attr_init(&a);
	pthread_attr_setdetachstate(&a, PTHREAD_CREATE_DETACHED);
	int balls[MAX_BALLS];
	
	// create tracking threads up to the maximum number of balls
	for (j = 0; j < MAX_BALLS; j++) {
		balls[j] = j;
		pthread_create(&p, &a, tracking_thread, (void*)&balls[j]);
	}
	// the randomizer and radartask are called at specific intervals
	ptask_create_prio(radartask, PERRADAR, 70, NOW);
	ptask_create_prio(path_randomizer, PERRANDOMIZER, 10, NOW);
	textout_ex(buffer, font, "Press I for entering k ", 10, 30, 3, 0);
	textout_ex(buffer, font, "Press SPACE to create a moving object", 10, 10,
	 11, BKG);
	tasks = 0;

	do {
		scan = 0;
		if (keypressed()) scan = readkey() >> 8;
		if (scan == KEY_SPACE) handles_spaces();
		if (scan == KEY_I) handles_i();

		sleep(0.2);
	} while (scan != KEY_ESC);
	end = 1;

	sleep(1);
	allegro_exit();
	return 0;
}

 
