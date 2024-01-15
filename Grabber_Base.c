#include<stdio.h>
#include<stdlib.h>
#include<math.h>

/************************************
 * 
 * 	Helpers
 * 
 * **********************************/
float root(float n){
  // Max and min are used to take into account numbers less than 1
  float lo = (n > 1) ? n : 1, hi = (n < 1) ? n : 1, mid;

  // Update the bounds to be off the target by a factor of 10
  while(100 * lo * lo < n) lo *= 10;
  while(0.01 * hi * hi > n) hi *= 0.1;

  int i;
  for(i = 0 ; i < 100 ; i++){
      mid = (lo+hi)/2;
      if(mid*mid == n) return mid;
      if(mid*mid > n) hi = mid;
      else lo = mid;
  }
  return mid;
}



/************************************
 * 
 * 	Functions
 * 
 * **********************************/

//Chaser

	//Image highest density finder -> optimize with aproximation
	struct pixel
	{
		unsigned int x, y;
	};
	void density(unsigned int *in, struct pixel *location)
	{
		int i, j;

		
		unsigned int density = 0;

		struct
		{
			unsigned int img[512][512];
		} *s_in;
		s_in = (void *)in;
		#define c_in (s_in->img)

		for(i=1; i < 511; i++)
		{
			for(j=1; j < 511; j++)
			{
				unsigned int current = c_in[i-1][j-1] + c_in[i][j-1] + c_in[i+1][j-1] + c_in[i-1][j+1] + c_in[i][j+1] + c_in[i+1][j+1];	
				current += c_in[i-1][j] + c_in[i][j] + c_in[i+1][j];

				if(current > density)
				{
					density = current;
					location->x = i;
					location->y = j;
				}
			}
		}
		
	}

	void pixel_to_depth(struct pixel p_1, struct pixel p_2, int *result)
	{
		//arbitrary distance between cameras, in meters
		#define distance  0.1
		//arbitrary focal point, in meters
		#define focal 	1
		*result = (focal * distance) / (p_1.x - p_2.x); 
	}

	//Image xy(pixel) to robot reference frame direction
	void pixel_to_direction(struct pixel p, float *result)
	{
		float origin_relative_x = p.x - 255.5;
		float origin_relative_y = -(p.y - 512);

		origin_relative_x = origin_relative_x * origin_relative_x;
		origin_relative_y = origin_relative_y * origin_relative_y;

		*result = root(origin_relative_x + origin_relative_y);
	}

	//PID controller for movement
		//Integrator
		//Derivator
	void PID(float current, float reference, float *result)
	{
		float P = 1.1, I=0.3, D=0.1;

		float error = reference - current;

		static float integral = 0.0;
		integral += error;

		static float prior = 0.0;
		

		float derivative = error - prior;
		prior = error;

		*result = P*error + I*integral + D*derivative;
	}


	//Motor control (H bridge) 2 motors
	struct motor
	{
		unsigned char right, left;
	};
	void arm_ctrl(int depth, unsigned int *sensor, struct motor *a)
	{
		unsigned int current = *sensor;
		a->right = (current - depth) * 10;
		a->left = (current - depth) * 10;
		return;
	}



//Grabber adjuster : stereo input for depth
	//Sobel filter (+ some image processing) -> optimize with perforation (row column skipping)
		//Image kernel applier
		//Filter kernel

	//Sobel filter (+ some image processing) -> optimize with perforation (row column skipping)
		//Image kernel applier
		//Filter kernel

	//Image highest density finder -> optimize with aproximation

	//Image xy(pixel) to robot reference frame direction

	//Arm controller (open loop)

	

/************************************
 * 
 * 	Interpreter
 * 
 * **********************************/

#define PROGRAM_SIZE (sizeof(P)/sizeof(enum instruction))

enum instruction{PROCESS_SOBEL_IMG, FIND_DENSITY, FIND_DENSITY2, FIND_DEPTH, DO_PID, UPDATE_MOTORS};

/*
	inputs and outputs as arg pointers
*/
void interpreter(unsigned int *indata, unsigned char *r, unsigned char *l, unsigned int *sensor)
{
	//P
	unsigned int pc = 0;
	enum instruction P[] = {FIND_DENSITY, FIND_DENSITY2, FIND_DEPTH, UPDATE_MOTORS};

	//E
	struct pixel p;
	struct pixel p2;
	float ref;
	//float direction = 0.0;
	struct motor m;



	int depth;

	//runtime
	while(1)
	{
		switch(P[pc])
		{
			case FIND_DENSITY:
				density(indata, &p);
				break;
			case FIND_DENSITY2:
				density(indata, &p2);
				break;
			case FIND_DEPTH:
				pixel_to_depth(p, p2,&depth);
				break;
			
			case UPDATE_MOTORS:
				arm_ctrl(depth,sensor,&m);
				*r = m.right;
				*l = m.left;
				break;
			default: break;
		}
		pc = (pc + 1)%PROGRAM_SIZE;
	}
}



/*int main()
{

	
	return 0;
}*/