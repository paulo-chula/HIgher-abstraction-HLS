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
 // Perforating row/column
 // and reducing number used components
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

		for(i=1; i < 511; i+=2)
		{
			for(j=1; j < 511; j+=2)
			{
				unsigned int current = c_in[i-1][j-1]  + c_in[i+1][j-1] + c_in[i-1][j+1] + c_in[i+1][j+1];	
				current +=  c_in[i][j] ;

				if(current > density)
				{
					density = current;
					location->x = i;
					location->y = j;
				}
			}
		}
		
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
	void motor_ctrl(float actuation, struct motor *m)
	{

		if(actuation > 0.0)
		{
			m->right = 128 + (unsigned char)((actuation * 128) / 90);
			m->left  = 128;
		}
		else
		{
			m->left = 128 + (unsigned char)((actuation * 128) / 90);
			m->right  = 128;
		}

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

enum instruction{PROCESS_SOBEL_IMG, FIND_DENSITY, FIND_DIRECTION, DO_PID, UPDATE_MOTORS};

/*
	inputs and outputs as arg pointers
*/
void interpreter(unsigned int *indata, unsigned char *r, unsigned char *l)
{
	//P
	unsigned int pc = 0;
	enum instruction P[] = {FIND_DENSITY, FIND_DIRECTION, DO_PID, UPDATE_MOTORS};

	//E
	struct pixel p;
	float ref;
	float direction = 0.0;
	struct motor m;



	//runtime
	while(1)
	{
		switch(P[pc])
		{
			case FIND_DENSITY:
				density(indata, &p);
				break;
			case FIND_DIRECTION:
				pixel_to_direction(p,&ref);
				break;
			case DO_PID:
				PID(direction, ref, &direction);
				break;
			case UPDATE_MOTORS:
				motor_ctrl(direction,&m);
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