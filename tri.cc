/****
     antix.cc
     version 1
     Richard Vaughan  
     Clone this package from git://github.com/rtv/Antix.git
****/

#include <assert.h>
#include <unistd.h>
#include <algorithm>
#include <sys/time.h> // for gettimeofday(3)
#include "tri.h"
using namespace Tri;

static double start_seconds(0);
static bool paused( false); // runs only when this is false
static bool show_data( false ); // controls visualization of pixel data
static double fov( dtor(90)) ; // sensor detects objects within this angular field-of-view about the current heading
static double radius( 0.01 ); // radius of all robot's bodies
static double range(0.1);    // sensor detects objects up tp this maximum distance
static double worldsize(1.0); // side length of the toroidal world
static std::vector<Robot> population;
static uint64_t updates(0); // number of simulation steps so far	 
static uint64_t updates_max(0); // number of simulation steps to run before quitting (0 means infinity)
static unsigned int sleep_msec(10); // number of milliseconds to sleep at each update  
static unsigned int gui_interval(100); // number of milliseconds between window redraws


template <class T, class C>
void EraseAll( T thing, C& cont )
{ cont.erase( std::remove( cont.begin(), cont.end(), thing ), cont.end() ); }

const char usage[] = "Antix understands these command line arguments:\n"

  "  -? : Prints this helpful message.\n"
  "  -a <int> : sets the number of pucks in the world.\n"
  "  -c <int> : sets the number of pixels in the robots' sensor.\n"
  "  -d  Enables drawing the sensor field of view. Speeds things up a bit.\n"
  "  -f <float> : sets the sensor field of view angle in degrees.\n"
  "  -g <int> : sets the interval between GUI redraws in milliseconds.\n"
  "  -p <int> : set the size of the robot population.\n"
  "  -r <float> : sets the sensor field of view range.\n"
  "  -s <float> : sets the side length of the (square) world.\n"
  "  -u <int> : sets the number of updates to run before quitting.\n"
  "  -w <int> : sets the initial size of the window, in pixels.\n"
  "  -z <int> : sets the number of milliseconds to sleep between updates.\n";


void Init( int argc, char** argv )
{
  // seed the random number generator with the current time
  //srand48(time(NULL));
  srand48(0); // for debugging - start the same every time
	
  // parse arguments to configure Robot static members
  int c;
  while( ( c = getopt( argc, argv, "?dh:a:p:s:f:g:r:c:u:z:w:")) != -1 )
    switch( c )
      {
      case 'h':
	home_count = atoi( optarg );
	printf( "[Antix] home count: %d\n", home_count );
	break;

      case 'a':
	puck_count = atoi( optarg );
	printf( "[Antix] puck count: %d\n", puck_count );
	break;
			  
      case 'p': 
	home_population = atoi( optarg );
	printf( "[Antix] home_population: %d\n", home_population );
	break;
				
      case 's': 
	worldsize = atof( optarg );
	printf( "[Antix] worldsize: %.2f\n", worldsize );
	break;
				
      case 'f': 
	fov = dtor(atof( optarg )); // degrees to radians
	printf( "[Antix] fov: %.2f\n", fov );
	break;
				
      case 'g':
	gui_interval = atol( optarg );
	printf( "[Antix] gui_interval: %lu\n", (long unsigned)gui_interval );
	break;

      case 'r': 
	range = atof( optarg );
	printf( "[Antix] range: %.2f\n", range );
	break;
								
      case 'u':
	updates_max = atol( optarg );
	printf( "[Antix] updates_max: %lu\n", (long unsigned)updates_max );
	break;
				
      case 'z':
	sleep_msec = atoi( optarg );
	printf( "[Antix] sleep_msec: %d\n", sleep_msec );
	break;
				
#if GRAPHICS
      case 'w': winsize = atoi( optarg );
	printf( "[Antix] winsize: %d\n", winsize );
	break;

      case 'd': show_data=true;
	puts( "[Antix] show data" );
	break;
#endif			
      case '?':
	puts( usage );
	exit(0); // ok
	break;

      default:
	fprintf( stderr, "[Antix] Option parse error.\n" );
	puts( usage );
	exit(-1); // error
      }
  	
#if GRAPHICS
  InitGraphics( argc, argv );
#endif // GRAPHICS
    
  FOR_EACH( it, population )
    {
      it->pose = Pose::Random();

      // todo: install the controllers
    }

  // record the starting time to measure how long we have run for
  struct timeval tv;
  gettimeofday( &tv, NULL );
  start_seconds = tv.tv_sec + tv.tv_usec/1e6;
}


// wrap around torus
double Robot::WrapDistance( double d )
{
  const double halfworld( worldsize * 0.5 );
  
  if( d > halfworld )
    d -= worldsize;
  else if( d < -halfworld )
    d += worldsize;
  
  return d;
}

/** Normalize a length to within 0 to worldsize. */
double Robot::DistanceNormalize( double d )
{
  while( d < 0 ) d += worldsize;
  while( d > worldsize ) d -= worldsize;
  return d; 
} 

/** Normalize an angle to within +/_ M_PI. */
double Robot::AngleNormalize( double a )
{
  while( a < -M_PI ) a += 2.0*M_PI;
  while( a >  M_PI ) a -= 2.0*M_PI;	 
  return a;
}	 


void Robot::UpdateSensor()
{
  see_robots.clear();
  
  // test squared ranges to avoid expensive sqrt()
  double rngsqrd( range * range );
  
  FOR_EACH( it, robots )
    {
      // discard if it's the same robot
      if( other == this )
	continue;
      
      // discard if it's out of range. We put off computing the
      // hypotenuse as long as we can, as it's relatively expensive.
      
      const double dx( WrapDistance( it->pose.x - pose.x ) );
      if( fabs(dx) > Robot::range )
	continue; // out of range
      
      const double dy( WrapDistance( it->pose.y - pose.y ) );		
      if( fabs(dy) > Robot::range )
	continue; // out of range
      
      // test distance squared
      const double dsq( dx*dx + dy*dy );
      if( dsq > rngsqrd ) 
	continue; 
      
      // discard if it's out of field of view 
      const double absolute_heading( fast_atan2( dy, dx ) );
      const double relative_heading( AngleNormalize((absolute_heading - pose.a) ));
      if( fabs(relative_heading) > fov/2.0   ) 
	continue; 
      
      // store a pointer to the visible robot
      see_robots.push_back( &(*it) );
    }
}	

void Robot::UpdatePose()
{
  // move according to the current speed 
  const double dx( speed.v * cos(pose.a) );
  const double dy( speed.v * sin(pose.a) ); 
  const double da( speed.w );
  
  pose.x = DistanceNormalize( pose.x + dx );
  pose.y = DistanceNormalize( pose.y + dy );
  pose.a = AngleNormalize( pose.a + da );    
}


void Robot::UpdateAll()
{
  FOR_EACH( r, population )
    {
      r->UpdateSensor();
      r->Controller();
      r->UpdatePose();
    }
  
  ++updates;  
}

void Robot::Run()
{
  // if we've done enough updates, exit the program
  if( updates_max > 0 && updates > updates_max )
    exit(1);


#if GRAPHICS
  UpdateGui();
#else
  while( 1 )
    UpdateAll();
#endif

  static double lastseconds=0;
  
  if( updates % 10 == 0 ) // every now and again
    {
      static struct timeval tv;
      gettimeofday( &tv, NULL );
      
      double seconds = tv.tv_sec + tv.tv_usec/1e6;
      double interval = seconds - lastseconds;
      printf( "[%llu] %.2f (%.2f)\n", updates, 10.0/interval, updates/(seconds-start_seconds) );      
      lastseconds = seconds;
    }
    
  // possibly snooze to save CPU and slow things down 
  if( sleep_msec > 0 )
    usleep( sleep_msec * 1e3 );
}


