/****
	  tri.h
   	  Clone this package from git://github.com/rtv/trianger.git
	  version 1
	  Richard Vaughan  
****/

#include <vector>
#include <set>
#include <list>
//#include <math.h> 
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#define GRAPHICS 1
#define DEBUGVIS 0

double const Pi = 4 * std::atan(1);

// handy STL iterator macro pair. Use FOR_EACH(I,C){ } to get an iterator I to
// each item in a collection C.
#define VAR(V,init) __typeof(init) V=(init)
#define FOR_EACH(I,C) for(VAR(I,(C).begin());I!=(C).end();I++)

namespace Tri
{
  /** Convert radians to degrees. */
  inline double rtod( double r ){ return( r * 180.0 / Pi ); }
  /** Convert degrees to radians */
  inline double dtor( double d){ return( d * Pi / 180.0 ); }
  	
	 /** initialization: call this before using any other calls. */	
  void Init( int argc, char** argv );

	
  static void UpdateGui();

  /** update all robots */
  static void UpdateAll();
  
  /** Normalize a length to within 0 to worldsize. */
  static double DistanceNormalize( double d );
  
  /** Normalize an angle to within +/_ M_PI. */
  static double AngleNormalize( double a );
  
  /** Wrap distances around the torus */
  static double WrapDistance( double d );
  
  /** Start running the simulation. Does not return. */
  static void Run();
  
  
#if GRAPHICS
  static int winsize; // initial size of the window in pixels
  
  /** initialization: call this before using any other calls. */	
  static void InitGraphics( int argc, char* argv[] );
  
  /** render all robots in OpenGL */
  static void DrawAll();
  
  // render the robot in OpenGL
  void Draw();	 
#endif
  
  class Robot
  {
  public:
    
    
    class Pose
    {
    public:
      double x,y,a; // 2d position and orientation
      
    Pose( double x, double y, double a ) : x(x), y(y), a(a) {}
    Pose() : x(0.0), y(0.0), a(0.0) {}
                  
      //Pose( const Pose &p ) : x(p.x), y(p.y), a(p.a) {}	
      
      // get a random pose 
      static Pose Random()
      {
	return Pose( drand48() * Robot::worldsize, 
		     drand48() * Robot::worldsize, 
		     Robot::AngleNormalize( drand48() * (M_PI*2.0)));
      }
    } pose; // instance: robot is located at this pose
    
    class Speed
    {		
    public:
	   double v; // forward speed
	   double w; // turn speed
	   
	 Speed() : v(0.0), w(0.0) {}		
	 } speed; // instance: robot is moving this fast
	 
	 	 
	 /** A sense vector containing information about all the robots
	     detected in my field of view */
	 std::vector<Robot*> see_robots;
	 
	 // constructor
	 Robot( const Pose& pose );
	 
	 // destructor
	 virtual ~Robot() {}
	 
	 /** pure virtual - subclasses must implement this method  */
	 virtual void Controller() = 0;

	private:
	 
	 // move the robot
	 void UpdatePose();
	 
	 // update the field of view
	 void UpdateSensor();

  public:
	 // update everything
	 void Update();
  };	

}; // namespace Tri
