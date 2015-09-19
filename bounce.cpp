#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

#include <cmath>
#include"Vector.h"
#include "Camera.h"


int WIDTH = 500;
int HEIGHT = 500;

Camera *camera;

#define PRECISION  0.01
#define BOUNCECOO  0.96

#define FRONTPLAN  0
#define BACKPLAN   1
#define LEFTPLAN   2
#define RIGHTPLAN  3
#define BOTTONPLAN 4
#define TOPPLAN    5

#define cameraCoef 0.04 //camera scale coefficent



static char *ParamFilename = (char*)"/Users/yuyaolong/Documents/自用代码库/OpenGL_EX/assiginment1/assiginment1/parameters";

double boxEdge = 300 * cameraCoef;
double ballRadius = 10 * cameraCoef;
double boxCoordinate = boxEdge/2.0-ballRadius;


int tCount = 0;
int cubeLine = 0;
int pauseSign = 0;
int stopSign = 0;

double P0x = 0 * cameraCoef;
double P0y = 130 * cameraCoef;
double P0z = 0 * cameraCoef;
double accelX = 0 * cameraCoef;
double accelY = -10 * cameraCoef;
double accelZ = 0 * cameraCoef;
double xInitVelo = 0 * cameraCoef;
double yInitVelo = 0 * cameraCoef;
double zInitVelo = 40 * cameraCoef;
double hStep = 0.1;
double airRES = 0.99; //air resistance coefficent
int automaticSpeed = 50;

Vector3d ballPosition(P0x, P0y, P0z), ballVelocity(xInitVelo, yInitVelo, zInitVelo), ballAcceleration(accelX, accelY, accelZ), ballAccelerationOrigin(accelX, accelY, accelZ);

const Vector3d boxBottom(0, -boxCoordinate, 0), boxTop(0, boxCoordinate, 0), boxFront(0, 0, boxCoordinate), boxBack(0, 0, -boxCoordinate), boxLeft(-boxCoordinate, 0, 0), boxRight(boxCoordinate, 0, 0);

Vector3d ballVelocityHit, ballPositionHit;


const Vector3d boxFacesNormal(int plane)
{
	switch(plane)
	{
		case FRONTPLAN:
			return Vector3d(0, 0, -1);
			break;
		case BACKPLAN:
			return Vector3d(0, 0, 1);
			break;
		case LEFTPLAN:
			return Vector3d(1, 0, 0);
			break;
		case RIGHTPLAN:
			return Vector3d(-1, 0, 0);
			break;
		case BOTTONPLAN:
			return Vector3d(0, 1, 0);
			break;
		case TOPPLAN:
			return Vector3d(0, -1, 0);
			break;
		default:
			return Vector3d(0, 0, 0);
			break;	
			
	}
}


/*
 Load parameter file and reinitialize global parameters
 */
void LoadParameters(char *filename){
    
    FILE *paramfile;
    
    if((paramfile = fopen(filename, "r")) == NULL){
        fprintf(stderr, "error opening parameter file %s\n", filename);
        exit(1);
    }
    
    //ParamFilename = filename;
    cout<<ParamFilename<<endl;
    
    if(fscanf(paramfile, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d",
              &P0x, &P0y, &P0z, &xInitVelo, &yInitVelo, &zInitVelo, &accelX, &accelY, &accelZ, &airRES, &hStep, &automaticSpeed) != 12){
        fprintf(stderr, "error reading parameter file %s\n", filename);
        fclose(paramfile);
        exit(1);
    }
    else
    {
        ballPosition.set(P0x*cameraCoef, P0y*cameraCoef, P0z*cameraCoef);
        ballVelocity.set(xInitVelo*cameraCoef, yInitVelo*cameraCoef, zInitVelo*cameraCoef);
        ballAcceleration.set(accelX*cameraCoef, accelY*cameraCoef, accelZ*cameraCoef);
    }
}


void hitCalculation(const Vector3d ballPosition, Vector3d& ballPositionNew, const Vector3d ballVelocity, Vector3d& ballVelocityNew, int hitPlane)
{

    if (hitPlane == BOTTONPLAN) {
        double s = 0;
        s = (ballPosition.y- boxBottom.y)/((ballPosition.y-boxBottom.y)+(boxBottom.y-ballPositionNew.y));
        ballVelocityHit = ballVelocity + ballAcceleration*(s*hStep);
        ballPositionHit = ballPosition + ballAcceleration*(s*hStep);
        cout<<"S= "<<s<<endl;
        cout<<"Collision Position: "<<ballPositionHit*(1.0/cameraCoef)<<endl;
        cout<<"Collision VelocityHit: "<<ballVelocityHit*(1.0/cameraCoef)<<endl;
        ballVelocityHit.y = -ballVelocityHit.y;
        ballVelocityNew = (ballVelocityHit + ballAcceleration*(1-s)*hStep)*BOUNCECOO;
        ballPositionNew = ballPositionHit + ballVelocityHit*(1-s)*hStep;
        
        //detect when to stop
        if ( (ballAcceleration*boxFacesNormal(hitPlane))<0  ) 
        {
			if(abs(ballPositionNew.y-boxBottom.y)*(1.0/cameraCoef)< 1)
			{
                if (ballVelocityHit.normsqr()*(1.0/cameraCoef)< PRECISION)
                {
                    pauseSign = 1;
                    stopSign = 1;
                    cout<<"stop Position: "<<ballPositionHit*(1.0/cameraCoef)<<endl;
                    cout<<"stop VelocityHit: "<<ballVelocityHit*(1.0/cameraCoef)<<endl;
                
                }
                ballAcceleration.y = ballAcceleration.y * 0.9;
				
			}
			
        
		}
    }
    
    
    if (hitPlane == TOPPLAN) {
        double s = 0;
        s = (ballPosition.y- boxTop.y)/((ballPosition.y-boxTop.y)+(boxTop.y-ballPositionNew.y));
        ballVelocityHit = ballVelocity + ballAcceleration*(s*hStep);
        ballPositionHit = ballPosition + ballAcceleration*(s*hStep);
        cout<<"S= "<<s<<endl;
        cout<<"Collision Position: "<<ballPositionHit*(1.0/cameraCoef)<<endl;
        cout<<"Collision VelocityHit: "<<ballVelocityHit*(1.0/cameraCoef)<<endl;
        ballVelocityHit.y = -ballVelocityHit.y;
        ballVelocityNew = (ballVelocityHit + ballAcceleration*(1-s)*hStep)*BOUNCECOO;
        ballPositionNew = ballPositionHit + ballVelocityHit*(1-s)*hStep;
        
        //detect when to stop
        if ( (ballAcceleration*boxFacesNormal(hitPlane))<0  ) 
        {
			if(abs(ballPositionNew.y-boxTop.y)*(1.0/cameraCoef)< 1)
            {
                if (ballVelocityHit.normsqr()*(1.0/cameraCoef)< PRECISION)
                {
                    pauseSign = 1;
                    stopSign = 1;
                    cout<<"stop Position: "<<ballPositionHit*(1.0/cameraCoef)<<endl;
                    cout<<"stop VelocityHit: "<<ballVelocityHit*(1.0/cameraCoef)<<endl;
                
                }
                ballAcceleration.y = ballAcceleration.y * 0.9;
                
            }
		}
    }
    
    if (hitPlane == RIGHTPLAN) {
        double s = 0;
        s = (ballPosition.x- boxRight.x)/((ballPosition.x-boxRight.x)+(boxRight.x-ballPositionNew.x));
        ballVelocityHit = ballVelocity + ballAcceleration*(s*hStep);
        ballPositionHit = ballPosition + ballAcceleration*(s*hStep);
        cout<<"S= "<<s<<endl;
        cout<<"Collision Position: "<<ballPositionHit*(1.0/cameraCoef)<<endl;
        cout<<"Collision VelocityHit: "<<ballVelocityHit*(1.0/cameraCoef)<<endl;
        ballVelocityHit.x = -ballVelocityHit.x;
        ballVelocityNew = (ballVelocityHit + ballAcceleration*(1-s)*hStep)*BOUNCECOO;
        ballPositionNew = ballPositionHit + ballVelocityHit*(1-s)*hStep;
        
        //detect when to stop
        if ( (ballAcceleration*boxFacesNormal(hitPlane))<0  ) 
        {
			if(abs(ballPositionNew.x-boxRight.x)*(1.0/cameraCoef)< 1)
            {
                if (ballVelocityHit.normsqr()*(1.0/cameraCoef)< PRECISION)
                {
                    pauseSign = 1;
                    stopSign = 1;
                    cout<<"stop Position: "<<ballPositionHit*(1.0/cameraCoef)<<endl;
                    cout<<"stop VelocityHit: "<<ballVelocityHit*(1.0/cameraCoef)<<endl;
                
                }
                ballAcceleration.x = ballAcceleration.x * 0.9;
                
            }
        
		}
    }
    
    if (hitPlane == LEFTPLAN) {
        double s = 0;
        s = (ballPosition.x- boxLeft.x)/((ballPosition.x-boxLeft.x)+(boxLeft.x-ballPositionNew.x));
        ballVelocityHit = ballVelocity + ballAcceleration*(s*hStep);
        ballPositionHit = ballPosition + ballAcceleration*(s*hStep);
        cout<<"S= "<<s<<endl;
        cout<<"Collision Position: "<<ballPositionHit*(1.0/cameraCoef)<<endl;
        cout<<"Collision VelocityHit: "<<ballVelocityHit*(1.0/cameraCoef)<<endl;
        ballVelocityHit.x = -ballVelocityHit.x;
        ballVelocityNew = (ballVelocityHit + ballAcceleration*(1-s)*hStep)*BOUNCECOO;
        ballPositionNew = ballPositionHit + ballVelocityHit*(1-s)*hStep;
        
        //detect when to stop
        if ( (ballAcceleration*boxFacesNormal(hitPlane))<0  ) 
        {
			if(abs(ballPositionNew.x-boxLeft.x)*(1.0/cameraCoef)< 1)
            {
                if (ballVelocityHit.normsqr()*(1.0/cameraCoef)< PRECISION)
                {
                    pauseSign = 1;
                    stopSign = 1;
                    cout<<"stop Position: "<<ballPositionHit*(1.0/cameraCoef)<<endl;
                    cout<<"stop VelocityHit: "<<ballVelocityHit*(1.0/cameraCoef)<<endl;
                
                }
                ballAcceleration.x = ballAcceleration.x * 0.9;
                
            }
        
		}
    }
    
    if (hitPlane == FRONTPLAN) {
        double s = 0;
        s = (ballPosition.z- boxFront.z)/((ballPosition.z-boxFront.z)+(boxFront.z-ballPositionNew.z));
        ballVelocityHit = ballVelocity + ballAcceleration*(s*hStep);
        ballPositionHit = ballPosition + ballAcceleration*(s*hStep);
        cout<<"S= "<<s<<endl;
        cout<<"Collision Position: "<<ballPositionHit*(1.0/cameraCoef)<<endl;
        cout<<"Collision VelocityHit: "<<ballVelocityHit*(1.0/cameraCoef)<<endl;
        ballVelocityHit.z = -ballVelocityHit.z;
        ballVelocityNew = (ballVelocityHit + ballAcceleration*(1-s)*hStep)*BOUNCECOO;
        ballPositionNew = ballPositionHit + ballVelocityHit*(1-s)*hStep;
        
        //detect when to stop
        if ( (ballAcceleration*boxFacesNormal(hitPlane))<0  ) 
        {
			if(abs(ballPositionNew.z-boxFront.z)*(1.0/cameraCoef)< 1)
            {
                if (ballVelocityHit.normsqr()*(1.0/cameraCoef)< PRECISION)
                {
                    pauseSign = 1;
                    stopSign = 1;
                    cout<<"stop Position: "<<ballPositionHit*(1.0/cameraCoef)<<endl;
                    cout<<"stop VelocityHit: "<<ballVelocityHit*(1.0/cameraCoef)<<endl;
                
                }
                ballAcceleration.z = ballAcceleration.z * 0.7;
                
            }
        
		}
    }
    
    if (hitPlane == BACKPLAN) {
        double s = 0;
        s = (ballPosition.z- boxBack.z)/((ballPosition.z-boxBack.z)+(boxBack.z-ballPositionNew.z));
        ballVelocityHit = ballVelocity + ballAcceleration*(s*hStep);
        ballPositionHit = ballPosition + ballAcceleration*(s*hStep);
        cout<<"S= "<<s<<endl;
        cout<<"Collision Position: "<<ballPositionHit*(1.0/cameraCoef)<<endl;
        cout<<"Collision VelocityHit: "<<ballVelocityHit*(1.0/cameraCoef)<<endl;
        ballVelocityHit.z = -ballVelocityHit.z;
        ballVelocityNew = (ballVelocityHit + ballAcceleration*(1-s)*hStep)*BOUNCECOO;
        ballPositionNew = ballPositionHit + ballVelocityHit*(1-s)*hStep;
        
        //detect when to stop
        if ( (ballAcceleration*boxFacesNormal(hitPlane))<0  ) 
        {
			if(abs(ballPositionNew.z-boxBack.z)*(1.0/cameraCoef)< 1)
            {
                if (ballVelocityHit.normsqr()*(1.0/cameraCoef)< PRECISION)
                {
                    pauseSign = 1;
                    stopSign = 1;
                    cout<<"stop Position: "<<ballPositionHit*(1.0/cameraCoef)<<endl;
                    cout<<"stop VelocityHit: "<<ballVelocityHit*(1.0/cameraCoef)<<endl;
                
                }
                ballAcceleration.z = ballAcceleration.z * 0.9;
                
            }
        
		}
    }
    //ballAcceleration.z = ballAcceleration.z * BOUNCECOO; 
}

void calculatePosition()
{
    Vector3d ballPositionNew, ballVelocityNew;
    ballVelocityNew = ballVelocity + ballAcceleration*hStep;
    ballPositionNew = ballPosition + ballVelocity*hStep;
    
    if ((ballPositionNew.y-boxBottom.y)*(ballPosition.y-boxBottom.y) <=  0) {
        cout<<"Bottom Collision!!!"<<endl;
        hitCalculation(ballPosition, ballPositionNew, ballVelocity, ballVelocityNew, BOTTONPLAN);
    }
    
    if ((ballPositionNew.y-boxTop.y)*(ballPosition.y-boxTop.y) <=  0) {
        cout<<"Top Collision!!!"<<endl;
        hitCalculation(ballPosition, ballPositionNew, ballVelocity, ballVelocityNew, TOPPLAN);
    }
    
    if ((ballPositionNew.x-boxRight.x)*(ballPosition.x-boxRight.x) <=  0) {
        cout<<"Right Collision!!!"<<endl;
        hitCalculation(ballPosition, ballPositionNew, ballVelocity, ballVelocityNew, RIGHTPLAN);
    }
    
    if ((ballPositionNew.x-boxLeft.x)*(ballPosition.x-boxLeft.x) <=  0) {
        cout<<"Left Collision!!!"<<endl;
        hitCalculation(ballPosition, ballPositionNew, ballVelocity, ballVelocityNew, LEFTPLAN);
    }
    
    if ((ballPositionNew.z-boxFront.z)*(ballPosition.z-boxFront.z) <=  0) {
        cout<<"Front Collision!!!"<<endl;
        hitCalculation(ballPosition, ballPositionNew, ballVelocity, ballVelocityNew, FRONTPLAN);
    }

    if ((ballPositionNew.z-boxBack.z)*(ballPosition.z-boxBack.z) <=  0) {
        cout<<"Back Collision!!!"<<endl;
        hitCalculation(ballPosition, ballPositionNew, ballVelocity, ballVelocityNew, BACKPLAN);
    }
    
    ballVelocity = ballVelocityNew;
    ballPosition = ballPositionNew;
    if (tCount%(int)(1.0/hStep) == 0) {
        ballVelocity = ballVelocity * airRES;
    }
    
    tCount++;
}



void myDisplay(void)
{

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
    // draw the camera created in perspective
    camera->PerspectiveDisplay(WIDTH, HEIGHT);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

   //Setting lights 
    
    GLfloat light0_position[] = {300.0f, 0.0f, 300.0f, 1.0f}; //spot light
    GLfloat light1_position[] = {0.0f, 300.0f, 0.0f, 1.0f};
    GLfloat light0_ambient[]  = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat light0_diffuse[]  = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat light0_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};


    glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  light0_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light0_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);

    glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
    glLightfv(GL_LIGHT1, GL_AMBIENT,  light0_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  light0_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light0_specular);
    

    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHTING);

//Setting transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
//drawing a square box  
   GLfloat cube_mat_ambient[]  = {0.0f, 0.0f, 0.0f, 0.5f};
   GLfloat cube_mat_diffuse[]  = {0.5f, 0.5f, 0.5f, 0.5f};
   GLfloat cube_mat_specular[] = {0.5f, 0.5f, 0.5f, 0.5f};
   GLfloat cube_mat_emission[] = {0.3f, 0.3f, 0.3f, 0.5f};
   GLfloat cube_mat_shininess  = 20.0f;
  
   glMaterialfv(GL_FRONT, GL_AMBIENT,   cube_mat_ambient);
   glMaterialfv(GL_FRONT, GL_DIFFUSE,   cube_mat_diffuse);
   glMaterialfv(GL_FRONT, GL_SPECULAR,  cube_mat_specular);
   glMaterialfv(GL_FRONT, GL_EMISSION,  cube_mat_emission);
   glMaterialf (GL_FRONT, GL_SHININESS, cube_mat_shininess);

   glTranslatef(0.0f, 0.0f, 0.0f);
   if(cubeLine == 1)
    glutWireCube(boxEdge);
   else
    glutSolidCube(boxEdge);

//Setting ball material
   GLfloat ball_mat_ambient[]  = {0.0f, 0.0f, 0.0f, 0.7f};
   GLfloat ball_mat_diffuse[]  = {0.5f, 0.0f, 0.0f, 0.7f};
   GLfloat ball_mat_specular[] = {0.5f, 0.0f, 0.0f, 0.7f};
   GLfloat ball_mat_emission[] = {0.2f, 0.0f, 0.0f, 0.7f};
   GLfloat ball_mat_shininess  = 10.0f;

   glMaterialfv(GL_FRONT, GL_AMBIENT,   ball_mat_ambient);
   glMaterialfv(GL_FRONT, GL_DIFFUSE,   ball_mat_diffuse);
   glMaterialfv(GL_FRONT, GL_SPECULAR,  ball_mat_specular);
   glMaterialfv(GL_FRONT, GL_EMISSION,  ball_mat_emission);
   glMaterialf (GL_FRONT, GL_SHININESS, ball_mat_shininess);

//make a ball
    glTranslatef(ballPosition.x, ballPosition.y, ballPosition.z);
    glutSolidSphere(ballRadius, 20, 20); //radius of ball; number of faces creating a ball
    
    //glFlush();
    glutSwapBuffers();
}

void RestartBall()
{
    pauseSign = 0;
    LoadParameters(ParamFilename);
    glutPostRedisplay();
}


void timeProc(int id)
{
    if (id == 1) {
        while (1) {
            calculatePosition();
            if (tCount%(int)(1.0/hStep) == 0) {
                cout<<"Velocity: "<<ballVelocity*(1.0/cameraCoef)<<endl;
                cout<<"Postion:  "<<ballPosition*(1.0/cameraCoef)<<endl;
                break;
            }
        }
        glutPostRedisplay();
        if (pauseSign != 1) {
            glutTimerFunc(automaticSpeed,timeProc,1);
        }
        
    }
}


void handleKey(unsigned char key, int x, int y){
  
  switch(key){
    case 'm':       // 'c' - cycle to next color
    case 'M':
          if (stopSign == 1) {
              RestartBall();
              stopSign = 0;
          }
          while (1) {
              calculatePosition();
              if (tCount%(int)(1.0/hStep) == 0) {
                  cout<<"Velocity: "<<ballVelocity*(1.0/cameraCoef)<<endl;
                  cout<<"Postion:  "<<ballPosition*(1.0/cameraCoef)<<endl;
                  break;
              }
          }
    
    glutPostRedisplay();
      break;
          
    case 'r':
    case 'R':
        RestartBall();
    break;
          
    case 'a':
    case 'A':
          if (stopSign == 1) {
              RestartBall();
              stopSign = 0;
          }
          pauseSign = 0;
          glutTimerFunc(automaticSpeed, timeProc, 1);
    break;
          
    case 's':
    case 'S':
          pauseSign = 1;
    break;
      
    case 'q':       // q - quit
    case 'Q':
    case 27:        // esc - quit
      exit(0);
      
    default:        // not a valid key -- just ignore it
      return;
  }
}




void init() {
    LoadParameters(ParamFilename);
    // set up camera
    // parameters are eye point, aim point, up vector
    camera = new Camera(Vector3d(150, 100, 200), Vector3d(0, 0, 0), Vector3d(0, 1, 0));
    
    // black background for window
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);
    //glDepthRange(0, 1);
    
    //glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
}

void mouseEventHandler(int button, int state, int x, int y) {
    // let the camera handle some specific mouse events (similar to maya)
    camera->HandleMouseEvent(button, state, x, y);
    glutPostRedisplay();
}

void motionEventHandler(int x, int y) {
    // let the camera handle some mouse motions if the camera is to be moved
    camera->HandleMouseMotion(x, y);
    glutPostRedisplay();
}

int main(int argc, char *argv[])
{
    if(argc>1)
    {
        cout<<argv[1]<<endl;
        if(strcmp(argv[1], "wc\n")){    
            cubeLine = 1;
        }
        else {
            cout<<"wrong command!"<<endl;
        }

    }
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("bounce");
    
    init();
    
    glutDisplayFunc(myDisplay);
    glutMouseFunc(mouseEventHandler);
    glutMotionFunc(motionEventHandler);
    glutKeyboardFunc(handleKey);

    glutMainLoop();
    return 0;
}

