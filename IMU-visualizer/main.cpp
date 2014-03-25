/*
 *  Name            :   main.cpp
 *  Function        :   10DOF visualizer
 *  Author          :   marald.mec
 *  Date            :   July, 2011
 */
  

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _USE_MATH_DEFINES 
#include <math.h>           

#include "include/GL/glut.h"          
#include "include/GL/freeglut_ext.h" 
#include "main.h" 
#include <windows.h>
#include <commdlg.h>
#include <assert.h>
#include <math.h>

char keytext[100]={0};
float xangle=0, yangle=0, zangle=0,altitude=0,temp=0;
FILE * fp;


void init(void)
{
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    glutInitWindowSize(WIDTH,HEIGHT);            
    glutInitWindowPosition(0,0);                  
    glShadeModel(GL_SMOOTH);   
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);  
    glClearDepth(1.0f);     
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);    
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  
    glutCreateWindow("10-DOF visualizer\t\t by Marald.mec");       
    glClearColor(0.1, 0.1, 0.13, 1.0);	
    glutIdleFunc(idle);
}



void printText(float x, float y, float z, float r, float g, float b)
{
    glColor3f(r, g, b);
    glRasterPos3f(x,y,z);
    for(int i=0;keytext[i]!='\0';i++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, keytext[i]);
    glutPostRedisplay();
}

void draw()
{
glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
glLoadIdentity();
glEnable(GL_DEPTH_TEST);
    
    float off= altitude/100,size=0.2,ssize=0.02,lsize=0.35,sizel=0.6,sizell=0.7;
    off=0;

     glLineWidth(2);
    glColor3f(1,1,1);
    glBegin(GL_LINES);
        glVertex3f(-0.8,0.8-sinf(xangle/180*M_PI)/7,0.8);
        glVertex3f(-0.4,0.8+sinf(xangle/180*M_PI)/7,0.8);
    glEnd();

    glBegin(GL_LINES);
        glVertex3f(-0.2,0.8-sinf(zangle/180*M_PI)/7,0.8);
        glVertex3f(0.2,0.8+sinf(zangle/180*M_PI)/7,0.8);
    glEnd();

    glBegin(GL_LINES);
        glVertex3f(0.4,0.8-sinf(yangle/180*M_PI)/7,0.8);
        glVertex3f(0.8,0.8+sinf(yangle/180*M_PI)/7,0.8);
    glEnd();

    gluLookAt(0,0,0, 45, 45, -90, 0, 1, 0);

    glColor3f(1,1,1);
    glBegin(GL_LINES);
     for(int i = 0; i < 370; i++)
                {
                    float xp = sizell * cosf(i * M_PI/180);
                    float yp = sizell * sinf(i * M_PI/180);
                    glVertex3f(xp,0,yp);
                }
    glEnd();

    glLineWidth(2);
    glColor3f(1,1,1);
    glBegin(GL_LINES);
        glVertex3f(0,-sizell,0);
        glVertex3f(0,sizell,0);
        glVertex3f(-sizell,0,0);
        glVertex3f(sizell,0,0);
        glVertex3f(0,0,-sizell);
        glVertex3f(0,0,sizell);
    glEnd();


    sprintf(keytext,"X-Angle: %.2f \t Y-Angle: %.2f \t Z-Angle: %.2f",xangle,yangle, zangle);
    printText(-0.6,-0.9,0,1,1,0.9);
    sprintf(keytext,"Altitude: %.2f M",altitude);
    printText(-0.18,-0.91,0,1,1,0.9);
    sprintf(keytext,"Temperature: %.2f C",temp);
    printText(-0.25,-1.0,0,1,1,0.9);
      sprintf(keytext,"X-Axis");
    printText(sizel+0.2,0,0,1,1,0.9);
      sprintf(keytext,"Y-Axis");
    printText(0,sizel+0.2,-0.14,1,1,0.9);
      sprintf(keytext,"Z-Axis");
    printText(0,0,-sizel-0.3,1,1,0.9);
       sprintf(keytext,"North");
    printText(0,0,sizel+0.16,1,1,0.9); 

    printf("X: %f Y: %f Z: %f A: %f T: %f\n\n", xangle, yangle,zangle, altitude, temp);

    glPushMatrix();
    glRotatef(xangle,1,0,0);
    glRotatef(yangle,0,1,0);
    glRotatef(zangle,0,0,1);

    glBegin(GL_QUADS);
        glColor3f(1,0,0);
        glVertex3f(-size,-ssize+off,-lsize);
        glVertex3f(size,-ssize+off,-lsize);
        glVertex3f(size,ssize+off,-lsize);
        glVertex3f(-size,ssize+off,-lsize);
    glEnd();

     glBegin(GL_QUADS);
        glColor3f(1,0,0);
        glVertex3f(-size,-ssize+off,lsize);
        glVertex3f(size,-ssize+off,lsize);
        glVertex3f(size,ssize+off,lsize);
        glVertex3f(-size,ssize+off,lsize);
    glEnd();

    glBegin(GL_QUADS);
        glColor3f(1,0,0);
        glVertex3f(size,ssize+off,lsize);
        glVertex3f(size,ssize+off,-lsize);
        glVertex3f(size,-ssize+off,-lsize);
        glVertex3f(size,-ssize+off,lsize);
    glEnd();

    glBegin(GL_QUADS);
        glColor3f(1,0,0);
        glVertex3f(-size,ssize+off,lsize);
        glVertex3f(-size,ssize+off,-lsize);
        glVertex3f(-size,-ssize+off,-lsize);
        glVertex3f(-size,-ssize+off,lsize);
    glEnd();

    glBegin(GL_QUADS);
        glColor3f(1,0,0);
        glVertex3f(size,-ssize+off,lsize);
        glVertex3f(-size,-ssize+off,lsize);
        glVertex3f(-size,-ssize+off,-lsize);
        glVertex3f(size,-ssize+off,-lsize);
        glColor3f(0,0,0.3);
        glVertex3f(ssize,-ssize+off-0.01,lsize/2);
        glVertex3f(-ssize,-ssize+off-0.01,lsize/2);
        glVertex3f(-ssize,-ssize+off-0.01,-lsize);
        glVertex3f(ssize,-ssize+off-0.01,-lsize);
    glEnd();

    glBegin(GL_QUADS);
        glColor3f(0.6,0.6,0.6);
        glVertex3f(size,ssize+off,lsize);
        glVertex3f(-size,ssize+off,lsize);
        glVertex3f(-size,ssize+off,-lsize);
        glVertex3f(size,ssize+off,-lsize);
        glColor3f(0,0,0.3);
        glVertex3f(ssize,ssize+off+0.01,lsize/2);
        glVertex3f(-ssize,ssize+off+0.01,lsize/2);
        glVertex3f(-ssize,ssize+off+0.01,-lsize);
        glVertex3f(ssize,ssize+off+0.01,-lsize);
    glEnd();

    glBegin(GL_POLYGON);
        glVertex3f(0,ssize+off+0.001,lsize);
        glVertex3f(-size/3*2,ssize+off+0.001,lsize/2);
        glVertex3f(size/3*2,ssize+off+0.001,lsize/2);
    glEnd();

    glBegin(GL_POLYGON);
        glVertex3f(0,-ssize+off-0.001,lsize);
        glVertex3f(-size/3*2,-ssize+off-0.001,lsize/2);
        glVertex3f(size/3*2,-ssize+off-0.001,lsize/2);
    glEnd();


    glLineWidth(2);
    glColor3f(0,0,0.15);
    glBegin(GL_LINE_LOOP);
        glVertex3f(-size,-ssize+off,-lsize);
        glVertex3f(size,-ssize+off,-lsize);
        glVertex3f(size,ssize+off,-lsize);
        glVertex3f(-size,ssize+off,-lsize);
    glEnd();

    glBegin(GL_LINE_LOOP);
        glVertex3f(-size,-ssize+off,lsize);
        glVertex3f(size,-ssize+off,lsize);
        glVertex3f(size,ssize+off,lsize);
        glVertex3f(-size,ssize+off,lsize);
    glEnd();

    glBegin(GL_LINE_LOOP);
        glVertex3f(size,ssize+off,lsize);
        glVertex3f(size,ssize+off,-lsize);
        glVertex3f(size,-ssize+off,-lsize);
        glVertex3f(size,-ssize+off,lsize);
    glEnd();

    glBegin(GL_LINE_LOOP);
        glVertex3f(-size,ssize+off,lsize);
        glVertex3f(-size,ssize+off,-lsize);
        glVertex3f(-size,-ssize+off,-lsize);
        glVertex3f(-size,-ssize+off,lsize);
    glEnd();

    glBegin(GL_LINE_LOOP);
        glVertex3f(size,-ssize+off,lsize);
        glVertex3f(-size,-ssize+off,lsize);
        glVertex3f(-size,-ssize+off,-lsize);
        glVertex3f(size,-ssize+off,-lsize);
    glEnd();

    glBegin(GL_LINE_LOOP);
        glVertex3f(size,ssize+off,lsize);
        glVertex3f(-size,ssize+off,lsize);
        glVertex3f(-size,ssize+off,-lsize);
        glVertex3f(size,ssize+off,-lsize);
    glEnd();

    glutSwapBuffers();
    glPopMatrix();

glFlush();
}


void idle()
{ 
        char x[10]={0},y[10]={0},z[10]={0},a[10]={0},t[10]={0};
        readSerial(x,y,z,a,t);
        xangle=(((float)(atoi(x))));
        yangle=(((float)(atoi(z))));
        zangle=(((float)(atoi(y))));
        altitude=(((float)(atoi(a))));
        temp=(((float)(atoi(t))));
        xangle/=-10;
        yangle/=10;
        zangle/=-10;
        altitude/=100;
        altitude-=60;
        temp/=1000;
        fprintf(fp,"%f %f %f %f %f\n", xangle, yangle,zangle, altitude, temp);
        draw();
}

void openSerial()
{
    DCB dcb;
    BOOL fSuccess;
    char com[32];
    char coms[32]="\\\\.\\";
    COMMTIMEOUTS timeouts = {0};

    printf("Enter the COM port the IMU is connected to: ");
    scanf(" %s", &com);
    strcat(coms,com);
    printf("Opening %s...\n", com);

    hCom = CreateFileA(coms, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, NULL, NULL);          //Open com-port

    //Set timeouts for the com
    timeouts.ReadTotalTimeoutConstant=10;
    timeouts.ReadTotalTimeoutMultiplier=1000;
    timeouts.WriteTotalTimeoutMultiplier=10;
    timeouts.WriteTotalTimeoutConstant=100;
     if(!SetCommTimeouts(hCom, &timeouts))
      printf ("Setting timeouts failed with error %d.\n", GetLastError());
    
    dcb.DCBlength = sizeof (dcb); 
    //Some debugging functions
    if (hCom == INVALID_HANDLE_VALUE)                   
        printf ("Opening serial port failed with error %d.\n", GetLastError());
    fSuccess = GetCommState(hCom, &dcb);
    if (!fSuccess) 
        printf ("Reading serial port settings failed with error %d.\n", GetLastError());

    // Change the DCB structure settings.
    dcb.BaudRate=CBR_115200;   
    dcb.fBinary = TRUE;            
    dcb.fParity = FALSE;             
    dcb.fOutxCtsFlow = FALSE;          
    dcb.fOutxDsrFlow = FALSE;         
    dcb.fDtrControl = DTR_CONTROL_DISABLE; 
    dcb.fDsrSensitivity = FALSE;      
    dcb.fTXContinueOnXoff = TRUE;      
    dcb.fOutX = FALSE;                
    dcb.fInX = FALSE;                  
    dcb.fErrorChar = FALSE;           
    dcb.fNull = FALSE;                
    dcb.fRtsControl = RTS_CONTROL_ENABLE; 
    dcb.fAbortOnError = FALSE;        
    dcb.ByteSize = 8;                
    dcb.Parity = NOPARITY;            
    dcb.StopBits = ONESTOPBIT;                   

    //Apply properties
    fSuccess = SetCommState(hCom, &dcb);
    if (!fSuccess) 
        printf ("Initialising serial port failed with error %d.\n", GetLastError());
    else
        printf ("Opening serial port %s succesfull!\n", com);


    if((fp = fopen("IMU.txt","w")) == 0)
        printf("Cannot create output file!\n");
    else
        printf("Output file created succesfull!\n");

    fprintf(fp,"10-DOF output file. Angle: X\tY\tZ\tA\n.");

}

void closeSerial()
{
    CloseHandle(hCom);
    fclose(fp);
}

int readSerial(char x[], char y[], char z[], char a[], char t[])
{
    DWORD nb;

    int i=0;
    while(x[0]!='X')
        ReadFile(hCom,x,1,&nb,0);
    
    do{
        ReadFile(hCom,&x[i],1,&nb,0);
        i++;
    } while(x[i-1]!='Y');

    i=0;
   do{
        ReadFile(hCom,&y[i],1,&nb,0);
        i++;
    } while(y[i-1]!='Z');

   i=0;
   do{
        ReadFile(hCom,&z[i],1,&nb,0);
        i++;
    } while(z[i-1]!='A');

    i=0;
   do{
        ReadFile(hCom,&a[i],1,&nb,0);
        i++;
    } while(a[i-1]!='T');
   i=0;
   do{
        ReadFile(hCom,&t[i],1,&nb,0);
        i++;
    } while(t[i-1]!='N');


    return nb;
}

int main(int argc, char* argv[])
{
    glutInit(&argc, argv);
    openSerial();
    init();
    glutMainLoop();
    return 0;
}

