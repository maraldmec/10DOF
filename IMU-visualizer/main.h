#ifndef MAIN_H
#define MAIN_H

const int WIDTH = 800;
const int HEIGHT = 800;
const int DEPTH = 800;

int main(int argc, char* argv[]);
void init(void);

void printText(float x, float y, float z, float r, float g, float b);
void draw();
void idle();

//Serial functions
int readSerial(char x[], char y[], char z[], char a[], char t[]);
static HANDLE hCom;
void openSerial();
void closeSerial();


#endif