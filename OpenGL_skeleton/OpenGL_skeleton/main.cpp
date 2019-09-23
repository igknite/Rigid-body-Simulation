#include <stdio.h>
#include <glut.h>
#include <Box2D.h>


//Function declaraions
void display();
void keyboard(unsigned char k, int x, int y);
void mouse(int button, int state, int x, int y);
void reshape(int w, int h);


//------------------------------------------------------------------------------
int main(int argc, char* argv[]) {
	
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DEPTH);
	glutCreateWindow("OpenGL");

	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutReshapeFunc(reshape);

	glutMainLoop();

	return 0;
}
//------------------------------------------------------------------------------
void display()
{
	


}
//------------------------------------------------------------------------------
void keyboard(unsigned char key, int x, int y)
{
	


}
//-----------------------------------------------------------------------------
void mouse(int button, int state, int mx, int my)
{
	


}
//------------------------------------------------------------------------------
void reshape(int w, int h)
{
	


}