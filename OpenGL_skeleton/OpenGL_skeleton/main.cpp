#include <stdlib.h>
#include <stdio.h>
#include <glut.h>
#include <Box2D/Box2D.h>

//global variables
int scr_width = 640;
int scr_height = 640;
b2World* world;
b2Body* ground;
b2Body* box;
b2EdgeShape gnd_shape;		//error occur
b2PolygonShape boxshape;	//error occur
int32 velocityIterations = 8;
int32 positionIterations = 3;

b2Body* body_line;
b2EdgeShape es;
b2Body* body_box;
b2PolygonShape ps;

float32 g_hz = 60.0f;
float32 timeStep = 1.0f / g_hz;

//Function declaraions
void display();
void keyboard(unsigned char k, int x, int y);
void mouse(int button, int state, int x, int y);
void reshape(int w, int h);
void Setup();
void Update(int value);

//------------------------------------------------------------------------------
int main(int argc, char* argv[]) {
	glutInitWindowSize(scr_width, scr_height);
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutCreateWindow("Rigid Body Simulation");

	Setup();

	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);

	glutReshapeFunc(reshape);
	glutTimerFunc(20, Update, 0);

	glutMainLoop();

	return 0;
}
//------------------------------------------------------------------------------
void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLoadIdentity();
	gluPerspective(60, 320.0 / 480.0, 1.0, 80.0);
	
	gluLookAt(0, 0, 21, 0, 0, 0, 0, 1, 0);

	b2Vec2 pos;	float angle = 0.0f;		pos = body_line->GetPosition();	angle = body_line->GetAngle();	glMatrixMode(GL_MODELVIEW);	glPushMatrix();	glTranslatef(pos.x, pos.y, 0.0f);	glRotatef(angle, 0.0f, 0.0f, 1.0f);	glColor3f(0.8f, 0.3f, 0.3f);	glLineWidth(2.0f);	glBegin(GL_LINES);	glVertex2d(es.m_vertex1.x, es.m_vertex1.y);	glVertex2d(es.m_vertex2.x, es.m_vertex2.y);	glEnd();	glPopMatrix();	
	
	pos = body_box->GetPosition();
	angle = body_box->GetAngle();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glTranslatef(pos.x, pos.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);
	glColor3f(1.0f, 1.0f, 1.0f);

	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(ps.m_vertices[i].x, ps.m_vertices[i].y);
	}
	glEnd();
	glPopMatrix();
	
	glutSwapBuffers();
	


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
void Setup() {
	b2Vec2 gravity;
	gravity.Set(0.0f, -10.0f);

	world = new b2World(gravity);
	{
		b2BodyDef gnd_bd;
		ground = world->CreateBody(&gnd_bd);
		gnd_shape.Set(b2Vec2(-25.0f, 0.0f), b2Vec2(25.0f, 0.0f));
		ground->CreateFixture(&gnd_shape, 0.0f);
	}
	/*b2BodyDef boxbd;
	boxbd.type = b2_dynamicBody;

	boxbd.position.Set(0.0f, 30.0f);
	boxbd.position.Set(1.0f, 30.0f);
	
	b2Body* body = world->CreateBody(&boxbd);

	boxshape.SetAsBox(5.0f, 5.0f);

	b2FixtureDef boxfd;
	boxfd.shape = &boxshape;
	boxfd.density = 1.0f;

	body->CreateFixture(&boxfd);
	box = body;*/

	b2BodyDef bd_line;
	body_line = world->CreateBody(&bd_line);
	b2Vec2 beginPoint, endPoint;
	beginPoint.Set(-40.0f, 0.0f);
	endPoint.Set(40.0f,0.0f);
	es.Set(beginPoint, endPoint);
	b2FixtureDef fd_line;
	fd_line.shape = &es;
	body_line->CreateFixture(&fd_line);


	b2BodyDef bd_box;
	bd_box.type = b2_dynamicBody;
	bd_box.position.Set(0.0f, 5.0f);
	body_box = world->CreateBody(&bd_box);
	ps.SetAsBox(2.0f, 1.0f);
	b2FixtureDef fd_box;
	fd_box.shape = &ps;
	fd_box.density = 1.0f;
	fd_box.friction = 0.3f;
	fd_box.restitution = 0.5f;

	body_box->CreateFixture(&fd_box);
	
}
void Update(int value) {
	world->Step(timeStep, velocityIterations, positionIterations);

	/*b2Vec2 position = box->GetPosition();
	printf("Box position ( %d , %d )\n" ,&position.x , &position.y);
	*/
	glutPostRedisplay();
	glutTimerFunc(20, Update, 0);
}