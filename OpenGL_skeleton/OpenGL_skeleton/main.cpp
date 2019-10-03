#include <stdlib.h>
#include <stdio.h>
#include <glut.h>
#include <Box2D/Box2D.h>

//global variables
int scr_width = 1280;
int scr_height = 720;
b2World* world;
b2Body* ground;
//b2Body* box;
b2EdgeShape gnd_shape;
b2PolygonShape boxshape;
int32 velocityIterations = 8;
int32 positionIterations = 3;

b2Body* player;
b2PolygonShape ps;

b2PolygonShape Dps[100];
b2Body* Dbox[100];
int numbox = 0;

float32 g_hz = 30.0f;
float32 timeStep = 1.0f / g_hz;

//Function declaraions
void display();
void keyboard(unsigned char k, int x, int y);
void mouse(int button, int state, int x, int y);
void reshape(int w, int h);
void Setup();
void Update(int value);
b2Body* makebox(float32 x, float32 y, float32 w, float32 h, b2BodyType type_name);
b2Body* makebox(int boxnum, float32 x, float32 y, float32 w, float32 h, b2BodyType type_name, float32 density, float32 friction, float32 restitution);

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
	gluPerspective(50, 500.0 / 500.0, 1.0, 80.0);
	b2Vec2 position = player->GetPosition();
	b2Vec2 poss = position;
	if (poss.x < 35.0f) {
		poss.x = 35.0f;
	}
	if (poss.y < 35.0f) {
		poss.y = 35.0f;
	}
	if (poss.y > 50.0f) {
		poss.y = 50.0f;
	}
	glTranslatef(-poss.x, -poss.y, 0);
	gluLookAt(0, 0, 80, 0, 0, 0, 0, 1, 0);


	b2Vec2 pos;
	float angle = 0.0f;

	for (int i = 0; i < numbox; i++) {
		pos = Dbox[i]->GetPosition();
		angle = Dbox[i]->GetAngle();
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glTranslatef(pos.x, pos.y, 0.0f);
		glRotatef(angle, 0.0f, 0.0f, 1.0f);
		glColor3f(0.8f, 0.3f, 0.1f);
		glBegin(GL_QUADS);

		for (int j = 0; j < 4; j++) {
			glVertex2f(Dps[i].m_vertices[j].x, Dps[i].m_vertices[j].y);
		}
		glEnd();
		glPopMatrix();

	}


	pos = player->GetPosition();
	angle = player->GetAngle();
	player->SetFixedRotation(true);
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
void keyboard(unsigned char key, int x, int y) {
	int x_force = 100;
	int y_force = 60;

	/*if (key == 'w')
	player->ApplyForce(b2Vec2(0, y_force), player->GetWorldCenter(), true); // gravity > y_force...
	*/
	if (key == 'a')
		player->ApplyForce(b2Vec2(-x_force, 0), player->GetWorldCenter(), true);

	if (key == 's')
		player->ApplyForce(b2Vec2(0, -y_force), player->GetWorldCenter(), true);

	if (key == 'd')
		player->ApplyForce(b2Vec2(x_force, 0), player->GetWorldCenter(), true);
	if (key == ' ') {
		// if(collision == True){
		player->ApplyLinearImpulse(b2Vec2(0, y_force), player->GetWorldCenter(), true);
		printf("JUMP!");
	}

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
	gravity.Set(0.0f, -25.0f);

	world = new b2World(gravity);
	/*{
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


	b2BodyDef bd_box;
	bd_box.type = b2_dynamicBody;
	bd_box.position.Set(10.0f, 5.0f);
	player = world->CreateBody(&bd_box);
	ps.SetAsBox(0.5f, 1.0f);
	b2FixtureDef fd_box;
	fd_box.shape = &ps;
	fd_box.density = 1.0f;
	fd_box.friction = 0.3f;
	fd_box.restitution = 0.5f;

	player->CreateFixture(&fd_box);

	//makebox(float32 x, float32 y, float32 w, float32 h, b2BodyType type_name, float32 density, float32 friction, float32 restitution)
	makebox(200.0f, 0.0f, 200.0f, 0.3f, b2_staticBody);		// 바닥
	makebox(0.0f, 40.0f, 0.3f, 40.3f, b2_staticBody);			// 왼쪽 벽
	makebox(200.0f, 80.0f, 200.0f, 0.3f, b2_staticBody);		// 천장
	makebox(400.0f, 40.0f, 0.3f, 40.3f, b2_staticBody);		// 오른쪽 벽
	makebox(23.0f, 2.0f, 1.5f, 2.3f, b2_staticBody);			// scene1 계단블록 시작
	makebox(33.0f, 5.0f, 1.5f, 5.3f, b2_staticBody);			// 좌측부터
	makebox(43.0f, 10.0f, 1.5f, 10.3f, b2_staticBody);		// 순서대로
	makebox(53.0f, 13.0f, 1.5f, 13.3f, b2_staticBody);		// 	scene1 계단블록 마지막
	makebox(33.0f, 63.0f, 25.0f, 0.5f, b2_staticBody);		// scene1 상단길
	makebox(43.0f, 43.0f, 6.0f, 0.5f, b2_staticBody);			// scene1 joint블록 시작
	makebox(23.0f, 38.0f, 7.0f, 0.5f, b2_staticBody);			// 우측부터 순서대로
	makebox(8.0f, 49.0f, 5.0f, 0.5f, b2_staticBody);			// scene1 joint블록 마지막





}
void Update(int value) {
	world->Step(timeStep, velocityIterations, positionIterations);

	b2Vec2 position = player->GetPosition();
	printf("Box position ( %f , %f )\n", position.x, position.y);


	glutPostRedisplay();
	glutTimerFunc(20, Update, 0);
}
b2Body* makebox(int boxnum, float32 x, float32 y, float32 w, float32 h, b2BodyType type_name, float32 density, float32 friction, float32 restitution) {
	b2BodyDef temp;
	temp.type = type_name;
	temp.position.Set(x, y);
	Dbox[boxnum] = world->CreateBody(&temp);
	Dps[boxnum].SetAsBox(w, h);
	b2FixtureDef fd_box;
	fd_box.shape = &Dps[boxnum];
	fd_box.density = density;
	fd_box.friction = friction;
	fd_box.restitution = restitution;
	Dbox[boxnum]->CreateFixture(&fd_box);
	numbox++;
	return Dbox[boxnum];
}
b2Body* makebox(float32 x, float32 y, float32 w, float32 h, b2BodyType type_name) {
	return makebox(numbox, x, y, w, h, type_name, 1.0f, 0.3f, 0.5f);
}

