#include <stdlib.h>
#include <stdio.h>
#include <glut.h>
#include <Box2D/Box2D.h>
#define M_PI 3.1415926535897932384626433

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
//b2PolygonShape ps;
b2CircleShape ps;
b2PolygonShape Dps[100];
b2Body* Dbox[100];
int numbox = 0;
b2PolygonShape angleshape[2];
b2Body* anglebox[2];

float32 g_hz = 30.0f;
float32 timeStep = 1.0f / g_hz;

//Function declaraions
void display();
void keyboard(unsigned char k, int x, int y);
void mouse(int button, int state, int x, int y);
void reshape(int w, int h);
void Setup();
void Update(int value);
b2Body* makebox(float32 x, float32 y, float32 w, float32 h, b2BodyType type_name, float32 density, float32 friction, float32 restitution);
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
	gluPerspective(50, 500.0 / 350.0, 1.0, 80.0);
	b2Vec2 position = player->GetPosition();
	b2Vec2 poss = position;
	if (poss.x < 50.0f) {
		poss.x = 50.0f;
	}
	if (poss.x > 475.0f) {
		poss.x = 475.0f;
	}
	if (poss.y < 35.0f) {
		poss.y = 35.0f;
	}
	if (poss.y > 47.0f) {
		poss.y = 47.0f;
	}
	glTranslatef(-poss.x, -poss.y, 0);
	gluLookAt(0, 0, 80, 0, 0, 0, 0, 1, 0);


	b2Vec2 pos;
	float angle = 0.0f;

	for (int i = 0; i < numbox; i++) {
		pos = Dbox[i]->GetPosition();
		angle = Dbox[i]->GetAngle();
		//Dbox[i]->SetFixedRotation(true);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glTranslatef(pos.x, pos.y, 0.0f);
		glRotatef(angle, 0.0f, 0.0f, 1.0f);
		if (i >28) glColor3f(0.7f, 0.5f, 0.1f);
		else glColor3f(0.8f, 0.3f, 0.1f);
		glBegin(GL_QUADS);

		for (int j = 0; j < 4; j++) {
			glVertex2f(Dps[i].m_vertices[j].x, Dps[i].m_vertices[j].y);
		}
		glEnd();
		glPopMatrix();
	}
	for (int i = 0; i < 2; i++) {
		pos = anglebox[i]->GetPosition();
		angle = anglebox[i]->GetAngle();
		angle = angle * 180 / M_PI;
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glTranslatef(pos.x, pos.y, 0.0f);
		glRotatef(angle, 0.0f, 0.0f, 1.0f);
		glColor3f(0.8f, 0.3f, 0.1f);
		glBegin(GL_QUADS);
		for (int j = 0; j < 4; j++) {
			glVertex2f(angleshape[i].m_vertices[j].x, angleshape[i].m_vertices[j].y);
		}

		glEnd();
		glPopMatrix();
	}


	pos = player->GetPosition();
	angle = player->GetAngle();
	player->SetFixedRotation(true);
	glMatrixMode(GL_MODELVIEW);
	/*
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
	//box player
	*/
	glPushMatrix();
	glTranslatef(pos.x, pos.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);
	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_POLYGON);
	for (int i = 0; i < 360; i++)
	{
		float deg2Rad = i * M_PI / 180;
		glVertex2f(cos(deg2Rad) * ps.m_radius, sin(deg2Rad) * ps.m_radius);
	}
	glEnd();
	glPopMatrix();
	//ball player

	glutSwapBuffers();



}
//------------------------------------------------------------------------------
void keyboard(unsigned char key, int x, int y) {
	int x_force = 30;
	int y_force = 22;

	/*if (key == 'w')
	player->ApplyForce(b2Vec2(0, y_force), player->GetWorldCenter(), true); // gravity > y_force...
	*/
	if (key == 'a')
		player->ApplyForce(b2Vec2(-x_force, 0), player->GetWorldCenter(), true);
	
	/*if (key == 's')
		player->ApplyForce(b2Vec2(0, -y_force), player->GetWorldCenter(), true);
		*/
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


	b2BodyDef bd_player;
	bd_player.type = b2_dynamicBody;
	bd_player.position.Set(60.0f, 75.0f);
	player = world->CreateBody(&bd_player);
	ps.m_radius = 0.5f; //ballplayer needs
	//ps.SetAsBox(0.5f, 1.0f); //boxplayer needs
	b2FixtureDef fd_player;
	fd_player.shape = &ps;
	fd_player.density = 1.0f;
	fd_player.friction = 0.3f;
	fd_player.restitution = 0.5f;
	player->CreateFixture(&fd_player);

	//makebox(float32 x, float32 y, float32 w, float32 h, b2BodyType type_name)
	makebox(250.0f, 0.0f, 250.0f, 0.3f, b2_staticBody,1.0f, 0.5f, 0.5f);	// 바닥 -0
	makebox(0.0f, 40.0f, 0.3f, 40.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// 왼쪽 벽
	makebox(250.0f, 80.0f, 250.0f, 0.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// 천장
	makebox(500.0f, 40.0f, 0.3f, 40.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// 오른쪽 벽 -3
	makebox(23.0f, 3.0f, 1.5f, 3.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// scene1 계단블록 시작
	makebox(33.0f, 7.0f, 1.5f, 7.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// 좌측부터
	makebox(43.0f, 11.5f, 1.5f, 11.8f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// 순서대로
	makebox(53.0f, 16.0f, 1.5f, 16.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene1 계단블록 마지막 -7
	makebox(33.0f, 63.0f, 25.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene1 상단길 -8
	makebox(135.0f, 59.4f, 0.5f, 20.9f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 왼쪽 세로벽 -9
	makebox(200.0f, 30.0f, 0.5f, 30.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 오른쪽 세로벽
	makebox(160.0f, 39.0f, 25.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 왼쪽 가로벽
	makebox(175.48f, 60.0f, 25.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 오른쪽 가로벽
	makebox(230.0f, 8.0f, 50.0f, 8.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 물 바닥
	makebox(280.0f, 28.0f, 10.0f, 28.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 물 우측벽 수심 약 25.0f -14
	makebox(307.0f, 57.0f, 0.5f, 22.9f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 왼쪽 위세로 벽
	makebox(307.0f, 13.2f, 0.5f, 6.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 왼쪽 아래세로 벽
	makebox(332.0f, 19.0f, 25.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 가로벽  -17
	makebox(356.5f, 49.5f, 0.5f, 30.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 오른쪽 위 세로벽
	makebox(337.5f, 24.0f, 12.5f, 5.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 도르래 아래 박스
	makebox(346.0f, 55.0f, 4.0f, 15.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 도르래 위 박스 -20
	makebox(337.0f, 13.2f, 0.5f, 6.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 하단 길
	makebox(322.0f, 5.3f, 0.5f, 5.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// scene3 하단 길
	makebox(345.0f, 5.3f, 0.5f, 5.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// scene3 하단 길 -23
	makebox(363.0f, 33.5f, 6.0f, 15.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene4 왼쪽 박스
	makebox(393.0f, 35.0f, 7.0f, 35.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene4 오른쪽 박스
	makebox(423.0f, 32.0f, 6.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// final scene 왼쪽 길
	makebox(450.0f, 50.0f, 7.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// final scene 오른쪽 길 
	makebox(495.0f, 20.0f, 5.0f, 20.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// 골인지점 수심 30.0f -28
	makebox(43.0f, 42.0f, 6.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// scene1 joint블록 시작  	other color
	makebox(19.0f, 40.0f, 7.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// 우측부터 순서대로		other color
	makebox(7.0f, 51.0f, 5.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// scene1 joint블록 마지막 	other color
	makebox(199.0f, 70.0f, 0.5f, 9.5f, b2_dynamicBody, 0.05f, 0.5f, 0.2f);	// scene2 물에 빠지는 벽 	other color
	makebox(151.0f, 1.0f, 1.0f, 1.0f, b2_dynamicBody, 0.05f, 0.1f, 0.2f);	// scene2 충돌박스 -33~40	other color
	makebox(151.0f, 3.0f, 1.0f, 1.0f, b2_dynamicBody, 0.05f, 0.1f, 0.2f);
	makebox(151.0f, 5.0f, 1.0f, 1.0f, b2_dynamicBody, 0.05f, 0.1f, 0.2f);
	makebox(151.0f, 7.0f, 1.0f, 1.0f, b2_dynamicBody, 0.05f, 0.1f, 0.2f);
	makebox(151.0f, 9.0f, 1.0f, 1.0f, b2_dynamicBody, 0.05f, 0.1f, 0.2f);
	makebox(151.0f, 11.0f, 1.0f, 1.0f, b2_dynamicBody, 0.05f, 0.1f, 0.2f);
	makebox(151.0f, 13.0f, 1.0f, 1.0f, b2_dynamicBody, 0.05f, 0.1f, 0.2f);
	makebox(151.0f, 15.0f, 1.0f, 1.0f, b2_dynamicBody, 0.05f, 0.1f, 0.2f);
	makebox(151.0f, 17.0f, 1.0f, 1.0f, b2_dynamicBody, 0.05f, 0.1f, 0.2f);
	makebox(151.0f, 19.0f, 1.0f, 1.0f, b2_dynamicBody, 0.05f, 0.1f, 0.2f);
	makebox(151.0f, 21.0f, 1.0f, 1.0f, b2_dynamicBody, 0.05f, 0.1f, 0.2f);
	makebox(151.0f, 23.0f, 1.0f, 1.0f, b2_dynamicBody, 0.05f, 0.1f, 0.2f);
	makebox(320.0f, 50.0f, 7.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 그네 - 그네 코드 추가 필요 other color
	makebox(378.0f, 31.0f, 4.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene4 회전판 -30	other color
	
	// anglebody : scene1 경사로(0 : 하단 1: 상단)
	b2BodyDef anglebody[2];
	b2FixtureDef angle_box[2];
	anglebody[0].type = b2_staticBody;
	anglebody[0].position.Set(89.5f, 31.2f);
	anglebody[0].angle = -45 * M_PI / 180;
	anglebox[0] = world->CreateBody(&anglebody[0]);
	angleshape[0].SetAsBox(45.0f, 0.5f);
	angle_box[0].shape = &angleshape[0];
	angle_box[0].density = 1.0f;
	angle_box[0].friction = 0.5f;
	angle_box[0].restitution = 0.5f;
	anglebox[0]->CreateFixture(&angle_box[0]);

	anglebody[1].type = b2_staticBody;
	anglebody[1].position.Set(89.5f, 48.8f);
	anglebody[1].angle = -45 * M_PI / 180;
	anglebox[1] = world->CreateBody(&anglebody[1]);
	angleshape[1].SetAsBox(45.0f, 0.5f);
	angle_box[1].shape = &angleshape[1];
	angle_box[1].density = 1.0f;
	angle_box[1].friction = 0.5f;
	angle_box[1].restitution = 0.5f;
	anglebox[1]->CreateFixture(&angle_box[1]);

	
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
	b2FixtureDef fd_player;
	fd_player.shape = &Dps[boxnum];
	fd_player.density = density;
	fd_player.friction = friction;
	fd_player.restitution = restitution;
	Dbox[boxnum]->CreateFixture(&fd_player);
	numbox++;
	return Dbox[boxnum];
}
b2Body* makebox(float32 x, float32 y, float32 w, float32 h, b2BodyType type_name, float32 density, float32 friction, float32 restitution) {
	return makebox(numbox, x, y, w, h, type_name, density, friction, restitution);
}