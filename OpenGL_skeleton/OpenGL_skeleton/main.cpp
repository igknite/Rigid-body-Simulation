#include <stdlib.h>
#include <stdio.h>
#include <glut.h>
#include <Box2D/Box2D.h>
#define M_PI 3.1415926535897932384626433

//global variables
int scr_width = 1280;
int scr_height = 720;
b2World* world;
//b2Body* ground;
//b2Body* box;
b2PulleyJoint* m_joint;
b2EdgeShape gnd_shape;
b2PolygonShape boxshape;
int32 velocityIterations = 30;
int32 positionIterations = 30;

b2Body* player;
//b2PolygonShape ps;
b2CircleShape ps;
b2PolygonShape Dps[100];
b2Body* Dbox[100];
int numbox = 0;
b2PolygonShape angleshape[2];
b2Body* anglebox[2];
b2RevoluteJoint* Rjoint[4];
b2DistanceJoint* Djoint[2];

bool keyin[3]; 
/*
0 : jump(space)
1 : left(a)
2 : right(d)
*/
float32 g_hz = 30.0f;
float32 timeStep = 1.0f / g_hz;

//Function declaraions
void display();
void keyboard(unsigned char k, int x, int y);
void upkeyboard(unsigned char k, int x, int y);
void mouse(int button, int state, int x, int y);
void reshape(int w, int h);
void Setup();
void Update(int value);
void moveplayer();
float32 raycast(b2Vec2 position);
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
	glutKeyboardUpFunc(upkeyboard);
	glutMouseFunc(mouse);

	glutReshapeFunc(reshape);
	glutTimerFunc(20, Update, 0);

	glutMainLoop();

	return 0;
}
//------------------------------------------------------------------------------
float32 raycast(b2Vec2 position) {
	float32 closestFraction = 1.0f;
	printf("%f %f\n", position.x, position.y);
	b2RayCastInput ppos;
	ppos.p1 = position;
	ppos.p2 = b2Vec2(position.x, position.y - 1.5f);
	ppos.maxFraction = 1.0f;
	for (b2Body* b = world->GetBodyList(); b; b = b->GetNext()) {
		for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext()) {
			b2RayCastOutput output;
			if (!f->RayCast(&output, ppos, 0)) {
				continue;
			}
			if (output.fraction < closestFraction) {
				closestFraction = output.fraction;
			}
		}
	}
	return closestFraction;
}
void moveplayer() {
	int x_force = 30;
	int y_force = 700;

	if (keyin[1])
		player->ApplyForce(b2Vec2(-x_force, 0), player->GetWorldCenter(), true);

	if (keyin[2])
		player->ApplyForce(b2Vec2(x_force, 0), player->GetWorldCenter(), true);

	if (keyin[0]) {
		if (raycast(player->GetWorldCenter()) < 0.6f) {
			player->ApplyForce(b2Vec2(0, y_force), player->GetWorldCenter(), true); // gravity > y_force...
		}
	}
	
	if (player->GetLinearVelocity().x > 10.0f)
		player->SetLinearVelocity(b2Vec2(10.0f, player->GetLinearVelocity().y));
	if (player->GetLinearVelocity().x < -10.0f)
		player->SetLinearVelocity(b2Vec2(-10.0f, player->GetLinearVelocity().y));
}
void display()
{
	moveplayer();

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
		angle = angle * 180.0 / M_PI;
		//Dbox[i]->SetFixedRotation(true);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glTranslatef(pos.x, pos.y, 0.0f);
		glRotatef(angle, 0.0f, 0.0f, 1.0f);
		if (i >29) glColor3f(0.7f, 0.5f, 0.1f);
		else glColor3f(0.8f, 0.3f, 0.1f);
		glBegin(GL_QUADS);
		b2PolygonShape* shape = (b2PolygonShape*)(Dbox[i]->GetFixtureList()->GetShape());
		for (int j = 0; j < 4; j++) {
			//glVertex2f(Dps[i].m_vertices[j].x, Dps[i].m_vertices[j].y);
			glVertex2f(shape->m_vertices[j].x, shape->m_vertices[j].y);
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
	
	//Draw Pulleys string
	glPushMatrix();
	glColor3f(0.8f, 0.8f, 0.8f);
	glLineWidth(1.0f);
	glBegin(GL_LINE_STRIP);
	glVertex2d(m_joint->GetAnchorA().x, m_joint->GetAnchorA().y);
	glVertex2d(m_joint->GetGroundAnchorA().x, m_joint->GetGroundAnchorA().y);
	glVertex2d(m_joint->GetGroundAnchorB().x, m_joint->GetGroundAnchorB().y);
	glVertex2d(m_joint->GetAnchorB().x, m_joint->GetAnchorB().y);
	glEnd();
	glPopMatrix();

	//Draw distance joint string
	glPushMatrix();
	glColor3f(0.1f, 1.0f, 1.0f);
	glLineWidth(1.0f);
	glBegin(GL_LINE_STRIP);
	glVertex2d(Djoint[0]->GetAnchorA().x, Djoint[0]->GetAnchorA().y);
	glVertex2d(Djoint[0]->GetAnchorB().x, Djoint[0]->GetAnchorB().y);
	glVertex2d(Djoint[1]->GetAnchorA().x, Djoint[1]->GetAnchorA().y);
	glVertex2d(Djoint[1]->GetAnchorB().x, Djoint[1]->GetAnchorB().y);

	glEnd();
	glPopMatrix();

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

	if (key == ' ')
		keyin[0] = true;
	
	if (key == 'a')
		keyin[1] = true;
	
	if (key == 'd')
		keyin[2] = true;

}
void upkeyboard(unsigned char key, int x, int y) {
	if (key == ' ')
		keyin[0] = false;

	if (key == 'a')
		keyin[1] = false;

	if (key == 'd')
		keyin[2] = false;
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
	bd_player.position.Set(388.0f, 25.0f);
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
	makebox(500.0f, 40.0f, 0.3f, 40.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// 오른쪽 벽 
	makebox(23.0f, 3.0f, 1.5f, 3.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// scene1 계단블록 시작
	makebox(33.0f, 7.0f, 1.5f, 7.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// 좌측부터 -5

	makebox(43.0f, 11.5f, 1.5f, 11.8f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// 순서대로
	makebox(53.0f, 16.0f, 1.5f, 16.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene1 계단블록 마지막 
	makebox(33.0f, 63.0f, 25.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene1 상단길 
	makebox(135.0f, 59.4f, 0.5f, 20.9f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 왼쪽 세로벽 
	makebox(200.0f, 30.0f, 0.5f, 30.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 오른쪽 세로벽 -10

	makebox(160.0f, 39.0f, 25.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 왼쪽 가로벽
	makebox(175.48f, 60.0f, 25.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 오른쪽 가로벽
	makebox(230.0f, 8.0f, 50.0f, 8.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 물 바닥
	makebox(280.0f, 28.0f, 10.0f, 28.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 물 우측벽 수심 약 25.0f 
	makebox(307.0f, 57.0f, 0.5f, 22.9f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 왼쪽 위세로 벽 -15

	makebox(307.0f, 13.2f, 0.5f, 6.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 왼쪽 아래세로 벽
	makebox(332.0f, 19.0f, 25.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 가로벽  -17
	makebox(356.5f, 49.5f, 0.5f, 30.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 오른쪽 위 세로벽
	makebox(337.5f, 24.0f, 12.5f, 5.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 도르래 아래 박스
	makebox(346.0f, 53.0f, 4.0f, 16.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 도르래 위 박스 -20

	makebox(337.0f, 13.2f, 0.5f, 6.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 하단 길
	makebox(322.0f, 5.3f, 0.5f, 5.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// scene3 하단 길
	makebox(345.0f, 5.3f, 0.5f, 5.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// scene3 하단 길 
	makebox(363.0f, 33.5f, 6.0f, 15.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene4 왼쪽 박스
	makebox(393.0f, 35.0f, 7.0f, 35.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene4 오른쪽 박스 -25

	makebox(376.0f, 58.0f, 5.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// final scene 진입판
	makebox(423.0f, 42.0f, 6.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// final scene 왼쪽 길
	makebox(450.0f, 50.0f, 7.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// final scene 오른쪽 길 
	makebox(495.0f, 20.0f, 5.0f, 20.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// 골인지점 수심 40.0f -29


	makebox(43.0f, 42.0f, 6.0f, 0.5f, b2_dynamicBody, 30.0f, 0.5f, 0.5f);	// scene1 joint블록 시작 -30 	other color

	makebox(19.0f, 40.0f, 7.0f, 0.5f, b2_dynamicBody, 30.0f, 0.5f, 0.5f);	// 우측부터 순서대로 
	makebox(7.0f, 51.0f, 5.0f, 0.5f, b2_dynamicBody, 30.0f, 0.5f, 0.5f);	// scene1 joint블록 마지막
	makebox(43.0f, 42.0f, 0.0f, 0.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// 우측부터 joint블록 연결
	makebox(19.0f, 40.0f, 0.0f, 0.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);		//
	makebox(7.0f, 51.0f, 0.0f, 0.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// -35

	makebox(151.0f, 1.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);	// scene2 충돌박스
	makebox(151.0f, 3.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	makebox(151.0f, 5.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	makebox(151.0f, 7.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);	// 
	makebox(151.0f, 9.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	
	makebox(151.0f, 11.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	makebox(151.0f, 13.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	makebox(151.0f, 15.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	makebox(151.0f, 17.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);	
	makebox(151.0f, 19.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);	// -45
	
	makebox(151.0f, 21.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	makebox(151.0f, 23.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	makebox(199.0f, 70.0f, 1.5f, 9.3f, b2_dynamicBody, 0.02f, 0.5f, 0.7f);	// scene2 물에 빠지는 벽
	makebox(320.0f, 50.0f, 7.0f, 0.5f, b2_dynamicBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 그네 - 그네 코드 추가 필요 -45
	makebox(320.0f, 60.0f, 0.0f, 0.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 그네 joint

	makebox(353.0f, 33.0f, 2.95f, 5.0f, b2_dynamicBody, 5.0f, 0.2f, 0.5f);	// scene3 오른쪽 도르래 
	makebox(303.5f, 5.0f, 2.95f, 5.0f, b2_dynamicBody, 5.0f, 0.2f, 0.5f);	// scene3 왼쪽 도르래	
	makebox(378.0f, 31.0f, 4.0f, 0.5f, b2_dynamicBody, 5.0f, 0.5f, 0.5f);	// scene4 회전판
	makebox(378.0f, 31.0f, 0.0f, 0.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene4 회전판 joint -53


	b2RevoluteJointDef Rjointbox;
	Rjointbox.Initialize(Dbox[30], Dbox[33], Dbox[33]->GetWorldCenter());
	Rjointbox.lowerAngle = -0.5;
	Rjointbox.upperAngle = 0.5;
	Rjointbox.enableLimit = true;
	Rjoint[0] = (b2RevoluteJoint*)world->CreateJoint(&Rjointbox);
	Dbox[30]->SetAngularDamping(0.99f);
	Rjointbox.Initialize(Dbox[31], Dbox[34], Dbox[34]->GetWorldCenter());
	Rjoint[1] = (b2RevoluteJoint*)world->CreateJoint(&Rjointbox);
	Dbox[31]->SetAngularDamping(0.99f);
	Rjointbox.Initialize(Dbox[32], Dbox[35], Dbox[35]->GetWorldCenter());
	Rjoint[2] = (b2RevoluteJoint*)world->CreateJoint(&Rjointbox);
	Dbox[32]->SetAngularDamping(0.99f);

	Rjointbox.Initialize(Dbox[53], Dbox[54], Dbox[54]->GetWorldCenter());
	Rjointbox.motorSpeed = 80.0f;
	Rjointbox.enableMotor = true;
	Rjointbox.enableLimit = false;
	Rjointbox.maxMotorTorque = 5.0f;
	Rjoint[3] = (b2RevoluteJoint*)world->CreateJoint(&Rjointbox);
	//그네
	b2DistanceJointDef Djointbox;
	Djointbox.Initialize(Dbox[49], Dbox[50], b2Vec2 (Dbox[49]->GetPosition().x-7.0f,Dbox[49]->GetPosition().y), Dbox[50]->GetPosition());
	Djointbox.collideConnected = true;
	Djoint[0] = (b2DistanceJoint*)world->CreateJoint(&Djointbox);
	Djoint[0]->SetLength(12.0f);

	Djointbox.Initialize(Dbox[49], Dbox[50], b2Vec2(Dbox[49]->GetPosition().x + 7.0f, Dbox[49]->GetPosition().y), Dbox[50]->GetPosition());
	Djointbox.collideConnected = true;
	Djoint[1] = (b2DistanceJoint*)world->CreateJoint(&Djointbox);
	Djoint[1]->SetLength(12.0f);

	b2Vec2 groundanchor1, groundanchor2;
	b2Vec2 anchor1, anchor2;
	groundanchor1.Set(353.0f, 79.0f);
	groundanchor2.Set(303.5f, 79.0f);
	float short_length = 159.5f;
	float ratio = 1.0f;
	b2PulleyJointDef pulleyDef;
	anchor2.Set(303.5f, 10.0f);
	anchor1.Set(353.0f, 38.0f);
	pulleyDef.Initialize(Dbox[51], Dbox[52], groundanchor1, groundanchor2, anchor1, anchor2, ratio);
	m_joint = (b2PulleyJoint*)world->CreateJoint(&pulleyDef);

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
	printf("Box position ( %f , %f )\n", player->GetPosition().x, player->GetPosition().y);


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