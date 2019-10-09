#include <stdlib.h>
#include <stdio.h>
#include <glut.h>
#include <Box2D/Box2D.h>
#include <set>
#include <vector>
#define M_PI 3.1415926535897932384626433
using namespace std;

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
b2Body* waterbox[2];
b2PolygonShape watershape[2];
int numbox = 0;
b2PolygonShape angleshape[2];
b2Body* anglebox[2];
b2RevoluteJoint* Rjoint[3];
b2RevoluteJoint* RjointM;

b2DistanceJoint* Djoint[6];

bool keyin[3];
/*
0 : jump(space)
1 : left(a)
2 : right(d)
*/
float32 g_hz = 30.0f;
float32 timeStep = 1.0f / g_hz;
typedef std::pair<b2Fixture*, b2Fixture*> fixturePair;
std::set<fixturePair> m_fixturePairs;
//Function declaraions
void display();
void keyboard(unsigned char k, int x, int y);
void upkeyboard(unsigned char k, int x, int y);
void mouse(int button, int state, int x, int y);
void reshape(int w, int h);
void Setup();
void Update(int value);
void moveplayer();

bool inside(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 p);
b2Vec2 intersection(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 s, b2Vec2 e);
bool findIntersectionOfFixtures(b2Fixture* fA, b2Fixture* fB, vector<b2Vec2>& outputVertices);
b2Vec2 ComputeCentroid(vector<b2Vec2> vs, float& area);

float32 raycast(b2Vec2 position);
b2Body* makebox(float32 x, float32 y, float32 w, float32 h, b2BodyType type_name, float32 density, float32 friction, float32 restitution);
b2Body* makebox(int boxnum, float32 x, float32 y, float32 w, float32 h, b2BodyType type_name, float32 density, float32 friction, float32 restitution);

//------------------------------------------------------------------------------
class b2ContactListener_ : public b2ContactListener {
public:

	void BeginContact(b2Contact* contact)
	{
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();
		if (fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody)
			m_fixturePairs.insert(make_pair(fixtureA, fixtureB));
		else if (fixtureB->IsSensor() && fixtureA->GetBody()->GetType() == b2_dynamicBody)
			m_fixturePairs.insert(make_pair(fixtureB, fixtureA));
	}

	void EndContact(b2Contact* contact)
	{
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody)
			m_fixturePairs.erase(make_pair(fixtureA, fixtureB));
		else if (fixtureB->IsSensor() && fixtureA->GetBody()->GetType() == b2_dynamicBody)
			m_fixturePairs.erase(make_pair(fixtureB, fixtureA));
	}
	void Presolve(b2Contact* contact) {};
	void PostSlove(b2Contact* contact) {};
};

int main(int argc, char* argv[]) {
	glutInitWindowSize(scr_width, scr_height);
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutCreateWindow("Rigid Body Simulation");
	b2ContactListener_ listener = b2ContactListener_();

	Setup();
	world->SetContactListener(&listener);

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
			player->ApplyForce(b2Vec2(0, y_force), player->GetWorldCenter(), true);
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
	//Draw box
	for (int i = 0; i < numbox; i++) {
		pos = Dbox[i]->GetPosition();
		angle = Dbox[i]->GetAngle();
		angle = angle * 180.0 / M_PI;
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glTranslatef(pos.x, pos.y, 0.0f);
		glRotatef(angle, 0.0f, 0.0f, 1.0f);
		if (i >27) glColor3f(0.7f, 0.5f, 0.1f);
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
	// scene1 -> scene2 road
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
	for (int i = 0; i < 5; i = i+2) {
		glPushMatrix();
		glColor3f(0.1f, 1.0f, 1.0f);
		glLineWidth(1.0f);
		glBegin(GL_LINE_STRIP);
		glVertex2d(Djoint[i]->GetAnchorA().x, Djoint[i]->GetAnchorA().y);
		glVertex2d(Djoint[i]->GetAnchorB().x, Djoint[i]->GetAnchorB().y);
		glVertex2d(Djoint[i+1]->GetAnchorA().x, Djoint[i+1]->GetAnchorA().y);
		glVertex2d(Djoint[i+1]->GetAnchorB().x, Djoint[i+1]->GetAnchorB().y);

		glEnd();
		glPopMatrix();
	}
	//player
	pos = player->GetPosition();
	angle = player->GetAngle();
	player->SetFixedRotation(true);
	glMatrixMode(GL_MODELVIEW);
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

	//water box
	for (int i = 0; i < 2; i++) {
		pos = waterbox[i]->GetPosition();
		angle = waterbox[i]->GetAngle();
		angle = angle * 180 / M_PI;
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glTranslatef(pos.x, pos.y, 0.0f);
		glRotatef(angle, 0.0f, 0.0f, 1.0f);
		glColor4f(0.1f, 0.1f, 1.0f,0.3f);
		glBegin(GL_QUADS);
		for (int j = 0; j < 4; j++) {
			glVertex2f(watershape[i].m_vertices[j].x, watershape[i].m_vertices[j].y);
		}

		glEnd();
		glPopMatrix();
	}

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
	bd_player.position.Set(3.0f, 5.0f);
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
	makebox(250.0f, 0.0f, 250.0f, 0.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// 바닥 -0

	makebox(0.0f, 40.0f, 0.3f, 40.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// 왼쪽 벽
	makebox(250.0f, 80.0f, 250.0f, 0.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// 천장
	makebox(500.0f, 40.0f, 0.3f, 40.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// 오른쪽 벽 
	makebox(23.0f, 3.0f, 1.5f, 3.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// scene1 계단블록 시작
	makebox(33.0f, 7.0f, 1.5f, 7.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// 좌측부터 -5

	makebox(43.5f, 11.5f, 2.0f, 11.8f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// 순서대로
	makebox(54.0f, 16.0f, 2.5f, 16.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene1 계단블록 마지막 
	makebox(33.0f, 63.0f, 25.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene1 상단길 
	makebox(135.0f, 59.4f, 0.5f, 20.9f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 왼쪽 세로벽 
	makebox(200.0f, 30.0f, 0.5f, 30.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 오른쪽 세로벽 -10

	makebox(160.0f, 39.0f, 25.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 왼쪽 가로벽
	makebox(175.48f, 60.0f, 25.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 오른쪽 가로벽
	makebox(230.0f, 8.0f, 50.0f, 8.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 물 바닥
	makebox(280.0f, 28.0f, 10.0f, 28.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene2 물 우측벽 수심 약 25.0f 
	makebox(307.0f, 57.0f, 0.5f, 22.9f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 왼쪽 위세로 벽 -15

	makebox(307.0f, 13.2f, 0.5f, 6.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 왼쪽 아래세로 벽
	makebox(332.0f, 19.0f, 25.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 가로벽
	makebox(356.5f, 49.5f, 0.5f, 30.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 오른쪽 위 세로벽
	makebox(337.5f, 24.0f, 12.5f, 5.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 도르래 아래 박스
	makebox(346.0f, 53.0f, 4.0f, 16.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 도르래 위 박스 -20

	makebox(337.0f, 13.2f, 0.5f, 6.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 하단 길
	makebox(322.0f, 5.3f, 0.5f, 5.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// scene3 하단 길
	makebox(345.0f, 5.3f, 0.5f, 5.3f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// scene3 하단 길 
	makebox(363.0f, 33.5f, 6.0f, 15.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene4 왼쪽 박스
	makebox(393.0f, 35.0f, 7.0f, 35.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene4 오른쪽 박스 -25

	makebox(376.0f, 58.0f, 5.0f, 0.5f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// final scene 진입판
	makebox(495.0f, 20.0f, 5.0f, 20.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// 골인지점 수심 40.0f -27
	makebox(43.0f, 42.0f, 6.0f, 0.5f, b2_dynamicBody, 1.0f, 0.5f, 0.5f);	// scene1 joint블록 시작 other color
	makebox(19.0f, 40.0f, 7.0f, 0.5f, b2_dynamicBody, 1.0f, 0.5f, 0.5f);	// 우측부터 순서대로 
	makebox(7.0f, 51.0f, 5.0f, 0.5f, b2_dynamicBody, 1.0f, 0.5f, 0.5f);	// scene1 joint블록 마지막 -30

	makebox(43.0f, 42.0f, 0.0f, 0.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// 우측부터 joint블록 연결
	makebox(19.0f, 40.0f, 0.0f, 0.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);		//
	makebox(7.0f, 51.0f, 0.0f, 0.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);		// joint 블록 연결 
	makebox(151.0f, 1.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);	// scene2 충돌박스
	makebox(151.0f, 3.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);	// -35

	makebox(151.0f, 5.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	makebox(151.0f, 7.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);	
	makebox(151.0f, 9.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);	
	makebox(151.0f, 11.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	makebox(151.0f, 13.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);	// -40

	makebox(151.0f, 15.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	makebox(151.0f, 17.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	makebox(151.0f, 19.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);	
	makebox(151.0f, 21.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);
	makebox(151.0f, 23.0f, 1.0f, 1.0f, b2_dynamicBody, 0.02f, 0.1f, 0.7f);	// -45

	makebox(199.0f, 70.0f, 1.5f, 9.3f, b2_dynamicBody, 0.2f, 0.5f, 0.7f);	// scene2 물에 빠지는 벽(치고 지나감)
	makebox(320.0f, 50.0f, 7.0f, 0.5f, b2_dynamicBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 그네
	makebox(320.0f, 60.0f, 0.0f, 0.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene3 중간 그네 joint
	makebox(353.0f, 33.0f, 2.95f, 5.0f, b2_dynamicBody, 2.0f, 0.2f, 0.5f);	// scene3 오른쪽 도르래 
	makebox(303.5f, 5.0f, 2.95f, 5.0f, b2_dynamicBody, 2.0f, 0.2f, 0.5f);	// scene3 왼쪽 도르래	-50

	makebox(378.0f, 31.0f, 5.0f, 0.5f, b2_dynamicBody, 6.0f, 0.5f, 0.5f);	// scene4 회전판
	makebox(378.0f, 31.0f, 0.0f, 0.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// scene4 회전판 joint
	makebox(235.0f, 70.0f, 3.0f, 4.0f, b2_dynamicBody, 0.2f, 0.5f, 0.7f);	// scene2 물에 빠지는 블록(중간) 
	makebox(255.0f, 70.0f, 3.0f, 4.0f, b2_dynamicBody, 0.2f, 0.5f, 0.7f);	// scene2 물에 빠지는 블록(오른쪽)
	makebox(421.0f, 42.0f, 8.0f, 0.5f, b2_dynamicBody, 1.0f, 0.5f, 0.5f);	// final scene 왼쪽 길 -55

	makebox(454.0f, 50.0f, 8.0f, 0.5f, b2_dynamicBody, 1.0f, 0.5f, 0.5f);	// final scene 오른쪽 길
	makebox(421.0f, 80.0f, 0.0f, 0.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// final scene 왼쪽 길 joint
	makebox(454.0f, 80.0f, 0.0f, 0.0f, b2_staticBody, 1.0f, 0.5f, 0.5f);	// final scene 오른쪽 길 joint -58


	//시소
	b2RevoluteJointDef Rjointbox;
	Rjointbox.Initialize(Dbox[30], Dbox[33], Dbox[33]->GetWorldCenter());
	Rjointbox.lowerAngle = -0.5;
	Rjointbox.upperAngle = 0.5;
	Rjointbox.enableLimit = true;
	Rjoint[0] = (b2RevoluteJoint*)world->CreateJoint(&Rjointbox);
	Dbox[30]->SetAngularDamping(0.99f);
	Rjointbox.Initialize(Dbox[28], Dbox[31], Dbox[31]->GetWorldCenter());
	Rjoint[1] = (b2RevoluteJoint*)world->CreateJoint(&Rjointbox);
	Dbox[31]->SetAngularDamping(0.99f);
	Rjointbox.Initialize(Dbox[29], Dbox[32], Dbox[32]->GetWorldCenter());
	Rjoint[2] = (b2RevoluteJoint*)world->CreateJoint(&Rjointbox);
	Dbox[32]->SetAngularDamping(0.99f);

	//회전판
	b2RevoluteJointDef Rjointmotor;
	Rjointmotor.Initialize(Dbox[51], Dbox[52], Dbox[52]->GetWorldCenter());
	Rjointmotor.enableMotor = true;
	//Rjointmotor.enableLimit = true;
	Rjointmotor.motorSpeed = 3.0f;
	Rjointmotor.maxMotorTorque = 1000.0f;
	RjointM = (b2RevoluteJoint*)world->CreateJoint(&Rjointmotor);

	//scene 3 그네
	b2DistanceJointDef Djointbox;
	Djointbox.Initialize(Dbox[47], Dbox[48], b2Vec2(Dbox[47]->GetPosition().x - 7.0f, Dbox[47]->GetPosition().y), Dbox[48]->GetPosition());
	Djointbox.collideConnected = true;
	Djoint[0] = (b2DistanceJoint*)world->CreateJoint(&Djointbox);
	Djoint[0]->SetLength(12.0f);
	Djointbox.Initialize(Dbox[47], Dbox[48], b2Vec2(Dbox[47]->GetPosition().x + 7.0f, Dbox[47]->GetPosition().y), Dbox[48]->GetPosition());
	Djointbox.collideConnected = true;
	Djoint[1] = (b2DistanceJoint*)world->CreateJoint(&Djointbox);
	Djoint[1]->SetLength(12.0f);

	//final scene 그네
	Djointbox.Initialize(Dbox[55], Dbox[57], b2Vec2(Dbox[55]->GetPosition().x - 8.0f, Dbox[55]->GetPosition().y), Dbox[57]->GetPosition());
	Djointbox.collideConnected = true;
	Djoint[2] = (b2DistanceJoint*)world->CreateJoint(&Djointbox);
	Djoint[2]->SetLength(32.0f);
	Djointbox.Initialize(Dbox[55], Dbox[57], b2Vec2(Dbox[55]->GetPosition().x + 8.0f, Dbox[55]->GetPosition().y), Dbox[57]->GetPosition());
	Djointbox.collideConnected = true;
	Djoint[3] = (b2DistanceJoint*)world->CreateJoint(&Djointbox);
	Djoint[3]->SetLength(32.0f);

	Djointbox.Initialize(Dbox[56], Dbox[58], b2Vec2(Dbox[56]->GetPosition().x - 8.0f, Dbox[56]->GetPosition().y), Dbox[58]->GetPosition());
	Djointbox.collideConnected = true;
	Djoint[4] = (b2DistanceJoint*)world->CreateJoint(&Djointbox);
	Djoint[4]->SetLength(27.0f);
	Djointbox.Initialize(Dbox[56], Dbox[58], b2Vec2(Dbox[56]->GetPosition().x + 8.0f, Dbox[56]->GetPosition().y), Dbox[58]->GetPosition());
	Djointbox.collideConnected = true;
	Djoint[5] = (b2DistanceJoint*)world->CreateJoint(&Djointbox);
	Djoint[5]->SetLength(27.0f);



	//도르래
	b2Vec2 groundanchor1, groundanchor2;
	b2Vec2 anchor1, anchor2;
	groundanchor1.Set(353.0f, 79.0f);
	groundanchor2.Set(303.5f, 79.0f);
	float short_length = 159.5f;
	float ratio = 1.0f;
	b2PulleyJointDef pulleyDef;
	anchor2.Set(303.5f, 10.0f);
	anchor1.Set(353.0f, 38.0f);
	pulleyDef.Initialize(Dbox[49], Dbox[50], groundanchor1, groundanchor2, anchor1, anchor2, ratio);
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
	//water box
	b2BodyDef waterbody[2];
	b2FixtureDef water_box[2];
	water_box[0].isSensor = true;
	waterbody[0].type = b2_staticBody;
	waterbody[0].position.Set(235.25f, 28.5f);
	waterbox[0] = world->CreateBody(&waterbody[0]);
	watershape[0].SetAsBox(34.75f, 12.5f);
	water_box[0].shape = &watershape[0];
	water_box[0].density = 0.5f;
	water_box[0].friction = 0.5f;
	water_box[0].restitution = 0.5f;
	waterbox[0]->CreateFixture(&water_box[0]);

	water_box[1].isSensor = true;
	waterbody[1].type = b2_staticBody;
	waterbody[1].position.Set(445.0f, 19.5f);
	waterbox[1] = world->CreateBody(&waterbody[1]);
	watershape[1].SetAsBox(45.0f, 19.0f);
	water_box[1].shape = &watershape[1];
	water_box[1].density = 0.5f;
	water_box[1].friction = 0.5f;
	water_box[1].restitution = 0.5f;
	waterbox[1]->CreateFixture(&water_box[1]);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

}
void Update(int value) {
	world->Step(timeStep, velocityIterations, positionIterations);

	b2Vec2 position = player->GetPosition();
	printf("Box position ( %f , %f )\n", player->GetPosition().x, player->GetPosition().y);

	set<fixturePair>::iterator it = m_fixturePairs.begin();
	set<fixturePair>::iterator end = m_fixturePairs.end();
	while (it != end) {

		//fixtureA is the fluid
		b2Fixture* fixtureA = it->first;
		b2Fixture* fixtureB = it->second;

		float density = fixtureA->GetDensity();

		vector<b2Vec2> intersectionPoints;
		if (findIntersectionOfFixtures(fixtureA, fixtureB, intersectionPoints)) {

			//find centroid
			float area = 0;
			b2Vec2 centroid = ComputeCentroid(intersectionPoints, area);

			b2Vec2 gravity(0.0f, -25.0f);
			float displacedMass = fixtureA->GetDensity() * area;
			b2Vec2 buoyancyForce = displacedMass * -gravity;
			fixtureB->GetBody()->ApplyForce(buoyancyForce, centroid, true);
			
			//find relative velocity between object and fluid
			b2Vec2 velDir = fixtureB->GetBody()->GetLinearVelocityFromWorldPoint(centroid) -
				fixtureA->GetBody()->GetLinearVelocityFromWorldPoint(centroid);
			float vel = velDir.Normalize();

			//apply simple linear drag
			float dragMag = fixtureA->GetDensity() * vel * vel;
			b2Vec2 dragForce = dragMag * -velDir;
			fixtureB->GetBody()->ApplyForce(dragForce, centroid, true);
			//apply simple angular drag
			float angularDrag = area * -fixtureB->GetBody()->GetAngularVelocity();
			fixtureB->GetBody()->ApplyTorque(angularDrag, true);
		}

		++it;
	}


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

bool inside(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 p) {
	return (cp2.x - cp1.x)*(p.y - cp1.y) > (cp2.y - cp1.y)*(p.x - cp1.x);
}

b2Vec2 intersection(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 s, b2Vec2 e) {
	b2Vec2 dc(cp1.x - cp2.x, cp1.y - cp2.y);
	b2Vec2 dp(s.x - e.x, s.y - e.y);
	float n1 = cp1.x * cp2.y - cp1.y * cp2.x;
	float n2 = s.x * e.y - s.y * e.x;
	float n3 = 1.0 / (dc.x * dp.y - dc.y * dp.x);
	return b2Vec2((n1*dp.x - n2 * dc.x) * n3, (n1*dp.y - n2 * dc.y) * n3);
}


//Note that this only works when fB is a convex polygon, but we know all 
//fixtures in Box2D are convex, so that will not be a problem
bool findIntersectionOfFixtures(b2Fixture* fA, b2Fixture* fB, vector<b2Vec2>& outputVertices)
{
	//currently this only handles polygon vs polygon
	if (fA->GetShape()->GetType() != b2Shape::e_polygon ||
		fB->GetShape()->GetType() != b2Shape::e_polygon)
		return false;

	b2PolygonShape* polyA = (b2PolygonShape*)fA->GetShape();
	b2PolygonShape* polyB = (b2PolygonShape*)fB->GetShape();

	//fill subject polygon from fixtureA polygon
	for (int i = 0; i < polyA->m_count; i++)
		outputVertices.push_back(fA->GetBody()->GetWorldPoint(polyA->m_vertices[i]));

	//fill clip polygon from fixtureB polygon
	vector<b2Vec2> clipPolygon;
	for (int i = 0; i < polyB->m_count; i++)
		clipPolygon.push_back(fB->GetBody()->GetWorldPoint(polyB->m_vertices[i]));

	b2Vec2 cp1 = clipPolygon[clipPolygon.size() - 1];
	for (int j = 0; j < clipPolygon.size(); j++) {
		b2Vec2 cp2 = clipPolygon[j];
		if (outputVertices.empty())
			return false;
		vector<b2Vec2> inputList = outputVertices;
		outputVertices.clear();
		b2Vec2 s = inputList[inputList.size() - 1]; //last on the input list
		for (int i = 0; i < inputList.size(); i++) {
			b2Vec2 e = inputList[i];
			if (inside(cp1, cp2, e)) {
				if (!inside(cp1, cp2, s)) {
					outputVertices.push_back(intersection(cp1, cp2, s, e));
				}
				outputVertices.push_back(e);
			}
			else if (inside(cp1, cp2, s)) {
				outputVertices.push_back(intersection(cp1, cp2, s, e));
			}
			s = e;
		}
		cp1 = cp2;
	}

	return !outputVertices.empty();
}
b2Vec2 ComputeCentroid(vector<b2Vec2> vs, float& area)
{
	int count = (int)vs.size();
	b2Assert(count >= 3);

	b2Vec2 c;
	c.Set(0.0f, 0.0f);
	area = 0.0f;

	// pRef is the reference point for forming triangles.
	// Its location doesnt change the result (except for rounding error).
	b2Vec2 pRef(0.0f, 0.0f);

	const float32 inv3 = 1.0f / 3.0f;

	for (int32 i = 0; i < count; ++i)
	{
		// Triangle vertices.
		b2Vec2 p1 = pRef;
		b2Vec2 p2 = vs[i];
		b2Vec2 p3 = i + 1 < count ? vs[i + 1] : vs[0];

		b2Vec2 e1 = p2 - p1;
		b2Vec2 e2 = p3 - p1;

		float32 D = b2Cross(e1, e2);

		float32 triangleArea = 0.5f * D;
		area += triangleArea;

		// Area weighted centroid
		c += triangleArea * inv3 * (p1 + p2 + p3);
	}

	// Centroid
	if (area > b2_epsilon)
		c *= 1.0f / area;
	else
		area = 0;
	return c;
}
