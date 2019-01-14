#include <random>
#include <irrlicht.h>
#include "solver.hpp"

static const float DisplayOrientationSize = 10;
using namespace std;
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

vector2d<int> v2ftoi(const vector2df& v) {
	return vector2d<int>(int(v.X), int(v.Y));
}

vector2df v2itof(const vector2d<int>& v) {
	return vector2df(v.X, v.Y);
}

vector3d<int> v3ftoi(const vector3df& v) {
	return vector3d<int>(int(v.X), int(v.Y), int(v.Z));
}

vector3df v3itof(const vector3d<int>& v) {
	return vector3df(v.X, v.Y, v.Z);
}

vector3df vTov3f(const Vector& v) {
	return {v.x, v.y, v.z};
}

Vector v3fTov(const vector3df& v) {
	return {v.X, v.Y, v.Z};
}


using Chain = ik::Chain;
using Joint = ik::Joint;

void drawChain(IVideoDriver* driver, const Chain& chain) {
	driver->setTransform(video::ETS_WORLD, core::IdentityMatrix);
	const Joint* prevJoint = &chain.getJoint(0/*chain.baseJointID()*/);
	for(unsigned i = /*chain.baseJointID()+*/1; i < chain.jointCount(); ++i) {
		const Joint* nextJoint = &chain.getJoint(i);
		driver->draw3DLine(vTov3f(prevJoint->position), vTov3f(nextJoint->position));
		driver->draw3DLine(vTov3f(prevJoint->position), vTov3f(prevJoint->position+prevJoint->orientation*DisplayOrientationSize));
		//driver->draw(v2ftoi(prevJoint->position), 10);
		prevJoint = nextJoint;
	}
	driver->draw3DLine(vTov3f(prevJoint->position), vTov3f(prevJoint->position+prevJoint->orientation*DisplayOrientationSize));
}

void drawJointRotationConstraints(IVideoDriver* driver, const Chain& chain) {
	random_device rd{};
	mt19937 gen{rd()};
	normal_distribution<> d{0,1};

	for(unsigned i = 1; i < chain.jointCount()-1; ++i) {
		const Joint& j = chain.getJoint(i);
		Vector jointPos = j.position;
		float boneLength = (jointPos-chain.getJoint(i+1).position).length();

		const unsigned sampleC = 1000;
		for(unsigned ii = 0; ii < sampleC; ++ii) {
			// pick a random point on a sphere around the joint
			Vector p;
			p.x = d(gen);
			p.y = d(gen);
			p.z = d(gen);
			if(p.length() != 0) {
				p.normalize();
				p = jointPos + p*boneLength;
				Vector prevBoneDir = (jointPos-chain.getJoint(i-1).position).normalize();
				Vector restrictedP = ik::FABRIK::constrainedJointRotation(&j, p, prevBoneDir);
				const float pointConcentration = 0.1;
				restrictedP = jointPos + (restrictedP-jointPos)*pointConcentration;
				driver->draw3DLine(vTov3f(restrictedP), vTov3f(restrictedP+0.1));
			}
		}
	}
}

class MyEventReceiver : public IEventReceiver
{
	public:
		virtual bool OnEvent(const SEvent& event)
		{
			if(event.EventType == irr::EET_MOUSE_INPUT_EVENT)
				LMBpressed = event.MouseInput.isLeftPressed();
			return false;
		}
		
		bool LMBpressed = false;
};


int main()
{
	MyEventReceiver receiver;
	IrrlichtDevice *device = createDevice( video::EDT_OPENGL, dimension2d<u32>(1024, 768), 16, false, false, false, &receiver);
	if (!device)
		return 1;
	device->setWindowCaption(L"IK visualizer");
	device->setResizable(true);
	IVideoDriver* driver = device->getVideoDriver();
	ISceneManager* smgr = device->getSceneManager();

	Chain chain;
	chain.appendJoint({Vector(0)				, Vector(1, 0, 0), 0, 0, 0,0,0,0});
	chain.appendJoint({Vector(0, 150, 0), Vector(1, 0, 0), 0, M_PI, M_PI_2,M_PI_2,0,0});
	chain.appendJoint({Vector(0, 250, 0), Vector(1, 0, 0), 0, 0, M_PI,0,M_PI,0});
	//chain.appendJoint({Vector(0, 350, 0), Vector(1, 0, 0), 0, 0, });
	//chain.appendJoint({Vector(0, 450, 0), Vector(1, 0, 0), 0, 0, });
	//chain.appendJoint({Vector(0, 500, 0), Vector(1, 0, 0), 0, 0, });

	ICameraSceneNode* camera = smgr->addCameraSceneNodeFPS();
	camera->setPosition(vector3df(0,0,1000));
	camera->setTarget(vector3df(0));

	smgr->addCubeSceneNode(40);
	smgr->addLightSceneNode(nullptr, vector3df(100, 0, 0), SColorf(1, 0.5, 0.5), 200);

	while(device->run())
	{
		driver->beginScene(true, true, SColor(255,100,101,140));
		if(receiver.LMBpressed) {
			camera->setInputReceiverEnabled(false);
			device->getCursorControl()->setVisible(true);
			line3d<f32> rayFromCursor = smgr->getSceneCollisionManager()->getRayFromScreenCoordinates(device->getCursorControl()->getPosition());
			float d = 300;
			vector3df p = rayFromCursor.start + rayFromCursor.getVector().normalize()*d;
			ik::FABRIK::solveChain(chain, chain.jointCount()-1, v3fTov(p), {0,1,0});
		}
		else {
			camera->setInputReceiverEnabled(true);
			device->getCursorControl()->setVisible(false);
		}
		drawChain(driver, chain);
		drawJointRotationConstraints(driver, chain);
		smgr->drawAll();
		driver->endScene();
	}
	device->drop();
	return 0;
}
