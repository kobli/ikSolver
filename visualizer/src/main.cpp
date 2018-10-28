#include <irrlicht.h>
#include "solver.hpp"
using namespace std;
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

#define D2

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

#ifndef D3
using Vector = vector2d<float>;
using Chain = ik::Chain<Vector>;
using Joint = ik::Joint<Vector>;

void drawChain(IVideoDriver* driver, const Chain& chain) {
	const Joint* prevJoint = &chain.getJoint(0);
	for(unsigned i = 1; i < chain.jointCount(); ++i) {
		const Joint* nextJoint = &chain.getJoint(i);
		driver->draw2DLine(v2ftoi(prevJoint->position), v2ftoi(nextJoint->position));
		driver->draw2DPolygon(v2ftoi(prevJoint->position), 10);
		prevJoint = nextJoint;
	}
}

#else

using Vector = vector3d<float>;
using Chain = ik::Chain<Vector>;
using Joint = ik::Joint<Vector>;

void drawChain(IVideoDriver* driver, const Chain& chain) {
	const Joint* prevJoint = &chain.getJoint(0);
	for(unsigned i = 1; i < chain.jointCount(); ++i) {
		const Joint* nextJoint = &chain.getJoint(i);
		driver->draw3DLine(prevJoint->position, nextJoint->position);
		//driver->draw(v2ftoi(prevJoint->position), 10);
		prevJoint = nextJoint;
	}
}
#endif

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
#ifndef D3
	chain.appendJoint({Vector(230)});
	chain.appendJoint({Vector(300, 150)});
	chain.appendJoint({Vector(350, 150)});
	chain.appendJoint({Vector(400, 200)});
#else
	chain.appendJoint({Vector(230)});
	chain.appendJoint({Vector(300, 150, 100)});
	chain.appendJoint({Vector(350, 150, 150)});
	chain.appendJoint({Vector(400, 200, 200)});

	ICameraSceneNode* camera = smgr->addCameraSceneNodeFPS();
	camera->setTarget(vector3df(100));
#endif

	while(device->run())
	{
		driver->beginScene(true, true, SColor(255,100,101,140));
#ifndef D3
	if(receiver.LMBpressed)
		ik::FABRIK::solveChain(chain, chain.jointCount()-1, v2itof(device->getCursorControl()->getPosition()));
#else
		if(receiver.LMBpressed) {
			camera->setInputReceiverEnabled(false);
			device->getCursorControl()->setVisible(true);
			vector2d<int> cursorPos = device->getCursorControl()->getPosition();
			vector3df p{(float(cursorPos.X)/driver->getScreenSize().Width)*2-1, (1-float(cursorPos.Y)/driver->getScreenSize().Height)*2-1, 0};
			CMatrix4<float> VPMatrixInverse;
			(camera->getProjectionMatrix()*camera->getViewMatrix()).getInverse(VPMatrixInverse);
			VPMatrixInverse.transformVect(p);
			ik::FABRIK::solveChain(chain, chain.jointCount()-1, p);
		}
		else {
			camera->setInputReceiverEnabled(true);
			device->getCursorControl()->setVisible(false);
		}
#endif
		drawChain(driver, chain);
		smgr->drawAll();
		driver->endScene();
	}
	device->drop();
	return 0;
}
