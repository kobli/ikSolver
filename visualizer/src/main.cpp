#include <iostream>
#include <irrlicht.h>
#include "solver.hpp"
using namespace std;
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

using Vector = vector2d<int>;
using Chain = ik::Chain<Vector>;
using Joint = ik::Joint<Vector>;

class MyEventReceiver : public IEventReceiver
{
	public:
		// This is the one method that we have to implement
		virtual bool OnEvent(const SEvent& event)
		{
			// Remember whether each key is down or up
			if (event.EventType == irr::EET_KEY_INPUT_EVENT)
				KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;

			return false;
		}

		// This is used to check whether a key is being held down
		virtual bool IsKeyDown(EKEY_CODE keyCode) const
		{
			return KeyIsDown[keyCode];
		}

		MyEventReceiver()
		{
			for (u32 i=0; i<KEY_KEY_CODES_COUNT; ++i)
				KeyIsDown[i] = false;
		}

	private:
		// We use this array to store the current state of each key
		bool KeyIsDown[KEY_KEY_CODES_COUNT];
};

void drawChain(IVideoDriver* driver, const Chain& chain) {
	const Joint* prevJoint = &chain.getJoint(0);
	for(unsigned i = 1; i < chain.jointCount(); ++i) {
		const Joint* nextJoint = &chain.getJoint(i);
		driver->draw2DLine(prevJoint->position, nextJoint->position);
		driver->draw2DPolygon(prevJoint->position, 10);
		prevJoint = nextJoint;
	}
}


int main()
{
	MyEventReceiver receiver;
	IrrlichtDevice *device =
		createDevice( video::EDT_OPENGL, dimension2d<u32>(1024, 768), 16, false, false, false, &receiver);
	if (!device)
		return 1;
	device->setWindowCaption(L"IK visualizer");
	device->setResizable(true);
	IVideoDriver* driver = device->getVideoDriver();
	ISceneManager* smgr = device->getSceneManager();
	IGUIEnvironment* guienv = device->getGUIEnvironment();

	Chain chain;
	chain.appendJoint({Vector(230)});
	chain.appendJoint({Vector(300, 150)});
	chain.appendJoint({Vector(350, 150)});
	chain.appendJoint({Vector(400, 200)});

	while(device->run())
	{
		driver->beginScene(true, true, SColor(255,100,101,140));

		drawChain(driver, chain);

		smgr->drawAll();
		guienv->drawAll();

		driver->endScene();
	}
	device->drop();
	return 0;
}
