#include <irrlicht.h>
#include "solver.hpp"
using namespace std;
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

using Vector = vector2d<float>;
using Chain = ik::Chain<Vector>;
using Joint = ik::Joint<Vector>;

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

vector2d<int> v2ftoi(const vector2df& v) {
	return vector2d<int>(int(v.X), int(v.Y));
}

vector2df v2itof(const vector2d<int>& v) {
	return vector2df(v.X, v.Y);
}

void drawChain(IVideoDriver* driver, const Chain& chain) {
	const Joint* prevJoint = &chain.getJoint(0);
	for(unsigned i = 1; i < chain.jointCount(); ++i) {
		const Joint* nextJoint = &chain.getJoint(i);
		driver->draw2DLine(v2ftoi(prevJoint->position), v2ftoi(nextJoint->position));
		driver->draw2DPolygon(v2ftoi(prevJoint->position), 10);
		prevJoint = nextJoint;
	}
}


int main()
{
	MyEventReceiver receiver;
	IrrlichtDevice *device = createDevice( video::EDT_OPENGL, dimension2d<u32>(1024, 768), 16, false, false, false, &receiver);
	if (!device)
		return 1;
	device->setWindowCaption(L"IK visualizer");
	device->setResizable(true);
	IVideoDriver* driver = device->getVideoDriver();

	Chain chain;
	chain.appendJoint({Vector(230)});
	chain.appendJoint({Vector(300, 150)});
	chain.appendJoint({Vector(350, 150)});
	chain.appendJoint({Vector(400, 200)});

	while(device->run())
	{
		driver->beginScene(true, true, SColor(255,100,101,140));
		if(receiver.LMBpressed)
			ik::FABRIK::solveChain(chain, chain.jointCount()-1, v2itof(device->getCursorControl()->getPosition()));
		drawChain(driver, chain);
		driver->endScene();
	}
	device->drop();
	return 0;
}
